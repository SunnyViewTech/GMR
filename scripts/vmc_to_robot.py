import argparse
import asyncio
import queue
import signal
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer


VMC_TO_GMR_NAME_MAP: Dict[str, str] = {
    "Hips": "Hips",
    "Spine": "Spine",
    "Chest": "Spine1",
    "Neck": "Neck",
    "Head": "Head",
    "LeftUpperLeg": "LeftUpLeg",
    "LeftLowerLeg": "LeftLeg",
    "LeftFoot": "LeftFoot",
    "LeftToes": "LeftToeBase",
    "RightUpperLeg": "RightUpLeg",
    "RightLowerLeg": "RightLeg",
    "RightFoot": "RightFoot",
    "RightToes": "RightToeBase",
    "LeftShoulder": "LeftShoulder",
    "LeftUpperArm": "LeftArm",
    "LeftLowerArm": "LeftForeArm",
    "LeftHand": "LeftHand",
    "RightShoulder": "RightShoulder",
    "RightUpperArm": "RightArm",
    "RightLowerArm": "RightForeArm",
    "RightHand": "RightHand",
}

VMC_PARENT_CANDIDATES: Dict[str, List[str]] = {
    "Hips": [],
    "Spine": ["Hips"],
    "Chest": ["Spine"],
    "Neck": ["Chest", "Spine"],
    "Head": ["Neck", "Chest"],
    "LeftShoulder": ["Chest", "Spine"],
    "LeftUpperArm": ["LeftShoulder"],
    "LeftLowerArm": ["LeftUpperArm"],
    "LeftHand": ["LeftLowerArm"],
    "RightShoulder": ["Chest", "Spine"],
    "RightUpperArm": ["RightShoulder"],
    "RightLowerArm": ["RightUpperArm"],
    "RightHand": ["RightLowerArm"],
    "LeftUpperLeg": ["Hips"],
    "LeftLowerLeg": ["LeftUpperLeg"],
    "LeftFoot": ["LeftLowerLeg"],
    "LeftToes": ["LeftFoot"],
    "RightUpperLeg": ["Hips"],
    "RightLowerLeg": ["RightUpperLeg"],
    "RightFoot": ["RightLowerLeg"],
    "RightToes": ["RightFoot"],
}



VMC_BONE_FALLBACKS: Dict[str, str] = {}



@dataclass
class CoordinateTransformer:
    """Affine helper for axis swaps / reflections when moving from Unity to GMR."""

    matrix: np.ndarray
    inverse: np.ndarray

    @classmethod
    def from_args(cls) -> "CoordinateTransformer":
        order = "xzy"
        signs = "--+"
        if len(order) != 3 or sorted(order) != ['x', 'y', 'z']:
            raise ValueError("--axis-order must be a permutation of xyz (e.g. xyz, xzy, zyx)")
        if len(signs) != 3 or any(ch not in '+-' for ch in signs):
            raise ValueError("--axis-sign must be three characters drawn from '+' or '-'")

        permutation = np.zeros((3, 3), dtype=float)
        axis_index = {'x': 0, 'y': 1, 'z': 2}
        for row, axis_char in enumerate(order):
            permutation[row, axis_index[axis_char]] = 1.0
        sign_diag = np.diag([1.0 if ch == '+' else -1.0 for ch in signs])
        matrix = sign_diag @ permutation
        inverse = matrix.T  # orthonormal even with reflections
        return cls(matrix=matrix, inverse=inverse)

    def apply_position(self, vec: np.ndarray) -> np.ndarray:
        return self.matrix @ vec

    def apply_rotation(self, rot: R) -> R:
        mat = rot.as_matrix()
        converted = self.matrix @ mat @ self.inverse
        return R.from_matrix(converted)


class VMCFrameAssembler:
    """Collects OSC messages and produces global bone transforms."""

    def __init__(
        self,
        frame_queue: "queue.Queue[Dict[str, Tuple[np.ndarray, np.ndarray]]]",
        transformer: CoordinateTransformer,
        verbose: bool = False,
        min_output_bones: int = 1,
        frame_trigger: str = "hips",
    ) -> None:
        if frame_trigger not in {"timestamp", "root", "hips"}:
            raise ValueError("frame_trigger must be 'timestamp', 'root', or 'hips'")
        self._transformer = transformer
        self._transformer_config = (frame_trigger, transformer)
        self._queue = frame_queue
        self._verbose = verbose
        self._min_output_bones = min_output_bones
        self._frame_trigger = frame_trigger

        self._lock = threading.Lock()
        self._root_pos: Optional[np.ndarray] = None
        self._root_rot: Optional[R] = None
        self._local_transforms: Dict[str, Tuple[np.ndarray, R]] = {}
        self._missing_once: set[str] = set()
        self._last_timestamp: Optional[float] = None

    def set_transformer(self, transformer: CoordinateTransformer) -> None:
        with self._lock:
            self._transformer = transformer

    def handle_root(self, address: str, *args: Iterable[float]) -> None:
        if len(args) < 13:
            return
        label = args[0]
        if isinstance(label, str) and label.lower() != "root":
            return
        pos = np.asarray(args[1:4], dtype=np.float64)
        quat_xyzw = np.asarray(args[4:8], dtype=np.float64)
        rot = R.from_quat(quat_xyzw)
        frame = None
        with self._lock:
            self._root_pos = pos
            self._root_rot = rot
            self._last_timestamp = time.time()
            if self._frame_trigger == "root":
                frame = self._build_frame_locked()
        if self._verbose:
            pass  # root data received
        if self._frame_trigger == "root":
            self._enqueue_frame(frame)

    def handle_bone(self, address: str, *args: Iterable[float]) -> None:
        if len(args) < 8:
            return
        name = args[0]
        if not isinstance(name, str):
            return
        pos = np.asarray(args[1:4], dtype=np.float64)
        quat_xyzw = np.asarray(args[4:8], dtype=np.float64)
        rot = R.from_quat(quat_xyzw)
        frame = None
        should_trigger = False
        with self._lock:
            self._local_transforms[name] = (pos, rot)
            if name == "Hips":
                if self._frame_trigger == "hips" or self._root_pos is None or self._root_rot is None:
                    self._root_pos = pos
                    self._root_rot = rot
                if self._frame_trigger == "hips":
                    self._last_timestamp = time.time()
                    frame = self._build_frame_locked()
                    should_trigger = True
        if should_trigger:
            self._enqueue_frame(frame)

    def handle_timestamp(self, address: str, *args: Iterable[float]) -> None:
        timestamp = float(args[0]) if args else time.time()
        with self._lock:
            self._last_timestamp = timestamp
            if self._frame_trigger != "timestamp":
                return
            frame = self._build_frame_locked()
        self._enqueue_frame(frame)

    def _enqueue_frame(self, frame: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]]) -> None:
        if frame is None:
            return
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(frame)

    def _build_frame_locked(self) -> Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]]:
        if (self._root_rot is None or self._root_pos is None) and "Hips" not in self._local_transforms:
            if self._verbose:
                print("[VMC] Waiting for Hips transform; cannot assemble frame yet.")
            return None

        cache: Dict[str, Tuple[np.ndarray, R]] = {}
        missing: List[str] = []

        def compute(vmc_name: str) -> Optional[Tuple[np.ndarray, R]]:
            if vmc_name in cache:
                return cache[vmc_name]

            local = self._local_transforms.get(vmc_name)
            if local is None and vmc_name in VMC_BONE_FALLBACKS:
                fallback_name = VMC_BONE_FALLBACKS[vmc_name]
                local = self._local_transforms.get(fallback_name)
                if local is not None and self._verbose and vmc_name not in self._missing_once:
                    print(f"[VMC] Using fallback bone {fallback_name} for missing {vmc_name}.")
            if local is None:
                if vmc_name == "Hips" and self._root_pos is not None and self._root_rot is not None:
                    cache[vmc_name] = (self._root_pos.copy(), self._root_rot)
                    return cache[vmc_name]
                missing.append(vmc_name)
                return None

            parents = VMC_PARENT_CANDIDATES.get(vmc_name, [])
            if not parents:
                if vmc_name == "Hips":
                    local_pos, local_rot = local
                    if self._root_pos is not None and self._root_rot is not None:
                        global_rot = self._root_rot * local_rot
                        global_pos = self._root_pos + self._root_rot.apply(local_pos)
                    else:
                        global_rot = local_rot
                        global_pos = local_pos
                    cache[vmc_name] = (global_pos, global_rot)
                    return cache[vmc_name]
                parent_pos, parent_rot = self._root_pos, self._root_rot
                if parent_pos is None or parent_rot is None:
                    missing.append(vmc_name)
                    return None
            else:
                parent_pos = None
                parent_rot = None
                for candidate in parents:
                    parent = compute(candidate)
                    if parent is not None:
                        parent_pos, parent_rot = parent
                        break
                if parent_pos is None or parent_rot is None:
                    missing.append(vmc_name)
                    return None

            local_pos, local_rot = local
            global_rot = parent_rot * local_rot
            global_pos = parent_pos + parent_rot.apply(local_pos)
            cache[vmc_name] = (global_pos, global_rot)
            return cache[vmc_name]

        output: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for vmc_name, gmr_name in VMC_TO_GMR_NAME_MAP.items():
            result = compute(vmc_name)
            if result is None:
                continue
            pos_u, rot_u = result
            pos_g = self._transformer.apply_position(pos_u)
            rot_g = self._transformer.apply_rotation(rot_u)
            quat_wxyz = rot_g.as_quat()
            quat_wxyz = np.array([quat_wxyz[3], quat_wxyz[0], quat_wxyz[1], quat_wxyz[2]], dtype=np.float64)
            output[gmr_name] = (pos_g.astype(np.float64), quat_wxyz)

        if not output and self._verbose:
            print("[VMC] No recognized bones in current frame.")

        if self._verbose:
            if missing:
                pass

        if len(output) < self._min_output_bones:
            if self._verbose:
                print(
                    f"[VMC] Skipping frame due to insufficient bones ({len(output)} < {self._min_output_bones})."
                )
            return None

        return output




def frame_has_invalid_values(frame: Dict[str, Tuple[np.ndarray, np.ndarray]]) -> Optional[str]:
    """Return the first bone name that contains NaN/Inf, or None if all good."""
    for bone_name, (pos, quat) in frame.items():
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return bone_name
    return None


def vector_has_invalid_values(vec: np.ndarray) -> bool:
    return not np.all(np.isfinite(vec))

class VMCOSCServer(threading.Thread):
    """Background OSC server providing VMC frames."""

    def __init__(self, listen_ip: str, listen_port: int, assembler: VMCFrameAssembler) -> None:
        super().__init__(daemon=True)
        self._listen_ip = listen_ip
        self._listen_port = listen_port
        self._assembler = assembler
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._transport = None

    def run(self) -> None:  # pragma: no cover - network loop
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        dispatcher = Dispatcher()
        dispatcher.map("/VMC/Ext/Root/Pos", self._assembler.handle_root)
        dispatcher.map("/VMC/Ext/Bone/Pos", self._assembler.handle_bone)
        dispatcher.map("/VMC/Ext/T", self._assembler.handle_timestamp)

        server = AsyncIOOSCUDPServer((self._listen_ip, self._listen_port), dispatcher, self._loop)
        self._transport, _ = self._loop.run_until_complete(server.create_serve_endpoint())
        print(f"[VMC] Listening on {self._listen_ip}:{self._listen_port}")
        try:
            self._loop.run_forever()
        finally:
            if self._transport is not None:
                self._transport.close()
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()

    def stop(self) -> None:
        if self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._loop.stop)
        self.join(timeout=1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Retarget VMC OSC streams to humanoid robots with GMR.")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="IP address to bind the OSC receiver (default: 0.0.0.0).")
    parser.add_argument("--listen-port", type=int, default=39539, help="UDP port for incoming VMC data (default: 39539).")
    parser.add_argument("--robot", type=str, default="unitree_g1", help="Target robot name (see params.py for supported options).")
    parser.add_argument("--actual-human-height", type=float, default=None, help="Override human height scaling (meters).")
    parser.add_argument("--frame-trigger", choices=["timestamp", "root", "hips"], default="hips", help="Select OSC message used to flush a frame (timestamp, root, or hips).")
    parser.add_argument("--queue-size", type=int, default=4, help="Frame queue size before dropping oldest frames.")
    parser.add_argument("--min-output-bones", type=int, default=1, help="Minimum bones required to emit a frame.")
    parser.add_argument("--rate-limit", action="store_true", help="Match robot playback rate to incoming motion.")
    parser.add_argument("--offset-to-ground", action="store_true", help="Shift human data so the lowest foot touches ground.")
    parser.add_argument("--verbose", action="store_true", help="Print diagnostics and raw VMC payloads.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    current_order, current_sign = "xzy", "--+"
    transformer = CoordinateTransformer.from_args()
    frame_queue: "queue.Queue[Dict[str, Tuple[np.ndarray, np.ndarray]]]" = queue.Queue(maxsize=args.queue_size)
    assembler = VMCFrameAssembler(
        frame_queue,
        transformer,
        verbose=args.verbose,
        min_output_bones=args.min_output_bones,
        frame_trigger=args.frame_trigger,
    )
    osc_server = VMCOSCServer(args.listen_ip, args.listen_port, assembler)

    osc_server.start()

    retarget = GMR(
        src_human="fbx",
        tgt_robot=args.robot,
        actual_human_height=args.actual_human_height,
    )
    viewer = RobotMotionViewer(robot_type=args.robot)

    def shutdown_handler(signum: int, frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("[VMC] Retargeting loop started. Press Ctrl+C to exit.")
    try:
        while True:
            try:
                frame = frame_queue.get(timeout=1.0)
            except queue.Empty:
                if args.verbose:
                    print("[VMC] Waiting for frame...")
                continue
            bad_bone = frame_has_invalid_values(frame)
            if bad_bone is not None:
                if args.verbose:
                    print(f"[VMC] Skipping frame due to invalid values in {bad_bone}.")
                continue
            qpos = retarget.retarget(frame, offset_to_ground=args.offset_to_ground)
            if vector_has_invalid_values(qpos):
                if args.verbose:
                    print("[VMC] Retarget output contained invalid values; frame skipped.")
                continue
            viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retarget.scaled_human_data,
                rate_limit=args.rate_limit,
            )
    except KeyboardInterrupt:
        print("\n[VMC] Shutting down...")
    finally:
        viewer.close()
        osc_server.stop()


if __name__ == "__main__":
    main()

