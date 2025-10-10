
# GMR: General Motion Retargeting (Fork for VMC Support)

This repository is forked from [GMR](https://github.com/YanjieZe/GMR)

Please refer to the original project's README first.

This repository has been modified to add real-time VMC retargeting.

**VMC Protocol specification:** https://protocol.vmc.info/english

## Real-Time Retargeting from VMC to Robot

1. Use your preferred VMC mocap software and enable its data streaming option. Here, we use Dollars MONO as an example.

**VMC Performer:** https://protocol.vmc.info/Reference

**Dollars MONO:** https://www.dollarsmocap.com/mono

![2025-10-07 21-26-14-349](https://github.com/user-attachments/assets/bd09d409-afae-4e6e-8471-415adc804da0)

2. run:
```bash
python scripts\vmc_to_robot.py --actual-human-height=1.65
```

Press `s` while the script is running to toggle motion recording.

When you stop, a timestamped `.pkl` file is saved in the current directory.

3. Use a camera or a video file as the input in Dollars MONO.

<img width="513" height="856" alt="2025_10_10_17_24_51-GMR-master - Visual Studio Code" src="https://github.com/user-attachments/assets/06c633bc-627b-4a17-98de-b115659e10b6" />

(Source footage courtesy of https://www.youtube.com/watch?v=cVjIqr8CTtQ)
## Visualize saved robot motion
```bash
python scripts/vis_robot_motion.py --robot unitree_g1 --robot_motion_path <path_to_save_robot_data.pkl>
```
