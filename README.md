
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

3. Use a camera or a video file as the input in Dollars MONO.

<img width="1272" height="1533" alt="2025_10_07_21_31_43-MuJoCo _ g1_mocap" src="https://github.com/user-attachments/assets/7aa02afc-178e-4786-8fc4-fb6bc589a813" />
