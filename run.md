# HoloAssist — Run Guide

## Calibrate (only if camera or robot moved)

```bash
cd ~/git/RS2-HoloAssist/nic
./calibrate.sh --robot-ip 192.168.0.192
```

- Stick AprilTag (36h11, ID 0, 32mm) on gripper
- On teach pendant: run External Control (host IP: 192.168.0.100)
- Move robot to 15-25 varied poses, click Take Sample at each
- Click Save, then Ctrl+C

No need to delete old calibration — Save overwrites it.

## Launch (teleoperation + perception)

```bash
cd ~/git/RS2-HoloAssist/nic
./launch.sh --robot-ip 192.168.0.192 --perception
```

## Dashboard (separate terminal)

```bash
cd ~/git/RS2-HoloAssist/nic
./dashboard.sh --fullscreen
```

## Then

1. Teach pendant: run **External Control** (host IP: `192.168.0.100`)
2. Unity: hit **Play** (ROS Settings: IP = `192.168.0.102`, port = `10000`)

## Quick debug

```bash
ros2 topic echo /detections --once              # tags detected?
ros2 topic echo /holoassist/unity/cube_1_pose --once  # poses reaching Unity?
ros2 run tf2_ros tf2_echo base_link camera_link  # calibration TF alive?
ros2 topic hz /camera/camera/color/image_raw     # camera streaming?
```
