# RS2 HoloAssist (Current End-to-End Stack)

This repository contains the current HoloAssist ROS 2 stack for:

- UR3e + OnRobot RG2 control
- Unity bridge (ROS-TCP + UDP beacon)
- Perception (RealSense, webcam fallback, or sim fallback)
- MoveIt planning and pick/place
- E-stop/dashboard UI

This README documents what we are using now across both real and fake hardware.

## What Changed (Current Working Flow)

The stack has been consolidated around a single orchestrator:

- `launch.py` (wrapper: `./launch.sh`) is now the primary entrypoint.
- Hardware mode is selected by whether `--robot-ip` is provided (`--robot-ip` = real, omitted = fake).
- Perception is optional (`--perception`) and auto-selects camera source (RealSense, then Brio/webcam, then sim fallback in fake mode).
- MoveIt is optional (`--moveit`) and integrated to consume live cube poses.
- Dashboard can run in parallel (`--dashboard` / `--dashboard-fullscreen`).
- Trolley scene marker publisher is launched by default.
- Process lifecycle is unified (startup ordering, health checks, graceful shutdown, log files in `/tmp/holoassist_*.log`).

## Repo Layout (Active Paths)

- `launch.py`, `launch.sh`: top-level unified launcher
- `calibrate.py`, `calibrate.sh`: eye-to-hand calibration launcher
- `dashboard/`: PyQt dashboard and ROS interface
- `ros2_ws/`: ROS 2 workspace
- `ros2_ws/src/holo_assist_depth_tracker/`: real-camera perception pipeline
- `ros2_ws/src/holo_assist_depth_tracker_sim/`: fake perception and sim bridge nodes
- `ros2_ws/src/moveit_robot_control/`: MoveIt execution and pick/place sequencer
- `ros2_ws/src/ur_onrobot/`: UR + OnRobot robot/config packages

## Prerequisites

Target distro: ROS 2 Humble (Ubuntu 22.04).

Install core dependencies (example):

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-apriltag \
  ros-humble-apriltag-ros \
  ros-humble-realsense2-camera \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-hardware-interface-testing
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

Install workspace dependencies:

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Single Entrypoint (Recommended)

From repo root:

```bash
cd /home/john/git/RS2-HoloAssist/main
./launch.sh [options]
```

Current options:

```bash
--robot-ip <ip>              # real robot mode (omit for fake hardware)
--ros-ip <ip>                # ROS-TCP bind IP (default 0.0.0.0)
--perception                 # start camera/tag/cube perception pipeline
--moveit                     # start MoveIt + pick/place stack
--dashboard                  # open dashboard window
--dashboard-fullscreen       # open dashboard fullscreen
--no-rviz                    # disable RViz
--verbose                    # stream all subprocess logs to terminal
```

## Run Modes We Use Now

### 1) Real robot, full stack (teleop + perception + moveit)

```bash
./launch.sh \
  --robot-ip 192.168.0.194 \
  --perception \
  --moveit \
  --dashboard
```

Notes:

- Keep UR teach pendant in External Control program.
- Driver launches first, controller switch is attempted automatically (real mode only).
- MoveIt consumes cube poses from `/holoassist/perception/april_cube_{1..4}_pose`.

### 2) Fake hardware, full stack (same launcher)

```bash
./launch.sh --perception --moveit --dashboard
```

Notes:

- No `--robot-ip` means fake mode.
- If no physical camera is detected, launcher auto-falls back to simulated cube perception.
- Fake mode skips the real-driver controller switch call.
- MoveIt safety/status checks are relaxed automatically for fake mode.

### 3) Communication-only / teleop base

```bash
./launch.sh --dashboard
```

Starts robot driver (real or fake), ROS-TCP endpoint, IP beacon, dashboard, and trolley scene marker. Add `--perception`/`--moveit` as needed.

## Perception Pipeline (Current)

Enabled via `--perception`.

Camera selection priority:

1. Intel RealSense
2. Logitech Brio
3. Generic UVC webcam
4. Sim fallback (fake mode only)

RealSense path:

- launches `camera_only.launch.py`
- launches AprilTag + cube fusion + tracker overlay via `visualize_depth_tracker.launch.py`

Webcam path:

- launches `holo_assist_webcam_image_publisher`
- publishes to `/camera/camera/color/image_raw` + `/camera/camera/color/camera_info`
- launches AprilTag + cube nodes without relaunching camera

Outputs used by downstream stack:

- `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
- `/holo_assist_depth_tracker/debug_image`

## MoveIt and Pick/Place (Current)

Enabled via `--moveit`.

Launcher starts:

- `moveit_robot_control/full_holoassist_hardware.launch.py`
- coordinate listener
- pick/place sequencer
- `/holoassist/pick_cube_to_bin` service bridge
- selected-cube to MoveIt target adapter

Service used by dashboard/workflows:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

## Calibration Workflow (Real Robot)

Run:

```bash
./calibrate.sh --robot-ip 192.168.0.194
```

This starts:

- UR + OnRobot driver
- RealSense camera
- AprilTag detector
- easy_handeye2 calibration GUI

Saved transform is expected at:

- `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`

During normal `--perception` bringup, launcher reads this file and publishes static TF from `base_link` to camera tracking frame.

## Dashboard

Run standalone:

```bash
./dashboard.sh
```

or use integrated option:

```bash
./launch.sh --dashboard
```

Dashboard tracks:

- robot connection/controller state
- joint rate/status
- gripper state
- perception topic health
- headset/camera feeds
- pick/place state
- e-stop and resume logic

## Unity Bridge

Launcher always starts:

- ROS-TCP endpoint on TCP `10000`
- UDP IP beacon (`beacon.py`) for headset/client auto-discovery

Set Unity ROS connection to the Wi-Fi IP printed by launcher.

## Operational Checks

After startup, verify:

```bash
ros2 topic list | rg holoassist
ros2 topic hz /joint_states
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 service list | rg pick_cube_to_bin
```

## Known Failure Modes and Fixes

### Duplicate package names during rosdep/colcon

If you see duplicate package errors (for example `ur_onrobot_control` appearing twice), ensure only one copy of each package exists in `ros2_ws/src`.

Quick check:

```bash
python3 - <<'PY'
import glob, xml.etree.ElementTree as ET
pkgs = {}
for p in glob.glob('ros2_ws/src/**/package.xml', recursive=True):
    name = ET.parse(p).getroot().findtext('name')
    pkgs.setdefault(name, []).append(p)
for n, paths in sorted(pkgs.items()):
    if len(paths) > 1:
        print(n)
        [print(' ', x) for x in paths]
PY
```

### Camera not detected

- Confirm device appears in `/dev/video*` (webcam) or `realsense-viewer` (RealSense).
- In fake mode with no camera, sim fallback is expected behavior.

### RViz MotionPlanning plugin missing semantic model

Current launcher suppresses driver RViz when MoveIt is enabled to avoid plugin/semantic race. Use `--moveit` path instead of manually opening competing RViz sessions.

### Real robot not moving

- Verify teach pendant is running External Control.
- Verify robot IP is reachable.
- Check controller state:

```bash
ros2 control list_controllers
```

## Logs and Shutdown

- Non-verbose mode logs to `/tmp/holoassist_*.log`.
- Ctrl+C in launcher terminal performs coordinated shutdown of all child processes.

## Package-Level Docs

- Motion stack: `ros2_ws/src/moveit_robot_control/README.md`
- Perception stack: `ros2_ws/src/holo_assist_depth_tracker/README.md`
- Dependency reference: `DEPENDENCIES.md`
