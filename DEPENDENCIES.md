# HoloAssist Dependency Reference

This document is the dependency source of truth for the current RS2 HoloAssist stack.

It covers:

- exact environment/version snapshot used in this repo
- every ROS package in `ros2_ws/src`
- where each package came from
- what each package is used for
- how to install/build exactly like this setup
- how custom packages tie together end to end

## Version Snapshot (Current Machine)

Snapshot date: 2026-05-26

Host baseline:

- OS: Ubuntu 22.04.5 LTS
- Kernel: 6.8.0-111-generic
- Python: 3.10.12
- ROS distro: Humble

Key system/Python packages installed:

| Package | Version |
|---|---|
| python3-netifaces | 0.11.0-1build2 |
| python3-numpy | 1:1.21.5-1ubuntu22.04.1 |
| python3-opencv | 4.5.4+dfsg-9ubuntu4 |
| python3-pyqt5 | 5.15.6+dfsg-1ubuntu3 |
| python3-yaml | 5.4.1-1ubuntu1 |
| ros-humble-apriltag | 3.4.5-1jammy.20260226.005100 |
| ros-humble-apriltag-ros | 3.3.0-1jammy.20260326.001446 |
| ros-humble-controller-manager | 2.53.1-1jammy.20260325.233100 |
| ros-humble-hardware-interface-testing | 2.54.0-1jammy.20260422.085245 |
| ros-humble-moveit | 2.5.9-1jammy.20260326.040749 |
| ros-humble-realsense2-camera | 4.57.7-4jammy.20260328.173416 |
| ros-humble-rviz2 | 11.2.26-1jammy.20260326.021558 |
| ros-humble-tf2-ros | 0.25.20-2jammy.20260325.223305 |

Runtime Python module versions observed:

| Module | Version |
|---|---|
| numpy | 1.26.4 |
| cv2 | 4.13.0 |
| PyQt5 | 5.15.6 (Qt 5.15.3) |
| yaml | 5.4.1 |
| netifaces | importable (no `__version__`) |
| rclpy | importable (no `__version__`) |

## Install Like This Setup

This repo vendors nearly all source packages directly under `ros2_ws/src`.

### 1) Base tools

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip
```

### 2) ROS + runtime packages

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-rviz2 \
  ros-humble-realsense2-camera \
  ros-humble-apriltag \
  ros-humble-apriltag-ros \
  ros-humble-hardware-interface-testing \
  python3-opencv \
  python3-numpy \
  python3-pyqt5 \
  python3-netifaces \
  python3-yaml
```

### 3) Resolve ROS package deps from workspace (recommended)

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4) Build workspace

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## External Upstream Repos Vendored in `ros2_ws/src`

These are upstream projects pulled into this workspace as source trees.

| Repo Dir | Upstream Remote | Pinned Commit (local) | Tag/Describe |
|---|---|---|---|
| `easy_handeye2` | `git@github.com:marcoesposito1988/easy_handeye2.git` | `b42cae604b5c01dbd650fcdac40dbf334cb098f4` | `b42cae6` |
| `ROS-TCP-Endpoint` | `git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git` | `54c1a64b6d5ef6ffa0a0431570bb74329b79b15b` | `ROS2v0.7.0` |
| `Universal_Robots_ROS2_Driver` | `git@github.com:UniversalRobots/Universal_Robots_ROS2_Driver.git` | `7658387ca25c6302ee651bf2ebb059fcdbadf326` | `2.12.0-14-g7658387` |

Note: these are vendored source trees in this repo, not git submodules.

## Workspace ROS Package Catalog

Package versions come from each package's `package.xml`.

| Package | Version | Source Path | Origin | What We Use It For |
|---|---:|---|---|---|
| `ros_tcp_endpoint` | 0.7.0 | `src/ROS-TCP-Endpoint` | Unity Technologies | ROS<->Unity TCP bridge (`default_server_endpoint`) |
| `ur` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur` | Universal Robots | metapackage dependency roll-up |
| `ur_bringup` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_bringup` | Universal Robots | UR launch/config helpers |
| `ur_calibration` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_calibration` | Universal Robots | UR factory calibration extraction tooling |
| `ur_controllers` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_controllers` | Universal Robots | speed-scaled UR controllers |
| `ur_dashboard_msgs` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_dashboard_msgs` | Universal Robots | dashboard/status messages used by control stack |
| `ur_moveit_config` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_moveit_config` | Universal Robots | base UR MoveIt configs (upstream reference) |
| `ur_robot_driver` | 2.12.0 | `src/Universal_Robots_ROS2_Driver/ur_robot_driver` | Universal Robots | actual UR hardware driver |
| `easy_handeye2` | 0.5.0 | `src/easy_handeye2/easy_handeye2` | Marco Esposito | eye-to-hand calibration GUI/workflow |
| `easy_handeye2_msgs` | 0.5.0 | `src/easy_handeye2/easy_handeye2_msgs` | Marco Esposito | message/service defs for calibration |
| `onrobot_description` | 0.0.1 | `src/onrobot_description` | vendored third-party (maintainer: Tony Le) | RG2/RG6 URDF/xacro descriptions |
| `onrobot_driver` | 0.0.1 | `src/onrobot_driver` | vendored third-party (maintainer: Tony Le) | OnRobot hardware interface/control |
| `ur_onrobot_description` | 0.0.1 | `src/ur_onrobot/ur_onrobot_description` | vendored integration layer | combined UR+OnRobot robot description |
| `ur_onrobot_control` | 0.0.1 | `src/ur_onrobot/ur_onrobot_control` | vendored integration layer | launches UR + OnRobot controllers |
| `ur_onrobot_moveit_config` | 0.0.1 | `src/ur_onrobot/ur_onrobot_moveit_config` | vendored integration layer | MoveIt config for UR + OnRobot |
| `holo_assist_depth_tracker` | 0.1.0 | `src/holo_assist_depth_tracker` | HoloAssist team (custom) | real-camera perception, cube pose fusion, debug image |
| `holo_assist_depth_tracker_sim` | 0.1.0 | `src/holo_assist_depth_tracker_sim` | HoloAssist team (custom) | fake perception, sim truth, MoveIt bridge nodes |
| `holo_assist_depth_tracker_sim_interfaces` | 0.1.0 | `src/holo_assist_depth_tracker_sim_interfaces` | HoloAssist team (custom) | custom service/msg for pick/place and sim interfaces |
| `moveit_robot_control` | 0.0.1 | `src/moveit_robot_control` | HoloAssist team (custom) | MoveIt execution, workspace scene, pick/place sequencer |
| `moveit_robot_control_msgs` | 0.0.1 | `src/moveit_robot_control_msgs` | HoloAssist team (custom) | custom MoveIt command/status messages |
| `holoassist_unity_bridge` | 0.0.0 | `src/holoassist_unity_bridge` | HoloAssist team (custom, WIP) | Unity-side ROS bridge helpers/topics |

## Custom Package Dependency Map (How It Ties Together)

Primary runtime orchestrator:

- top-level `launch.py` and wrappers `launch.sh`, `calibrate.sh`, `dashboard.sh`

Custom package relationships:

1. `holo_assist_depth_tracker`
- consumes camera streams + AprilTag detections
- publishes `/holoassist/perception/april_cube_{1..4}_pose`
- publishes debug image `/holo_assist_depth_tracker/debug_image`

2. `holo_assist_depth_tracker_sim`
- provides simulated cube truth/perception when no physical camera is used
- bridges selected cube targets into MoveIt command topics
- provides `/holoassist/pick_cube_to_bin` service bridge node

3. `holo_assist_depth_tracker_sim_interfaces`
- defines service/message types used by sim and pick/place service bridge

4. `moveit_robot_control`
- consumes target topics (from dashboard or sim adapter)
- plans/executes trajectories through UR controllers
- runs pick/place sequencer state machine
- publishes workspace/trolley planning scene

5. `moveit_robot_control_msgs`
- defines typed target/status message contracts for MoveIt control

6. `holoassist_unity_bridge`
- additional Unity integration hooks (still minimally defined)

Control and hardware stack dependencies under these custom packages:

- UR robot actuation path uses `ur_robot_driver`, `ur_controllers`, `ur_onrobot_control`
- gripper model/driver path uses `onrobot_description`, `onrobot_driver`
- motion planning path uses `ur_onrobot_moveit_config` + MoveIt 2
- calibration path uses `easy_handeye2`
- Unity networking path uses `ros_tcp_endpoint`

## Dependency Groups by Function

| Function | Main Packages |
|---|---|
| Hardware control (real UR3e) | `ur_robot_driver`, `ur_controllers`, `ur_onrobot_control` |
| Gripper stack | `onrobot_driver`, `onrobot_description`, `ur_onrobot_description` |
| Motion planning | `moveit_robot_control`, `ur_onrobot_moveit_config`, `moveit_robot_control_msgs` |
| Real perception | `holo_assist_depth_tracker`, `apriltag_ros`, `realsense2_camera` |
| Sim perception/fallback | `holo_assist_depth_tracker_sim`, `holo_assist_depth_tracker_sim_interfaces` |
| Calibration | `easy_handeye2`, `easy_handeye2_msgs` |
| Unity comms | `ros_tcp_endpoint`, `holoassist_unity_bridge` |
| Operator UI | top-level `dashboard/` app (PyQt5 + ROS topics/services) |

## Reproducibility Commands

To re-capture package list and versions on any machine:

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
colcon list
```

```bash
rg --files src -g 'package.xml'
```

```bash
dpkg-query -W -f='${Package}\t${Version}\n' | rg '^(ros-humble-|python3-)'
```

```bash
python3 - <<'PY'
import glob, xml.etree.ElementTree as ET
for p in sorted(glob.glob('src/**/package.xml', recursive=True)):
    r = ET.parse(p).getroot()
    print(r.findtext('name'), r.findtext('version'))
PY
```

## Notes

- Some vendored upstream repos in this working tree have local file deletions/edits unrelated to runtime code (mainly removed upstream docs). That does not change package versions listed above.
- For full dependency closure, `rosdep install --from-paths src --ignore-src -r -y` is the canonical installer step and should be run after cloning.
