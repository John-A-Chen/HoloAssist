# HoloAssist Dependencies

## System Baseline

| Item | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-111-generic |
| Python | 3.10.12 |
| ROS 2 | Humble Hawksbill |

---

## Root-Level Python Dependencies

These are required to run `launch.py`, `calibrate.py`, and `dashboard/`.

### `launch.py` / `calibrate.py`

| Package | Install | Notes |
|---|---|---|
| `python3-netifaces` | `sudo apt install python3-netifaces` | WiFi IP auto-detect for Unity hint |
| `python3-yaml` | `sudo apt install python3-yaml` | Reads `holoassist_calibration.calib` |
| `pyrealsense2` | bundled with `ros-humble-realsense2-camera` | Camera auto-detection |

### `dashboard/`

| Package | Install | Notes |
|---|---|---|
| `python3-pyqt5` | `sudo apt install python3-pyqt5` | Dashboard UI |
| `websockets` | `pip install websockets` | Bridge server for Steam Deck remote |
| `rclpy` | via ROS Humble | ROS interface (optional `--no-ros` mode exists) |
| `python3-opencv` | `sudo apt install python3-opencv` | Headset image stream decoding |

### `beacon.py`

Standard library only — no additional deps.

---

## APT Packages (ROS + System)

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
  python3-yaml \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## `ros2_ws/src` Package Catalog

### Vendored Upstream Repos

| Directory | Remote | Pinned |
|---|---|---|
| `easy_handeye2` | `github.com/marcoesposito1988/easy_handeye2` | `b42cae6` |
| `ROS-TCP-Endpoint` | `github.com/Unity-Technologies/ROS-TCP-Endpoint` | `ROS2v0.7.0` |
| `Universal_Robots_ROS2_Driver` | `github.com/UniversalRobots/Universal_Robots_ROS2_Driver` | `2.12.0` |

### All Packages

| Package | Version | Purpose |
|---|---|---|
| `ros_tcp_endpoint` | 0.7.0 | ROS ↔ Unity TCP bridge (port 10000) |
| `ur_robot_driver` | 2.12.0 | UR3e hardware driver |
| `ur_controllers` | 2.12.0 | Speed-scaled trajectory + velocity controllers |
| `ur_bringup` | 2.12.0 | UR launch/config helpers |
| `ur_calibration` | 2.12.0 | Factory calibration extraction |
| `ur_dashboard_msgs` | 2.12.0 | Dashboard/status message types |
| `ur_moveit_config` | 2.12.0 | Upstream UR MoveIt configs (reference) |
| `easy_handeye2` | 0.5.0 | Eye-to-hand calibration GUI |
| `easy_handeye2_msgs` | 0.5.0 | Calibration service/message types |
| `onrobot_description` | 0.0.1 | RG2 URDF/xacro |
| `onrobot_driver` | 0.0.1 | OnRobot RG2 hardware interface |
| `ur_onrobot_description` | 0.0.1 | Combined UR + OnRobot robot description |
| `ur_onrobot_control` | 0.0.1 | Launches UR + OnRobot controllers |
| `ur_onrobot_moveit_config` | 0.0.1 | MoveIt SRDF/config for UR + RG2 |
| `holo_assist_depth_tracker` | 0.1.0 | Real-camera perception, cube pose fusion |
| `holo_assist_depth_tracker_sim` | 0.1.0 | Sim perception, pick/place service bridge |
| `holo_assist_depth_tracker_sim_interfaces` | 0.1.0 | `PickCubeToBin` service + custom msgs |
| `moveit_robot_control` | 0.0.1 | MoveIt execution, workspace scene, pick/place sequencer |
| `moveit_robot_control_msgs` | 0.0.1 | Typed target/status messages |
| `holoassist_unity_bridge` | 0.0.0 | Unity integration hooks (WIP) |

---

## Custom Package Dependency Flow

```
RealSense / apriltag_ros
        │
        ▼
holo_assist_depth_tracker          holo_assist_depth_tracker_sim
(cube_pose_node → /holoassist/     (sim truth / pick-place service
 perception/april_cube_N_pose)      bridge → PickCubeToBin service)
        │                                       │
        └──────────────┬────────────────────────┘
                       ▼
              moveit_robot_control
           (coordinate_listener +
            pick_place_sequencer +
            workspace_scene_manager)
                       │
              ur_onrobot_control
           (UR driver + controllers)
                       │
                   UR3e robot
```

---

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
