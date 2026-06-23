# HoloAssist Dependencies

## System Baseline

| Item | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-111-generic |
| Python | 3.10.12 |
| ROS 2 | Humble Hawksbill |

---

## APT Packages

```bash
sudo apt install -y \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-rviz2 \
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

> `ros-humble-ur` installs the UR robot driver, controllers, calibration, and MoveIt configs as prebuilt binaries — no source build needed.

---

## Python Dependencies (outside ros2_ws)

### `scripts/launch.py` / `scripts/calibrate.py`

| Package | Install |
|---|---|
| `python3-netifaces` | `sudo apt install python3-netifaces` |
| `python3-yaml` | `sudo apt install python3-yaml` |

### `dashboard/`

| Package | Install |
|---|---|
| `python3-pyqt5` | `sudo apt install python3-pyqt5` |
| `websockets` | `pip install websockets` |
| `python3-opencv` | `sudo apt install python3-opencv` |
| `rclpy` | via ROS Humble |

### `scripts/beacon.py`

Standard library only.

---

## ros2_ws Source Packages

### Vendored Submodules

| Directory | Remote |
|---|---|
| `easy_handeye2` | `github.com/marcoesposito1988/easy_handeye2` |
| `ROS-TCP-Endpoint` | `github.com/Unity-Technologies/ROS-TCP-Endpoint` |
| `UR_OnRobot_ROS2` | `github.com/tonydle/UR_OnRobot_ROS2` |
| `onrobot_description` | `github.com/tonydle/OnRobot_ROS2_Description` |
| `onrobot_driver` | `github.com/tonydle/OnRobot_ROS2_Driver` |

### HoloAssist Packages

| Package | Purpose |
|---|---|
| `holoassist_perception` | Webcam publisher, AprilTag cube tracking, trolley scene, pick/place service bridge, custom msgs |
| `holoassist_movement` | MoveIt execution, workspace scene, pick/place sequencer, typed target/status messages |

### Third-Party (built from source)

| Package | Purpose |
|---|---|
| `ros_tcp_endpoint` | ROS ↔ Unity TCP bridge (port 10000) |
| `easy_handeye2` / `easy_handeye2_msgs` | Eye-to-hand calibration GUI |
| `onrobot_description` / `onrobot_driver` | RG2 URDF and hardware interface |
| `ur_onrobot_description` / `ur_onrobot_control` / `ur_onrobot_moveit_config` | Combined UR + RG2 description, launch, and MoveIt config |

---

## Dependency Flow

```
webcam (/dev/video0)
        │
        ▼
holoassist_perception
(aprilcube_tracker + trolley scene + pick/place bridge)
        │
        ▼
holoassist_movement
(pick_place_sequencer + workspace_scene_manager)
        │
        ▼
ur_onrobot_control
(UR driver [apt] + OnRobot controllers)
        │
        ▼
    UR3e + RG2
```

---

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
