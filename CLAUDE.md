# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR teleoperation framework integrating a Meta Quest 3 headset with a Universal Robots UR3e collaborative arm. This branch (`nic`) is one of several personal branches in a shared research team repository (other branches: `john`, `ollie`, `seb`).

The current focus is a **digital twin**: the UR3e in Unity mirrors the real robot's joint positions in real time via ROS 2.

## Repository Layout

```
nic/
  ros2_ws/          — ROS 2 workspace
    src/
      ROS-TCP-Endpoint/              — Unity-ROS TCP bridge (cloned from source)
      Universal_Robots_ROS2_Driver/  — UR driver (cloned from source, fixes apt segfault)
  Unity/My project/ — Unity 6.3 project
    Assets/
      Scripts/JointStateSubscriber.cs  — subscribes to /joint_states, drives robot joints
      URDF/                            — ur3e.urdf + meshes
      RosMessages/                     — generated C# ROS message classes
  ROS-TCP-Connector/  — Unity package (local, referenced via file:// in manifest.json)
  URDF-Importer/      — Unity package (local, referenced via file:// in manifest.json)
  SETUP.md            — full onboarding guide for new contributors
```

## Development Environment

**Prerequisites:** ROS 2 Humble on Ubuntu 22.04, Unity 6.3 LTS.

**Do not use Docker** — the apt-installed UR driver has a segfault bug on Ubuntu 22.04; build from source instead (already done in this workspace).

### Build the ROS workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Sequence (real robot at 192.168.0.194)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true

# Terminal 2
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Then on the teach pendant: load and run External Control program.
Then hit Play in Unity.

## Architecture

```
Meta Quest 3 (XR Interface)
  └─ controller poses, button input → Cartesian velocity commands
       ↓
Perception Layer (RGB-D camera)
  └─ point clouds → obstacle/workspace detection → collision map updates
       ↓
Planning Layer (MoveIt 2)
  └─ trajectory generation, collision checking, dynamic no-go zone enforcement
       ↓
Control Layer (ur_robot_driver / ros2_control)
  └─ Cartesian velocity control or joint trajectory execution → UR3e hardware
```

**Teleoperation modes (planned):**
- **Freeform**: Direct end-effector velocity control from XR controllers
- **Assisted**: Operator intent + system-enforced collision avoidance, velocity limits, no-go zones
- **Training**: Record XR demonstrations as replayable trajectories

**Planned ROS package layout** (to be created in `ros2_ws/src/`):

| Package | Purpose |
|---|---|
| `xr_interface` | Meta Quest input, spatial tracking, XR overlays |
| `perception` | RGB-D processing, point cloud pipeline |
| `planning` | Trajectory generation, workspace constraints |
| `control` | Low-level robot control, velocity scaling |
| `robot_bringup` | Hardware launch files and configuration |
| `experiments` | Research evaluation and data collection |

## Unity Project

- **Path:** `Unity/My project/`
- **Version:** Unity 6.3 LTS (6000.3.9f1)
- **Build target:** Android (Meta Quest 3)
- **Scene:** `Assets/Scenes/SampleScene.unity`
- **ROS IP:** 127.0.0.1, Port: 10000 (configured in Robotics → ROS Settings)

### Key scripts

- `Assets/Scripts/JointStateSubscriber.cs` — attach to root `ur` GameObject; subscribes to `/joint_states` and drives ArticulationBody joints to mirror the real robot

## Known Issues

- **UR driver segfault (exit code -11)** — apt-installed `ros-humble-ur-robot-driver` crashes on Ubuntu 22.04. Fixed by building `Universal_Robots_ROS2_Driver` from source (already in `ros2_ws/src/`).
- **Robot falls in Unity on Play** — disable gravity (uncheck Use Gravity) on the root `ur` ArticulationBody.
- **Git HTTPS clone fails on this machine** — always use SSH (`git@github.com:...`).
- **Unity Package Manager can't find robotics packages** — clone locally, reference via `file://` in `Packages/manifest.json`.
