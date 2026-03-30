# HoloAssist — Setup Guide

This guide gets you up to speed with the current state of the `nic` branch. The system provides a **digital twin** (UR3e mirrored in Unity via ROS 2) and **XR teleoperation** (Quest 3 controllers drive the real robot).

## Current State

- Unity project set up with Meta XR SDK, ROS-TCP-Connector, and URDF-Importer
- UR3e URDF imported into Unity with visual meshes
- **Digital twin**: `JointStateSubscriber.cs` subscribes to `/joint_states` and drives robot joints (purely kinematic, no physics)
- **Robot control**: `RobotController.cs` with two modes — Direct Joint (jog individual joints) and MoveIt Servo (Cartesian end-effector velocity)
- **Robot placement**: `RobotBasePlacer.cs` lets you pinch-to-grab and reposition the robot in mixed reality
- **HUD**: `RobotHUD.cs` shows current control mode and selected joint in a floating panel
- Input via Unity Input System (`RobotControlActions.inputactions`) — no OVRInput dependency
- `ros_tcp_endpoint` built in the ROS workspace
- UR ROS2 driver cloned from source (fixes segfault in apt version)

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (native install)
- Unity 6.3 LTS (6000.3.9f1)
- Android Build Support module installed in Unity (for Quest deployment)
- `adb` installed, Meta Quest 3 in developer mode

## ROS Workspace Setup

```bash
cd ros2_ws/src

# ROS-Unity bridge
git clone -b main-ros2 git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git

# UR driver from source (apt version has a segfault bug on Ubuntu 22.04)
git clone -b humble git@github.com:UniversalRobots/Universal_Robots_ROS2_Driver.git

# Install dependencies (run from ros2_ws/)
cd ..
sudo apt-get install -y ros-humble-hardware-interface-testing
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

## Unity Project Setup

The Unity project is at `Unity/My project/`. Packages are referenced as local file paths in `Packages/manifest.json` — no registry needed.

### Local packages (already cloned)

| Package | Path |
|---|---|
| ROS-TCP-Connector | `ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector` |
| URDF-Importer | `URDF-Importer/com.unity.robotics.urdf-importer` |

Clone these into the `nic/` directory (same level as `Unity/`):

```bash
# Run from nic/ directory
git clone git@github.com:Unity-Technologies/ROS-TCP-Connector.git
git clone git@github.com:Unity-Technologies/URDF-Importer.git
```

`Packages/manifest.json` already uses relative paths so no editing needed — Unity will find them automatically.

### URDF and meshes

The UR3e URDF and meshes are already in `Unity/My project/Assets/URDF/`:

```
Assets/URDF/
  ur3e.urdf
  ur_description/
    meshes/ur3e/
      visual/    ← .dae files (copied from /opt/ros/humble/share/ur_description/)
      collision/ ← .stl files
```

If setting up fresh, copy meshes from your ROS install:

```bash
mkdir -p "Unity/My project/Assets/URDF/ur_description/meshes/ur3e/visual"
mkdir -p "Unity/My project/Assets/URDF/ur_description/meshes/ur3e/collision"
cp /opt/ros/humble/share/ur_description/meshes/ur3e/visual/*.dae "Unity/My project/Assets/URDF/ur_description/meshes/ur3e/visual/"
cp /opt/ros/humble/share/ur_description/meshes/ur3e/collision/*.stl "Unity/My project/Assets/URDF/ur_description/meshes/ur3e/collision/"
```

### ROS Settings in Unity

- **Robotics → ROS Settings**
- ROS IP Address: `127.0.0.1` (same machine) or your Linux machine's IP
- ROS Port: `10000`

### Scripts

All scripts are in `Assets/Scripts/`. Attach them as follows:

| Script | Attach to | Inspector setup |
|---|---|---|
| `JointStateSubscriber.cs` | Root `ur` GameObject | None — auto-discovers joints |
| `RobotController.cs` | Any GameObject | Drag `RobotControlActions.inputactions` into **Input Actions** field |
| `RobotBasePlacer.cs` | Empty parent wrapper (e.g. `RobotRig`) containing `ur` as child | None |
| `RobotHUD.cs` | Empty GameObject | Drag the RobotController GameObject into **Controller** field |

**Note:** JointStateSubscriber disables all ArticulationBody components on Start (purely kinematic). No need to manually disable gravity.

## Launch Sequence (real robot, IP: 192.168.0.194)

```bash
# Terminal 1 — UR driver (source-built version)
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true

# Terminal 2 — ROS-Unity bridge
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 3 — MoveIt Servo (only needed for Servo mode, not Direct Joint)
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false launch_servo:=true
```

Then on the **teach pendant**: load and run the External Control program (Host IP: `192.168.0.100`).

Then hit **Play** in Unity.

## Controller Mapping (Quest 3)

| Input | Direct Joint Mode | MoveIt Servo Mode |
|---|---|---|
| **Menu button** (left) | Toggle to Servo | Toggle to Direct Joint |
| **A button** (right) | Next joint | — |
| **B button** (right) | Previous joint | — |
| **Right stick Y** | Jog selected joint (proportional velocity) | End-effector up/down |
| **Right stick X** | — | End-effector yaw |
| **Left stick Y** | — | End-effector forward/back |
| **Left stick X** | — | End-effector left/right |

- **Direct Joint mode** works with Terminals 1+2 only
- **Servo mode** requires all 3 terminals (MoveIt Servo must be running)

## Known Issues

- **UR driver segfault (exit code -11)** — affects apt-installed `ros-humble-ur-robot-driver` on Ubuntu 22.04. Fix: build from source (see above).
- **Git HTTPS clone fails** — use SSH (`git@github.com:...`) for all cloning.
- **Unity Package Manager can't find robotics packages** — clone locally and reference via `file://` in manifest.json.
- **Meta XR SDK cached packages cause issues** — if you hit errors related to Meta XR packages, clear the cached copies: `rm -rf "Unity/My project/Library/PackageCache/com.meta.xr.*"`
- **ros_tcp_endpoint crashes on Unity disconnect/reconnect** — restart the endpoint after every Unity Play/Stop cycle.
- **Do NOT use OVRInput** — the MR template uses Unity Input System, not OVR. `OVRInput.Get()` returns zero without `OVRManager`. All controller input goes through `RobotControlActions.inputactions`.
- **XRI input conflicts** — XRI Default Input Actions bind thumbsticks to teleport/turn/move. `RobotController.cs` disables these automatically at runtime. If you add new XRI features, check for binding conflicts.
