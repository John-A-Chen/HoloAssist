# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR teleoperation framework integrating a Meta Quest 3 headset with a Universal Robots UR3e collaborative arm. This branch (`nic`) is one of several personal branches in a shared research team repository (other branches: `john`, `ollie`, `seb`).

The current focus is **XR teleoperation**: Quest 3 controllers drive the UR3e end-effector via Cartesian velocity commands through MoveIt 2 Servo.

### Progress Status
- ✅ Unity XR template (Mixed Reality / passthrough) working on Quest 3
- ✅ Meta XR All-in-One SDK v85.0.0 installed and configured
- ✅ ROS-TCP-Connector installed (local package, file:// in manifest.json)
- ✅ URDF-Importer installed; ur3e.urdf imported into scene
- ✅ ROS workspace built; ros_tcp_endpoint and UR driver ready
- ✅ Robot connects (UR driver + External Control program on teach pendant)
- ✅ `/joint_states` publishes 6 joints at runtime: shoulder_pan, shoulder_lift, elbow, wrist_1/2/3
- ✅ JointStateSubscriber.cs in place; joints register correctly in Unity Console
- ✅ Digital twin working (Unity mirrors real UR3e joints) — fixed joint name mismatch between ROS and Unity
- ✅ Quest 3 build connects to ROS over WiFi (ROS IP set to laptop's WiFi IP, ros_tcp_endpoint on 0.0.0.0)
- ✅ RobotController.cs created with two control modes (Direct Joint + MoveIt Servo) — rewritten to use Unity Input System (was OVRInput, which didn't work without OVRManager)
- ✅ MoveIt 2 + Servo config already present in `ur_moveit_config` package (launch with `launch_servo:=true`)
- ✅ RobotBasePlacer.cs working — hand-tracking pinch-to-grab and drag robot in MR (on parent wrapper GameObject)
- ✅ JointStateSubscriber.cs rewritten to be purely kinematic (no ArticulationBody physics, direct Transform rotations)
- ✅ Joint axes fixed — extracts actual axis from ArticulationBody anchorRotation, composes with initial URDF rest rotation
- ✅ Joint limits added from URDF (elbow ±180°, others ±360°)
- ✅ Digital twin verified on Quest 3 — joints match real robot direction and axes
- ✅ RobotControlActions.inputactions created — dedicated Input System actions for robot control
- ✅ RobotController.cs disables conflicting XRI locomotion action maps at runtime (teleport, turn, snap turn, move, jump, rotate manipulation)
- ✅ RobotHUD.cs created — floating TextMeshPro panel follows camera, shows mode + selected joint
- ⬜ Test RobotController.cs on Quest 3 (not yet tested — script compiles but needs real hardware test)
- ⬜ Wire up RobotControlActions asset in Inspector (drag into RobotController's Input Actions field)
- ⬜ Test MoveIt Servo mode end-to-end (requires Terminal 3: `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false launch_servo:=true`)
- ⬜ Test full teleoperation loop with real robot

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
      Scripts/RobotController.cs       — robot control (Direct Joint + MoveIt Servo modes)
      Scripts/RobotBasePlacer.cs       — hand-tracking pinch-to-grab robot placement
      Scripts/RobotHUD.cs              — floating HUD showing mode + selected joint
      Scripts/RobotControlActions.inputactions — Unity Input System bindings for robot control
      URDF/                            — ur3e.urdf + meshes
      (RosMessages/ deleted — types built into ROS-TCP-Connector)
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
# Check laptop IPs first (Ethernet for robot, WiFi for Quest)
hostname -I

# Terminal 1
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true

# Terminal 2
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 3 (only for MoveIt Servo mode)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false launch_servo:=true
```

Then on the teach pendant: load and run External Control program (Host IP: `192.168.0.100`).
Then hit Play in Unity.

**Note:** Terminal 3 is required for Servo mode. Without it, `/servo_node/delta_twist_cmds` has no subscriber. Direct Joint mode works with just Terminals 1+2.

### Network Setup

```
Laptop (nic-XPS-15-9520):
  ├─ Ethernet (192.168.0.100) ←──→ UR3e robot (192.168.0.194)
  └─ WiFi (172.19.115.245)    ←──→ Quest 3 (172.19.119.95)
```

- **Teach pendant External Control host IP**: `192.168.0.100` (laptop Ethernet)
- **Unity ROS Settings IP**: laptop's WiFi IP (e.g., `172.19.115.245`) — this is what the Quest connects to
- **ros_tcp_endpoint**: `ROS_IP:=0.0.0.0` (listens on all interfaces, bridging both networks)
- WiFi IP may change if network changes — re-check with `hostname -I`

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
- **ROS IP:** Set to laptop's WiFi IP for Quest builds (e.g., `172.19.115.245`), Port: 10000 (configured in Robotics → ROS Settings). Use `0.0.0.0` for ros_tcp_endpoint so it listens on all interfaces.

### Key scripts

- `Assets/Scripts/JointStateSubscriber.cs` — attach to root `ur` GameObject; subscribes to `/joint_states` and sets joint rotations directly via `Transform.localRotation` (no ArticulationBody physics). Disables all ArticulationBody components on Start. Contains `rosToUnity` name mapping dictionary.
- `Assets/Scripts/RobotBasePlacer.cs` — attach to an **empty parent wrapper** (e.g. `RobotRig`) that contains `ur` (and optionally the trolley) as children. Uses XR hand tracking — pinch to grab, drag to move. Pure transform movement, no physics.
- `Assets/Scripts/RobotController.cs` — attach to any GameObject; uses **Unity Input System** (not OVRInput). Requires `RobotControlActions.inputactions` dragged into the Inspector's **Input Actions** field. Two control modes switched via Menu button:
  - **Direct Joint mode** (default): A/B to cycle joints, right stick Y for smooth proportional velocity control of selected joint. Sends trajectory points with both position AND velocity fields so the controller interpolates smoothly. Publishes continuously (even at zero input) so the robot holds position. Publishes `JointTrajectory` to `/scaled_joint_trajectory_controller/joint_trajectory` (no MoveIt needed).
  - **MoveIt Servo mode**: left stick = XY translation, right stick = Z translation + yaw. Publishes `TwistStamped` to `/servo_node/delta_twist_cmds`. **Requires MoveIt Servo running** (Terminal 3). Sends commands continuously (including zeros) to prevent servo 100ms idle timeout. Messages are timestamped — servo rejects stale messages.
  - On Start, automatically disables conflicting XRI Default Input Actions (locomotion maps, Jump, Rotate Manipulation, UI Scroll) so thumbsticks and A/B buttons are not consumed by teleport/turn/move.
  - Publish rate: 30Hz (must be >10Hz to stay within servo's 100ms `incoming_command_timeout`).
- `Assets/Scripts/RobotControlActions.inputactions` — Unity Input System action asset with bindings: LeftStick, RightStick, NextJoint (A), PrevJoint (B), ToggleMode (Menu). Deadzone handled via StickDeadzone processor.

### Controller mapping (Quest 3)

| Input | Direct Joint Mode | MoveIt Servo Mode |
|---|---|---|
| **Menu button** (left) | Toggle to Servo | Toggle to Direct Joint |
| **A button** (right) | Next joint | — |
| **B button** (right) | Previous joint | — |
| **Right stick Y** | Jog selected joint (proportional velocity) | End-effector up/down (Z) |
| **Right stick X** | — | End-effector yaw |
| **Left stick Y** | — | End-effector forward/back (X) |
| **Left stick X** | — | End-effector left/right (Y) |
- `Assets/Scripts/RobotHUD.cs` — attach to an empty GameObject; assign the `RobotController` reference in Inspector. Floating TextMeshPro panel tracks the camera, shows current mode and selected joint.

## Known Issues

- **UR driver segfault (exit code -11)** — apt-installed `ros-humble-ur-robot-driver` crashes on Ubuntu 22.04. Fixed by building `Universal_Robots_ROS2_Driver` from source (already in `ros2_ws/src/`).
- **Robot falls in Unity on Play** — no longer an issue. JointStateSubscriber.cs now disables all ArticulationBody components entirely (purely kinematic). No physics simulation on the robot.
- **Git HTTPS clone fails on this machine** — always use SSH (`git@github.com:...`).
- **Unity Package Manager can't find robotics packages** — clone locally, reference via `file://` in `Packages/manifest.json`.
- **ros_tcp_endpoint crashes on Unity disconnect/reconnect** — known race condition in v0.7.0 (`InvalidHandle` exception). Restart the endpoint after every Unity Play/Stop cycle. Command must be on one line (no line breaks in `--ros-args`): `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`
- **JointStateSubscriber: ArticulationBody approach abandoned** — `SetDriveTarget` with high stiffness caused physics conflicts with hand-tracking grab. Rewritten to disable ArticulationBody entirely and set `Transform.localRotation` directly. Joint axis assumed `Vector3.forward` — needs verification with real robot.
- **`/joint_states` not arriving in Unity despite subscription registering** — caused by duplicate message class registration. Delete `Assets/RosMessages/` entirely; those types are already built into ROS-TCP-Connector and the duplicates silently break deserialization. (Fixed — RosMessages deleted.)
- **ROS joint names don't match Unity GameObject names** — URDF importer names GameObjects after links (`shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, etc.) but ROS publishes joint names (`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, etc.). Fixed with a `rosToUnity` mapping dictionary in JointStateSubscriber.cs.
- **Quest 3 networking** — laptop connects to robot via Ethernet (`192.168.0.100` ↔ `192.168.0.194`), Quest connects to laptop via WiFi. Both must be on the same WiFi network. Set ROS IP in Unity to laptop's WiFi IP. Teach pendant External Control host IP = `192.168.0.100`.
- **OVRInput does nothing on Quest 3 (MR template)** — the Mixed Reality template uses Unity Input System + XR Interaction Toolkit, not OVR. `OVRInput.Get()` silently returns zero without `OVRManager` in the scene. Fixed by rewriting RobotController.cs to use `InputAction` from Unity Input System with `<XRController>` bindings.
- **XRI Default Input Actions consume thumbsticks/buttons** — the MR template's XRI actions bind both thumbsticks to teleport/turn/move and A button to Jump. RobotController.cs now disables these conflicting action maps at runtime (`XRI Left Locomotion`, `XRI Right Locomotion`, plus individual Rotate Manipulation and UI Scroll actions).
- **MoveIt Servo config** — `ros2_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/ur_servo.yaml`: `command_in_type: speed_units` (m/s, rad/s), `incoming_command_timeout: 0.1` (100ms), `robot_link_command_frame: tool0`, `publish_period: 0.004` (250Hz output), outputs to `forward_position_controller`. Servo switches the active ros2_control controller — do not expect `scaled_joint_trajectory_controller` to work while Servo is active.
