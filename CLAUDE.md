# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR teleoperation framework integrating a Meta Quest 3 headset with a Universal Robots UR3e collaborative arm. This branch (`nic`) is one of several personal branches in a shared research team repository (other branches: `john`, `ollie`, `seb`).

The current focus is **XR teleoperation**: Quest 3 controllers drive the UR3e end-effector via Cartesian velocity commands using RMRC (Resolved Motion Rate Control) — Jacobian-based velocity resolution computed locally in Unity, no MoveIt needed.

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
- ✅ RobotController.cs created with three control modes (Direct Joint + RMRC + Hand Guide) — rewritten to use Unity Input System (was OVRInput, which didn't work without OVRManager)
- ✅ MoveIt 2 + Servo config already present in `ur_moveit_config` package (launch with `launch_servo:=true`)
- ✅ RobotBasePlacer.cs working — controller grip-to-grab and drag robot in MR (on parent wrapper GameObject)
- ✅ JointStateSubscriber.cs rewritten to be purely kinematic (no ArticulationBody physics, direct Transform rotations)
- ✅ Joint axes fixed — extracts actual axis from ArticulationBody anchorRotation, composes with initial URDF rest rotation
- ✅ Joint limits added from URDF (elbow ±180°, others ±360°)
- ✅ Digital twin verified on Quest 3 — joints match real robot direction and axes
- ✅ RobotControlActions.inputactions created — dedicated Input System actions for robot control
- ✅ RobotController.cs disables conflicting XRI locomotion action maps at runtime (teleport, turn, snap turn, move, jump, rotate manipulation)
- ✅ RobotHUD.cs created — colour-coded floating HUD with mode, control hints, readable joint names
- ✅ RobotController.cs tested on Quest 3 — teleoperation works but was jerky in Direct Joint mode
- ✅ UR3eKinematics.cs created — forward kinematics + geometric Jacobian + DLS inverse for RMRC
- ✅ RobotController.cs rewritten: Servo mode replaced with RMRC mode (Jacobian-based Cartesian velocity control, no MoveIt needed)
- ✅ Dashboard app created — Python/PyQt5 e-stop + debug console, streams to Steam Deck OLED
- ✅ HeadsetStreamPublisher.cs created — renders Unity scene view to dashboard via `/headset/image_compressed` (render capture, no passthrough)
- ✅ Dashboard HEADSET tab verified working — shows Unity scene from operator's XR viewpoint
- ✅ launch.sh / launch.py created — one-command launcher for UR driver + controller switch + TCP endpoint (fake hardware default, --robot-ip for real)
- ✅ dashboard.sh created — sources ROS automatically before launching dashboard
- ✅ RMRC mode tested on Quest 3 with real robot — works, some lag but functional
- ✅ Full teleoperation loop with RMRC verified
- ✅ SpatialMarkers.cs created — RGB axes on tool0 + yellow velocity arrow (coordinate fix applied)
- ✅ RMRC Rotate sub-mode added — X button toggles Translate/Rotate, direct wrist joint control (joints 3/4/5)
- ✅ Hand Guide mode added — hold right grip to track controller position in 3D space, robot EE follows hand via Jacobian IK. Mellow gain (2) + speed cap (0.15 m/s) + joint bias favouring wrist over base/shoulder. Axis mapping verified on Quest 3.
- ✅ Subsystem 3 evaluation renegotiated — HD/Perfect criteria revised to remove cross-subsystem dependencies (see `subsystem3_renegotiation.md`)
- ✅ SessionLogger.cs created — tracks mode switches, per-mode durations, session time; publishes JSON to `/session/status` (2Hz) + `/session/events`; saves session log JSON to `Application.persistentDataPath/SessionLogs/` on quit
- ✅ Dashboard STATS tab reworked — session info bar (duration, mode, switches, e-stops, mode usage %) + rolling joint velocity graph (6 joints, 30s) + topic health graph (3 topics as % of expected rate, 60s)
- ✅ Dashboard LATENCY tab added — live latency numbers + rolling message age graph (joint_state & velocity_cmd freshness, 30s) + command interval graph (time between velocity commands, 30s)
- ✅ Dashboard SESSION tab added — text-based overview: control mode, sub-mode, hand guide state, mode durations, connection status, all topic rates
- ✅ Dashboard saves session log to `~/holoassist_sessions/` on shutdown (events, e-stop count, Unity session info)
- ⬜ Resilience under adverse conditions (WiFi dropout → auto e-stop, latency spike detection, graceful recovery) — needed for Perfect grade

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
      Scripts/RobotController.cs       — robot control (RMRC + Direct Joint + Hand Guide modes)
      Scripts/UR3eKinematics.cs        — UR3e forward kinematics + Jacobian for RMRC
      Scripts/RobotBasePlacer.cs       — controller grip-to-grab robot placement
      Scripts/RobotHUD.cs              — colour-coded floating HUD with mode, controls, joint names
      Scripts/SpatialMarkers.cs        — end-effector axes + velocity arrow visualisation
      Scripts/HeadsetStreamPublisher.cs — captures XR camera view, publishes JPEG to ROS for dashboard
      Scripts/SessionLogger.cs           — session metrics: mode tracking, durations, events → ROS + file
      Scripts/RobotControlActions.inputactions — Unity Input System bindings for robot control
      URDF/                            — ur3e.urdf + meshes
      (RosMessages/ deleted — types built into ROS-TCP-Connector)
  dashboard/           — Python/PyQt5 e-stop & debug dashboard (streamed to Steam Deck)
    main.py            — PyQt5 UI (status bar, tabbed screens, e-stop column)
    ros_interface.py   — rclpy node (velocity publisher, joint subscriber, controller switching)
  launch.py            — Python launcher: starts UR driver, controller switch, TCP endpoint
  launch.sh            — Shell wrapper for launch.py (sources ROS automatically)
  dashboard.sh         — Shell wrapper to run dashboard with ROS sourced
  ROS-TCP-Connector/  — Unity package (local, referenced via file:// in manifest.json)
  URDF-Importer/      — Unity package (local, referenced via file:// in manifest.json)
  SETUP.md            — full onboarding guide for new contributors
  subsystem3_renegotiation.md  — Nic's proposed evaluation criteria changes (no cross-subsystem deps)
  subsystem2_renegotiation_options.md — Options for Oliver's subsystem renegotiation
```

## Development Environment

**Prerequisites:** ROS 2 Humble on Ubuntu 22.04, Unity 6.3 LTS.

**Do not use Docker** — the apt-installed UR driver has a segfault bug on Ubuntu 22.04; build from source instead (already done in this workspace).

### Quick Start (launcher scripts)

```bash
# Fake hardware (default) — no real robot needed
./launch.sh

# Real robot
./launch.sh --robot-ip 192.168.0.194

# No RViz (lighter)
./launch.sh --no-rviz

# Dashboard (separate terminal)
./dashboard.sh --fullscreen

# Dashboard without ROS (UI testing only)
./dashboard.sh --no-ros --fullscreen
```

`launch.sh` and `dashboard.sh` source ROS automatically. `launch.py` accepts `--robot-ip`, `--ros-ip`, and `--no-rviz` flags. No `--robot-ip` = fake hardware mode.

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

# Terminal 3 (activate forward_velocity_controller for RMRC / Direct Joint)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers --activate forward_velocity_controller --deactivate scaled_joint_trajectory_controller
```

Then on the teach pendant: load and run External Control program (Host IP: `192.168.0.121`).
Then hit Play in Unity.

**Note:** Both RMRC and Direct Joint modes publish to `/forward_velocity_controller/commands`. The `forward_velocity_controller` must be active — check with `ros2 control list_controllers`. MoveIt Servo is no longer used for teleoperation.

### Simulated Hardware (no real robot)

```bash
# Terminal 1 — fake UR3e hardware
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=0.0.0.0 use_fake_hardware:=true fake_sensor_commands:=true launch_rviz:=true

# Terminal 2 — switch to velocity controller (wait for Terminal 1 to finish loading)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers --activate forward_velocity_controller --deactivate scaled_joint_trajectory_controller

# Terminal 3 — ROS-TCP bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

No teach pendant needed. Ignore `urscript_interface` connection errors — that node doesn't check for fake hardware mode. Joint states and controllers work fine. Robot moves in RViz and Unity digital twin.

### Network Setup

```
Laptop (nic-XPS-15-9520):
  ├─ Ethernet (192.168.0.121) ←──→ UR3e robot (192.168.0.194)
  └─ WiFi (172.19.115.104)    ←──→ Quest 3 (172.19.119.95)
```

- **Teach pendant External Control host IP**: `192.168.0.121` (laptop Ethernet)
- **Unity ROS Settings IP**: laptop's WiFi IP (e.g., `172.19.115.104`) — this is what the Quest connects to
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
- `Assets/Scripts/RobotBasePlacer.cs` — attach to an **empty parent wrapper** (e.g. `RobotRig`) that contains `ur` (and optionally the trolley) as children. Uses XR controller grip button — hold grip near rig to grab and drag. Supports Y-axis rotation while grabbed (keeps robot upright). Pure transform movement, no physics.
- `Assets/Scripts/UR3eKinematics.cs` — static utility class. Forward kinematics using standard DH parameters, numerical Jacobian (3x6 linear, finite differences on FK), geometric angular Jacobian (3x6, joint z-axes). DLS pseudoinverse for both linear and angular velocity resolution. Manipulability measure for adaptive damping.
- `Assets/Scripts/SpatialMarkers.cs` — attach to robot root GameObject (must contain `tool0` in hierarchy). RGB axis cylinders on end-effector + yellow velocity arrow showing commanded Cartesian velocity direction/magnitude.
- `Assets/Scripts/RobotController.cs` — attach to any GameObject; uses **Unity Input System** (not OVRInput). Requires `RobotControlActions.inputactions` dragged into the Inspector's **Input Actions** field. Three control modes cycled via Menu button (RMRC → Direct Joint → Hand Guide → RMRC):
  - **RMRC mode** (default): Resolved Motion Rate Control with two sub-modes toggled by X button (left controller):
    - **Translate** (default): right stick = XY translation (forward/back, left/right), left stick Y = Z translation (up/down), left stick X = yaw. Uses 3x6 linear Jacobian.
    - **Rotate**: directly drives wrist joints — right stick Y = wrist_1 (pitch/tilt), right stick X = wrist_2 (roll), left stick X = wrist_3 (yaw/spin). Uses `jointJogSpeed` for responsive 1:1 control (angular Jacobian approach was too sluggish).
    - Both use `UR3eKinematics` DLS pseudoinverse. Adaptive damping near singularities. Proportional joint velocity scaling for safety. Publishes `Float64MultiArray` to `/forward_velocity_controller/commands`. No MoveIt needed.
  - **Direct Joint mode**: A/B to cycle joints, right stick Y for smooth proportional velocity control of selected joint. Publishes `Float64MultiArray` to `/forward_velocity_controller/commands`.
  - **Hand Guide mode**: Hold right grip trigger to engage — controller 3D position is tracked and mapped to end-effector position via proportional control + Jacobian IK. Release grip to stop. Requires `robotBase` field set to the `ur` GameObject in Inspector. Tunable `positionGain` (default 2) and `maxTrackingSpeed` (default 0.15 m/s) — deliberately mellow so robot gently catches up to hand. Joint bias weights (`[0.3, 0.3, 0.5, 1.0, 1.0, 1.0]`) favour wrist movement over base/shoulder. Axis mapping verified on Quest 3 (DH_x=-Unity_z, DH_y=Unity_x, DH_z=Unity_y).
  - On Start, automatically disables conflicting XRI Default Input Actions (locomotion maps, Jump, Rotate Manipulation, UI Scroll) so thumbsticks and A/B buttons are not consumed by teleport/turn/move.
  - Publish rate: 50Hz.
- `Assets/Scripts/RobotControlActions.inputactions` — Unity Input System action asset with bindings: LeftStick, RightStick, NextJoint (A), PrevJoint (B), ToggleMode (Menu). Deadzone handled via StickDeadzone processor.

### Controller mapping (Quest 3)

| Input | Direct Joint Mode | RMRC Translate | RMRC Rotate | Hand Guide |
|---|---|---|---|---|
| **Menu button** (left) | → Hand Guide | → Direct Joint | → Direct Joint | → RMRC |
| **X button** (left) | — | Toggle to Rotate | Toggle to Translate | — |
| **A button** (right) | Next joint | — | — | — |
| **B button** (right) | Previous joint | — | — | — |
| **Right grip** (right) | — | — | — | Hold to track hand |
| **Right stick Y** | Jog selected joint | End-effector forward/back (X) | Wrist 1 (pitch/tilt) | — |
| **Right stick X** | — | End-effector left/right (Y) | Wrist 2 (roll) | — |
| **Left stick Y** | — | End-effector up/down (Z) | — | — |
| **Left stick X** | — | End-effector yaw | Wrist 3 (yaw/spin) | — |
- `Assets/Scripts/HeadsetStreamPublisher.cs` — attach to an empty GameObject (e.g. `HeadsetStream`). Renders the Unity scene from the XR camera's perspective via a hidden secondary camera, JPEG-encodes it, and publishes `CompressedImage` to `/headset/image_compressed` at configurable FPS (default 15). The capture camera uses a skybox background so the dashboard sees the full scene (does not affect the user's passthrough XR view). No webcam/passthrough capture — Quest 3 doesn't expose passthrough as a camera device.
- `Assets/Scripts/SessionLogger.cs` — attach to any GameObject; assign `RobotController` in Inspector (auto-finds via `FindObjectOfType` if not set). Publishes JSON session status to `/session/status` at 2Hz (mode, sub-mode, session time, mode switches, per-mode durations) and discrete events to `/session/events`. On application quit, saves a JSON session log to `Application.persistentDataPath/SessionLogs/session_YYYY-MM-DD_HH-MM-SS.json`. On Quest 3 the persistent path is `/storage/emulated/0/Android/data/com.DefaultCompany.MixedRealityTemplate/files/SessionLogs/` — pull via `adb pull`.
- `Assets/Scripts/RobotHUD.cs` — attach to an empty GameObject; assign the `RobotController` reference in Inspector. Floating HUD panel tracks the camera with colour-coded mode title (amber=Joint, green=Translate, blue=Rotate, purple/red=Hand Guide), control hints for current mode, and readable joint names in Direct Joint mode.

## Dashboard (E-Stop & Debug Console)

- **Path:** `dashboard/`
- **Requirements:** PyQt5, rclpy (from ROS 2)
- **Target display:** Steam Deck OLED (1280x800) via Ubuntu desktop streaming from laptop (3456x2160 HiDPI)

### Running

```bash
# With ROS:
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 dashboard/main.py --fullscreen

# Without ROS (UI testing):
python3 dashboard/main.py --no-ros --fullscreen

# Custom scale (default 2.5x in fullscreen for HiDPI→Steam Deck):
QT_SCALE_FACTOR=2.0 python3 dashboard/main.py --fullscreen
```

### Architecture

- `main.py` — PyQt5 GUI. Status bar (top), 6 tabbed screens (left), e-stop column (right, always visible). Polls ROS status at 30Hz. F11 toggles fullscreen. Left/Right arrow keys cycle tabs. Includes `RollingGraph` QPainter widget for real-time line charts.
- `ros_interface.py` — rclpy node (`holoassist_dashboard`). Independent of Unity/ros_tcp_endpoint — talks directly to ROS 2. Subscribes to `/session/status` and `/session/events` from Unity `SessionLogger`. Samples rolling data buffers: joint velocities (10Hz, 30s), topic health % (2Hz, 60s), latency metrics (10Hz, 30s). Saves dashboard-side session log to `~/holoassist_sessions/` on shutdown.

### E-Stop Behaviour

1. **Trigger:** Click the red EMERGENCY STOP button
2. **Immediate:** Burst-publishes `[0,0,0,0,0,0]` to `/forward_velocity_controller/commands` (10x), then continuous zeros at 50Hz
3. **Follow-up:** Deactivates `forward_velocity_controller` via `ros2 control switch_controllers`
4. **Resume:** Hold yellow RESUME button for 5 seconds → reactivates `forward_velocity_controller` + deactivates `scaled_joint_trajectory_controller`
5. **Cooldown:** 1-second cooldown after resume prevents accidental re-trigger

### Screens (tabs)

| Tab | Status | Description |
|---|---|---|
| STATUS | Working | Joint positions/velocities, event log |
| HEADSET | Working | Quest 3 XR scene view (JPEG stream from `HeadsetStreamPublisher.cs`) |
| CAMERA | Placeholder | Depth camera ROS topics |
| STATS | Working | Session info bar (duration, mode, switches, e-stops, mode %) + rolling joint velocity graph (6 coloured lines, 30s) + topic health graph (joint_states/vel_cmd/headset as % of expected Hz, 60s) |
| LATENCY | Working | Live latency numbers (colour-coded) + rolling message age graph (joint_state & vel_cmd freshness, 30s) + command interval graph (time between velocity commands, 30s) |
| SESSION | Working | Text overview: control mode, sub-mode, hand guide state, mode durations, e-stop count, connection status, all 11 topic rates |

## Known Issues

- **UR driver segfault (exit code -11)** — apt-installed `ros-humble-ur-robot-driver` crashes on Ubuntu 22.04. Fixed by building `Universal_Robots_ROS2_Driver` from source (already in `ros2_ws/src/`).
- **Robot falls in Unity on Play** — no longer an issue. JointStateSubscriber.cs now disables all ArticulationBody components entirely (purely kinematic). No physics simulation on the robot.
- **Git HTTPS clone fails on this machine** — always use SSH (`git@github.com:...`).
- **Unity Package Manager can't find robotics packages** — clone locally, reference via `file://` in `Packages/manifest.json`.
- **ros_tcp_endpoint crashes on Unity disconnect/reconnect** — known race condition in v0.7.0 (`InvalidHandle` exception). Restart the endpoint after every Unity Play/Stop cycle. Command must be on one line (no line breaks in `--ros-args`): `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`
- **JointStateSubscriber: ArticulationBody approach abandoned** — `SetDriveTarget` with high stiffness caused physics conflicts with hand-tracking grab. Rewritten to disable ArticulationBody entirely and set `Transform.localRotation` directly. Joint axis assumed `Vector3.forward` — needs verification with real robot.
- **`/joint_states` not arriving in Unity despite subscription registering** — caused by duplicate message class registration. Delete `Assets/RosMessages/` entirely; those types are already built into ROS-TCP-Connector and the duplicates silently break deserialization. (Fixed — RosMessages deleted.)
- **ROS joint names don't match Unity GameObject names** — URDF importer names GameObjects after links (`shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, etc.) but ROS publishes joint names (`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, etc.). Fixed with a `rosToUnity` mapping dictionary in JointStateSubscriber.cs.
- **Quest 3 networking** — laptop connects to robot via Ethernet (`192.168.0.100` ↔ `192.168.0.194`), Quest connects to laptop via WiFi. Both must be on the same WiFi network. Set ROS IP in Unity to laptop's WiFi IP. Teach pendant External Control host IP = `192.168.0.121`.
- **OVRInput does nothing on Quest 3 (MR template)** — the Mixed Reality template uses Unity Input System + XR Interaction Toolkit, not OVR. `OVRInput.Get()` silently returns zero without `OVRManager` in the scene. Fixed by rewriting RobotController.cs to use `InputAction` from Unity Input System with `<XRController>` bindings.
- **XRI Default Input Actions consume thumbsticks/buttons** — the MR template's XRI actions bind both thumbsticks to teleport/turn/move and A button to Jump. RobotController.cs now disables these conflicting action maps at runtime (`XRI Left Locomotion`, `XRI Right Locomotion`, plus individual Rotate Manipulation and UI Scroll actions).
- **MoveIt Servo not currently used** — RMRC replaces MoveIt Servo for teleoperation. Servo config still exists in `ur_moveit_config/config/ur_servo.yaml` if needed later for collision-aware planning.
- **forward_velocity_controller must be active** — both RMRC and Direct Joint modes publish `Float64MultiArray` to `/forward_velocity_controller/commands`. After launching the UR driver, switch controllers: `ros2 control switch_controllers --activate forward_velocity_controller --deactivate scaled_joint_trajectory_controller`.
- **RobotHUD may not render on Quest 3** — `Camera.main` can return null in the MR template (camera not tagged MainCamera), `Shader.Find` may return null on Android builds (shader stripping), and runtime-created TextMeshPro needs an explicit font asset. Partially patched with fallback camera/shader/font search but not fully verified on device. Functional enough for now.
