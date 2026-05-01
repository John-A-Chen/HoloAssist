# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR robotic sorting system. A UR3e collaborative arm with an OnRobot gripper sorts physical objects into bins within a workspace. An Intel RealSense depth camera detects and identifies objects as they enter the workspace, and the system operates in two modes:

- **Teleoperation mode** — a human operator wearing a Meta Quest 3 headset controls the robot arm and gripper in mixed reality to pick and sort objects.
- **Autonomous mode** — the robot autonomously sorts objects using MoveIt 2 trajectory planning without human input.

Real-world objects are mapped to different virtual objects in the XR scene (e.g., a physical tomato appears as a bomb that needs "defusing" into the correct bin), creating a gamified operator experience.

This branch (`nic`) is one of several personal branches in a shared research team repository (other branches: `john`, `ollie`, `seb`).

### Subsystem Ownership

| Subsystem | Lead | Scope |
|---|---|---|
| Perception | John | RealSense depth camera, object detection/classification, pose tracking |
| Autonomous Sorting | Oliver | MoveIt 2 trajectory planning, autonomous pick-and-place |
| Teleoperation & Interaction | Nic | XR teleop (RMRC, Direct Joint, Hand Guide), gripper control, e-stop dashboard, session logging |
| XR Scene & Visualisation | Sebastian | Unity environment, virtual object overlays (real->XR mapping), bin visualisation |

### Evaluation Criteria (Nic's Subsystem — from official contract)

| Grade | Summary | Status |
|---|---|---|
| **P** | Stable digital twin on Quest 3 — JointStateSubscriber drives joints, purely kinematic, verified on device | DONE |
| **C** | RMRC + Direct Joint + e-stop dashboard — Jacobian velocity control, 50Hz publish, dashboard with burst e-stop + resume | DONE |
| **D** | Three modes + headset streaming dashboard — RMRC/Direct Joint/Hand Guide, translate/rotate sub-modes, HeadsetStreamPublisher at 15 FPS, 5 dashboard tabs, launch scripts | DONE |
| **HD** | Hand Guide mode + session logging + gripper — Jacobian IK tracking (gain 2, 0.15 m/s cap), SessionLogger (2Hz + file), **OnRobot gripper open/close from XR controller** | DONE |

Full criteria details in `evaluation/subsystem3_nic_teleoperation.md`. All subsystem evaluations in `evaluation/`. Official contract: `RS2 Project Contract Renegotiated.pdf`.

### System-Level Evaluation (shared team responsibility)

| Grade | Criteria |
|---|---|
| **P** | XR headset displays stable robot model and EE pose. Operator can teleoperate via reactive velocity control. System stops safely on communication loss. Depth camera publishes to desktop UI. Operator can observe pose markers/transforms in XR. |
| **C** | Autonomous mode operational — MoveIt 2 plans and executes basic pick-and-place. Teleop sorting achievable in repeated trials. Gripper open/close integrated from XR controller. |
| **D** | XR virtual object mapping operational (e.g. tomato → bomb). Mode switching between teleop and autonomous validated. Demonstration task completed consistently under defined time threshold. Passthrough / spatial marker visualisation working. |
| **HD** | Human vs CPU race mode overlay functional. Quantitative comparison between teleop and autonomous modes (sort time, accuracy). Mixed reality UI visualises and debugs depth camera feed. WiFi dropout auto e-stop implemented. |

### Progress Status (Nic's Subsystem)
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
- ✅ Hand Guide mode added — hold right grip to track controller position in 3D space, robot EE follows hand via Jacobian IK
- ✅ SessionLogger.cs created — tracks mode switches, per-mode durations, session time; publishes JSON to `/session/status` (2Hz) + `/session/events`; saves session log JSON on quit
- ✅ Dashboard STATS tab — session info bar + rolling joint velocity graph + topic health graph
- ✅ Dashboard LATENCY tab — live latency numbers + message age graph + command interval graph
- ✅ Dashboard SESSION tab — text overview of control mode, durations, connection status, topic rates
- ✅ Dashboard saves session log to `~/holoassist_sessions/` on shutdown
- ✅ OnRobot RG2 gripper URDF created — `ur3e_rg2.urdf` combines UR3e + RG2 gripper (attached to tool0), STL meshes in `Assets/URDF/onrobot_rg2/`
- ✅ RG2 URDF imported in Unity — replaced old `ur` GameObject with `ur3e_rg2`, visual STLs renamed to `*_visual.stl` to work around URDF-Importer name-collision bug (collision/visual share filenames → `UsedTemplateFiles` skip)
- ✅ New robot verified working — all 6 arm joints mirror correctly, velocity arrow enlarged (0.25m length, 0.008m thick) to clear gripper geometry
- ✅ Gripper control from XR — right index trigger (analog 0–100%) maps to gripper width, publishes Float64MultiArray (metres) to `/finger_width_controller/commands` via `ur_onrobot` driver. Digital twin animates all 6 gripper joints (finger_joint + 5 mimic joints) from `finger_width` in `/joint_states`. Default grip force lowered to 5N (was 20N) in `onrobot_driver/src/RG.cpp`. Hardware interface only sends Modbus commands when width changes (0.5mm threshold), and re-syncs command to current width on activation to prevent snap-close.
- ✅ **Switched to `UR_OnRobot_ROS2` driver** (by Tony Le) — replaces separate UR driver + custom `gripper_node.py`. Combined launch (`ur_onrobot_control start_robot.launch.py`) handles UR3e + RG2 gripper natively via C++ Modbus (no more socat/pymodbus hack). Old `gripper_node.py` is now obsolete.
- ✅ **Collision protection rewritten (mesh-based)** — `MeshCollisionGuard.cs` replaces old point-based system. Auto-discovers all MeshColliders in robot hierarchy, sets them convex for `ClosestPoint` queries. Two collision layers: (1) **Table** — checks each collider's bounding box min Y against table height; (2) **Self-collision** — proximal links (base/shoulder/upper_arm) vs distal links (wrist_1 through gripper) using `ClosestPoint` distance. Both use soft-zone gradual slowdown + hard stop. Gizmo drawing in Scene view (table planes, collider bounds colour-coded by zone). Attach to `ur3e_rg2`, assign `robotBase` to itself, drag into `RobotController.collisionGuard` field.
- ✅ EE lock-down mode — Y button toggles; uses combined 6x6 Jacobian solve (`ResolveFullVelocity`) so linear + angular goals are resolved simultaneously (no jitter from competing solves). Works immediately without grip. 1° dead zone, gain 0.5, max 0.3 rad/s (slow gentle transition). Works in RMRC Translate + Hand Guide modes. HUD shows "LOCK ▼" when active.
- ✅ Output velocity smoothing (60ms exponential low-pass) on all joint velocities before publishing. Resets on mode switch.
- ✅ Dashboard shows gripper status (bar + percentage) on STATUS tab
- ✅ RobotHUD shows gripper bar + EE lock indicator in all modes
- ⚠️ **Performance/lag** — noticeable lag during teleoperation, even on fake hardware (not just Quest 3 WiFi). Gripper lag is partly the ROS round-trip (Unity → ros_tcp_endpoint → controller → `/joint_states` → Unity). Possible fixes: (1) add local gripper preview in `JointStateSubscriber` using `RobotController.GripperValue` for instant visual while still publishing to ROS; (2) reduce GC pressure — pre-allocate arrays in UR3eKinematics; (3) check if ros_tcp_endpoint is the bottleneck.
- ⬜ **Tuning needed** — linear movement too fast, wrist rotation (RMRC Rotate sub-mode) too slow. Adjust `linearSpeed` (currently 0.25 m/s — reduce) and `jointJogSpeed` (currently 0.5 rad/s — increase for wrist joints in Rotate mode).
- ⬜ **Collision protection calibration** — `MeshCollisionGuard` implemented, needs `tableWorldY` or `tableTransform` set to actual table height, and self-collision margins tuned on real hardware.
- ✅ WiFi reliability solved — laptop + Quest 3 both on robot's dedicated router (192.168.0.x subnet), no more university WiFi dropouts

## Repository Layout

```
nic/
  ros2_ws/          — ROS 2 workspace
    src/
      ROS-TCP-Endpoint/              — Unity-ROS TCP bridge (cloned from source)
      Universal_Robots_ROS2_Driver/  — UR driver (dependency of ur_onrobot)
      ur_onrobot/                    — Combined UR + OnRobot driver (launch, controllers, URDF)
      onrobot_driver/                — OnRobot hardware interface (C++ Modbus)
      onrobot_description/           — OnRobot gripper URDF/meshes
  Unity/My project/ — Unity 6.3 project
    Assets/
      Scripts/JointStateSubscriber.cs  — subscribes to /joint_states, drives robot joints
      Scripts/RobotController.cs       — robot control (RMRC + Direct Joint + Hand Guide modes)
      Scripts/UR3eKinematics.cs        — UR3e forward kinematics + Jacobian for RMRC
      Scripts/RobotBasePlacer.cs       — controller grip-to-grab robot placement
      Scripts/RobotHUD.cs              — colour-coded floating HUD with mode, controls, joint names
      Scripts/SpatialMarkers.cs        — end-effector axes + velocity arrow visualisation
      Scripts/MeshCollisionGuard.cs     — mesh-based collision protection (table + self-collision)
      Scripts/CollisionDebugVisualizer.cs — collision status on-screen label
      Scripts/ROSAutoConnect.cs            — auto-discovers ros_tcp_endpoint on 192.168.0.101–109:10000 via TCP scan
      Scripts/HeadsetStreamPublisher.cs — captures XR camera view, publishes JPEG to ROS for dashboard
      Scripts/SessionLogger.cs           — session metrics: mode tracking, durations, events → ROS + file
      Scripts/RobotControlActions.inputactions — Unity Input System bindings for robot control
      URDF/ur3e.urdf                     — original UR3e URDF (no gripper)
      URDF/ur3e_rg2.urdf                 — UR3e + OnRobot RG2 gripper combined URDF
      URDF/onrobot_rg2/                  — RG2 gripper STL meshes (visual + collision)
      URDF/ur_description/               — UR3e meshes (DAE visual + STL collision)
  dashboard/           — Python/PyQt5 e-stop & debug dashboard (streamed to Steam Deck)
    main.py            — PyQt5 UI (status bar, tabbed screens, e-stop column)
    ros_interface.py   — rclpy node (velocity publisher, joint subscriber, controller switching)
  gripper_node.py      — OBSOLETE: old custom Modbus gripper node (replaced by ur_onrobot driver)
  beacon.py            — UDP multicast beacon broadcasting laptop IP for Quest auto-discovery
  launch.py            — Python launcher: starts ur_onrobot driver, controller switch, TCP endpoint, beacon
  launch.sh            — Shell wrapper for launch.py (sources ROS automatically)
  dashboard.sh         — Shell wrapper to run dashboard with ROS sourced
  urdf/                — OnRobot RG2 xacro source files (from UOsaka-Harada-Laboratory/onrobot)
  meshes/              — OnRobot RG2 STL meshes (rg2/visual + rg2/collision)
  ROS-TCP-Connector/  — Unity package (local, referenced via file:// in manifest.json)
  URDF-Importer/      — Unity package (local, referenced via file:// in manifest.json)
  evaluation/          — Individual subsystem evaluation criteria (P/C/D/HD for each member)
  PROJECT_PLAN.md    — Full project plan (all subsystems, architecture, milestones)
  RS2 Project Contract Renegotiated.pdf — Official project contract with evaluation criteria
  SETUP.md            — Onboarding guide for new contributors
  subsystem3_renegotiation.md  — Nic's evaluation criteria renegotiation
  subsystem2_renegotiation_options.md — Options for Oliver's subsystem renegotiation
```

## Development Environment

**Prerequisites:** ROS 2 Humble on Ubuntu 22.04, Unity 6.3 LTS.

**Do not use Docker** — workspace is built from source using `UR_OnRobot_ROS2` (combined UR + OnRobot driver). All dependencies already in `ros2_ws/src/`.

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

# Terminal 1 — UR + OnRobot combined driver
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194 launch_rviz:=true

# Terminal 2 — ROS-TCP bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 3 — switch to velocity + gripper controllers
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers --activate forward_velocity_controller finger_width_controller --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller
```

Then on the teach pendant: load and run External Control program (Host IP: `192.168.0.100`).
Then hit Play in Unity.

**Note:** Both RMRC and Direct Joint modes publish to `/forward_velocity_controller/commands`. Gripper publishes to `/finger_width_controller/commands` (Float64MultiArray, position in metres). Both controllers must be active — check with `ros2 control list_controllers`. MoveIt Servo is no longer used for teleoperation. MoveIt 2 will be used by Oliver's autonomous sorting subsystem.

### Simulated Hardware (no real robot)

```bash
# Terminal 1 — fake UR3e + RG2 hardware
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=true

# Terminal 2 — switch to velocity + gripper controllers (wait for Terminal 1 to finish loading)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers --activate forward_velocity_controller finger_width_controller --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller

# Terminal 3 — ROS-TCP bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

No teach pendant needed. Joint states and controllers work fine.

### Network Setup

Both laptop and Quest 3 connect to the UR3e's own router — all devices on the same `192.168.0.x` subnet.

```
Robot's Router (192.168.0.x):
  ├─ UR3e robot:  192.168.0.194
  ├─ Laptop WiFi: 192.168.0.102  ←──→ Quest 3
  └─ Laptop Ethernet: 192.168.0.100 ←──→ UR3e (direct)
```

- **Teach pendant External Control host IP**: `192.168.0.100` (laptop Ethernet)
- **Unity ROS Settings IP**: `192.168.0.102` (laptop WiFi on robot's router)
- **ros_tcp_endpoint**: `ROS_IP:=0.0.0.0` (listens on all interfaces)
- No more university WiFi issues — dedicated robot router provides stable, low-latency connection

## System Architecture

```
Intel RealSense (Depth Camera)
  └─ object detection, colour/CV classification, pose tracking
       ↓
ROS 2 Network
  ├─ Object poses broadcast (robot frame, via QR code calibration)
  ├─ ros_tcp_endpoint → Unity (XR overlays + teleop)
  └─ MoveIt 2 (autonomous trajectory planning — Oliver's subsystem)
       ↓
Two Operating Modes:
  ├─ TELEOP (Nic): Quest 3 controllers → RMRC/Hand Guide → velocity commands → UR3e + gripper
  └─ AUTONOMOUS (Oliver): MoveIt 2 → planned trajectories → UR3e + gripper
       ↓
UR3e + OnRobot Gripper
  └─ picks objects, places in bins
       ↓
Depth Camera verifies correct bin placement
```

**Teleoperation modes (Nic's subsystem):**
- **RMRC**: Jacobian-based Cartesian velocity control (translate + rotate sub-modes)
- **Direct Joint**: Individual joint jogging
- **Hand Guide**: Controller position tracking with Jacobian IK
- **Gripper**: Analog control via right index trigger (0–100%), publishes to `/finger_width_controller/commands` (metres, via `ur_onrobot` driver)
- **EE Lock-Down**: Y button toggle — locks tool pointing straight down via angular Jacobian correction
- **Collision Protection**: Table + self-collision prevention via predictive FK link position checks

## Unity Project

- **Path:** `Unity/My project/`
- **Version:** Unity 6.3 LTS (6000.3.9f1)
- **Build target:** Android (Meta Quest 3)
- **Scene:** `Assets/Scenes/SampleScene.unity`
- **ROS IP:** Set to laptop's WiFi IP for Quest builds (e.g., `172.19.115.245`), Port: 10000 (configured in Robotics → ROS Settings). Use `0.0.0.0` for ros_tcp_endpoint so it listens on all interfaces.

### OnRobot RG2 Gripper (URDF)

The combined `ur3e_rg2.urdf` adds the RG2 gripper to the UR3e, attached to `tool0` via a fixed joint. Gripper structure:

- `onrobot_rg2_base_link` — gripper body (fixed to tool0)
- `finger_joint` — **primary actuated joint** (revolute, left outer knuckle, limits: -0.558 to 0.785 rad)
- 5 mimic joints mirror `finger_joint`: `left_inner_knuckle_joint`, `left_inner_finger_joint`, `right_outer_knuckle_joint`, `right_inner_knuckle_joint`, `right_inner_finger_joint`
- Only `finger_joint` needs to be driven — all other gripper joints follow via mimic

Mesh source: [UOsaka-Harada-Laboratory/onrobot](https://github.com/UOsaka-Harada-Laboratory/onrobot) (`onrobot_rg_description`). Raw xacro files stored in `nic/urdf/`, meshes in `nic/meshes/rg2/`.

### Key Scripts

- `Assets/Scripts/JointStateSubscriber.cs` — attach to root `ur` GameObject; subscribes to `/joint_states` and sets joint rotations directly via `Transform.localRotation` (no ArticulationBody physics). Disables all ArticulationBody components on Start. Contains `rosToUnity` name mapping dictionary. Also animates gripper joints (finger_joint + 5 mimic joints) from `finger_width` in `/joint_states` (metres, from `ur_onrobot` driver) — maps width to finger_joint angle internally.
- `Assets/Scripts/RobotBasePlacer.cs` — attach to an **empty parent wrapper** (e.g. `RobotRig`) that contains `ur` (and optionally the trolley) as children. Uses XR controller grip button — hold grip near rig to grab and drag. Supports Y-axis rotation while grabbed (keeps robot upright). Pure transform movement, no physics.
- `Assets/Scripts/UR3eKinematics.cs` — static utility class. Forward kinematics using standard DH parameters, numerical Jacobian (3x6 linear, finite differences on FK), geometric angular Jacobian (3x6, joint z-axes). DLS pseudoinverse for both linear and angular velocity resolution. Manipulability measure for adaptive damping. `GetLinkPositions()` returns all 7 link origins (used for collision protection). `GetToolZAxis()` returns EE Z-axis direction (used for EE lock-down).
- `Assets/Scripts/SpatialMarkers.cs` — attach to robot root GameObject (must contain `tool0` in hierarchy). RGB axis cylinders on end-effector + yellow velocity arrow showing commanded Cartesian velocity direction/magnitude.
- `Assets/Scripts/RobotController.cs` — attach to any GameObject; uses **Unity Input System** (not OVRInput). Requires `RobotControlActions.inputactions` dragged into the Inspector's **Input Actions** field. Three control modes cycled via Menu button (RMRC → Direct Joint → Hand Guide → RMRC):
  - **RMRC mode** (default): Resolved Motion Rate Control with two sub-modes toggled by X button (left controller):
    - **Translate** (default): right stick = XY translation (forward/back, left/right), left stick Y = Z translation (up/down), left stick X = yaw. Uses 3x6 linear Jacobian.
    - **Rotate**: directly drives wrist joints — right stick Y = wrist_1 (pitch/tilt), right stick X = wrist_2 (roll), left stick X = wrist_3 (yaw/spin). Uses `jointJogSpeed` for responsive 1:1 control.
    - Both use `UR3eKinematics` DLS pseudoinverse. Adaptive damping near singularities. Proportional joint velocity scaling for safety. Publishes `Float64MultiArray` to `/forward_velocity_controller/commands`. No MoveIt needed.
  - **Direct Joint mode**: A/B to cycle joints, right stick Y for smooth proportional velocity control of selected joint. Publishes `Float64MultiArray` to `/forward_velocity_controller/commands`.
  - **Hand Guide mode**: Hold right grip trigger to engage — controller 3D position is tracked and mapped to end-effector position via proportional control + Jacobian IK. Release grip to stop (EE lock-down still applies if active). Requires `robotBase` field set to the `ur` GameObject in Inspector. Tunable `positionGain` (default 2) and `maxTrackingSpeed` (default 0.15 m/s). Joint bias weights (`[0.5, 0.5, 0.7, 1.0, 1.0, 1.0]`) softly favour wrist movement over base/shoulder.
  - **Gripper**: Right index trigger (analog 0–1) controls gripper width. Dead zone (0.05) so resting finger = fully open. Smoothed, publishes `Float64MultiArray` (position in metres, 0=closed, 0.11=open) to `/finger_width_controller/commands` at 50Hz. Works in all modes simultaneously.
  - **EE Lock-Down**: Y button (left controller) toggles. When active, angular Jacobian drives tool Z-axis to [0,0,-1] (pointing straight down). Works immediately on toggle — no grip required. 2° dead zone prevents jitter when nearly aligned. Gain 1.5, max 1.0 rad/s, 2× damping on angular solve. Works in RMRC Translate and Hand Guide modes (including when grip is released).
  - **Collision Protection**: Delegated to `MeshCollisionGuard.cs`. Assign the guard in Inspector (`collisionGuard` field). Runs after all velocity computation, before publishing — scales or zeros `qDot` based on `collisionGuard.ComputeVelocityScale()`.
  - **Output Smoothing**: All joint velocities pass through an exponential low-pass filter (`outputSmoothTime` default 0.06s) before publishing. Prevents jitter in lock-down mode and smooths Hand Guide/RMRC output. Resets on mode switch to avoid sluggish transitions.
  - On Start, automatically disables conflicting XRI Default Input Actions (locomotion maps, Jump, Rotate Manipulation, UI Scroll) so thumbsticks and A/B buttons are not consumed by teleport/turn/move.
  - Publish rate: 50Hz.
- `Assets/Scripts/MeshCollisionGuard.cs` — attach to `ur3e_rg2` GameObject; set `robotBase` to itself. Auto-discovers all MeshColliders in robot hierarchy at Start, sets them convex for `ClosestPoint` queries. Groups colliders into proximal (base/shoulder/upper_arm) and distal (wrist through gripper). Table collision uses bounding box min Y vs table height. Self-collision uses `ClosestPoint` distance between proximal and distal groups. Both have soft-zone gradual slowdown + hard stop. Gizmo drawing in Scene view. `CollisionDebugVisualizer.cs` shows on-screen collision status label.
- `Assets/Scripts/RobotControlActions.inputactions` — Unity Input System action asset with bindings: LeftStick, RightStick, NextJoint (A), PrevJoint (B), ToggleMode (Menu). Deadzone handled via StickDeadzone processor.
- `Assets/Scripts/ROSAutoConnect.cs` — attach to any GameObject; auto-discovers `ros_tcp_endpoint` by TCP-scanning `192.168.0.101`–`109` on port 10000 (skips `.100` which is the Ethernet-to-robot interface). Sets `ConnectOnStart = false` in Awake, then scans from a background thread with 300ms connect timeout per IP. Retries up to 20 rounds (500ms between rounds). Falls back to Unity ROS Settings IP if nothing found. No beacon/multicast needed — works reliably on Quest/Android.
- `Assets/Scripts/HeadsetStreamPublisher.cs` — attach to an empty GameObject (e.g. `HeadsetStream`). Renders the Unity scene from the XR camera's perspective via a hidden secondary camera, JPEG-encodes it, and publishes `CompressedImage` to `/headset/image_compressed` at configurable FPS (default 15). Uses a skybox background so the dashboard sees the full scene.
- `Assets/Scripts/SessionLogger.cs` — attach to any GameObject; assign `RobotController` in Inspector (auto-finds via `FindObjectOfType` if not set). Publishes JSON session status to `/session/status` at 2Hz and discrete events to `/session/events`. On application quit, saves a JSON session log to `Application.persistentDataPath/SessionLogs/`.
- `Assets/Scripts/RobotHUD.cs` — attach to an empty GameObject; assign the `RobotController` reference in Inspector. Floating HUD panel tracks the camera with colour-coded mode title (amber=Joint, green=Translate, blue=Rotate, purple/red=Hand Guide), control hints for current mode, and readable joint names in Direct Joint mode.

### Controller Mapping (Quest 3)

| Input | Direct Joint Mode | RMRC Translate | RMRC Rotate | Hand Guide |
|---|---|---|---|---|
| **Menu button** (left) | → Hand Guide | → Direct Joint | → Direct Joint | → RMRC |
| **X button** (left) | — | Toggle to Rotate | Toggle to Translate | — |
| **Y button** (left) | Toggle EE lock-down | Toggle EE lock-down | Toggle EE lock-down | Toggle EE lock-down |
| **A button** (right) | Next joint | — | — | — |
| **B button** (right) | Previous joint | — | — | — |
| **Right grip** (right) | — | — | — | Hold to track hand |
| **Right trigger** (index) | Gripper 0–100% | Gripper 0–100% | Gripper 0–100% | Gripper 0–100% |
| **Right stick Y** | Jog selected joint | EE forward/back (X) | Wrist 1 (pitch/tilt) | — |
| **Right stick X** | — | EE left/right (Y) | Wrist 2 (roll) | — |
| **Left stick Y** | — | EE up/down (Z) | — | — |
| **Left stick X** | — | EE yaw | Wrist 3 (yaw/spin) | — |

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
3. **Follow-up:** Deactivates `forward_velocity_controller` + `finger_width_controller` via `ros2 control switch_controllers`
4. **Resume:** Hold yellow RESUME button for 5 seconds → reactivates `forward_velocity_controller` + `finger_width_controller` + deactivates `scaled_joint_trajectory_controller`
5. **Cooldown:** 1-second cooldown after resume prevents accidental re-trigger

### Screens (tabs)

| Tab | Status | Description |
|---|---|---|
| STATUS | Working | Joint positions/velocities, event log |
| HEADSET | Working | Quest 3 XR scene view (JPEG stream from `HeadsetStreamPublisher.cs`) |
| CAMERA | Placeholder | Depth camera ROS topics (will show RealSense feed) |
| STATS | Working | Session info bar (duration, mode, switches, e-stops, mode %) + rolling joint velocity graph (6 coloured lines, 30s) + topic health graph (3 topics, 60s) |
| LATENCY | Working | Live latency numbers (colour-coded) + rolling message age graph (30s) + command interval graph (30s) |
| SESSION | Working | Text overview: control mode, sub-mode, hand guide state, mode durations, e-stop count, connection status, all topic rates |

## Known Issues

- **UR driver segfault (exit code -11)** — apt-installed `ros-humble-ur-robot-driver` crashes on Ubuntu 22.04. Now using `UR_OnRobot_ROS2` which builds the driver from source as a dependency.
- **Robot falls in Unity on Play** — no longer an issue. JointStateSubscriber.cs disables all ArticulationBody components (purely kinematic).
- **Git HTTPS clone fails on this machine** — always use SSH (`git@github.com:...`).
- **Unity Package Manager can't find robotics packages** — clone locally, reference via `file://` in `Packages/manifest.json`.
- **ros_tcp_endpoint crashes on Unity disconnect/reconnect** — known race condition in v0.7.0 (`InvalidHandle` exception). Restart the endpoint after every Unity Play/Stop cycle. Command must be on one line: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`
- **`/joint_states` not arriving in Unity despite subscription registering** — caused by duplicate message class registration. Delete `Assets/RosMessages/` entirely; types are built into ROS-TCP-Connector. (Fixed — RosMessages deleted.)
- **ROS joint names don't match Unity GameObject names** — Fixed with a `rosToUnity` mapping dictionary in JointStateSubscriber.cs.
- **Quest 3 networking** — laptop connects to robot via Ethernet (`192.168.0.100` ↔ `192.168.0.194`), Quest connects via WiFi. Set ROS IP in Unity to laptop's WiFi IP. Teach pendant External Control host IP = `192.168.0.100`.
- **OVRInput does nothing on Quest 3 (MR template)** — the Mixed Reality template uses Unity Input System + XR Interaction Toolkit, not OVR. Fixed by rewriting RobotController.cs to use `InputAction` with `<XRController>` bindings.
- **XRI Default Input Actions consume thumbsticks/buttons** — RobotController.cs disables conflicting action maps at runtime.
- **MoveIt Servo not used for teleoperation** — RMRC replaces it. MoveIt 2 will be used by Oliver's autonomous sorting subsystem.
- **forward_velocity_controller + finger_width_controller must be active** — RMRC and Direct Joint modes publish to `/forward_velocity_controller/commands`, gripper publishes to `/finger_width_controller/commands`. Switch controllers after launching the `ur_onrobot` driver.
- **RobotHUD may not render on Quest 3** — `Camera.main` can return null in the MR template, `Shader.Find` may return null on Android (shader stripping), runtime TextMeshPro needs an explicit font asset. Partially patched, functional enough.
- **URDF-Importer visual/collision STL name collision** — `LocateAssetHandler.cs:31` uses `Path.GetFileNameWithoutExtension` to check `UsedTemplateFiles`, so if visual and collision meshes share the same filename (e.g., both `base_link.stl`), the visual prefab creation is skipped. Fixed by renaming visual STLs to `*_visual.stl`.
