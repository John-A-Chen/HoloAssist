# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR teleoperation framework integrating a Meta Quest headset with a Universal Robots UR3e collaborative arm. This branch (`seb`) is one of several personal branches in a shared research team repository (other branches: `john`, `nic`, `ollie`).

The current focus is **XR teleoperation**: Quest controllers drive the UR3e end-effector via velocity commands. The `seb` branch extends Nic's base with visualization features, environment objects, and Quest 2 deployment.

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
- ✅ RobotController tested on Quest 2 — Direct Joint mode working with velocity controller
- ✅ RobotControlActions wired up in Inspector (already done by Nic in scene)
- ✅ ROSObjectPublisher.cs — publishes any GameObject's pose to ROS as TF frame + visualization marker (used for interactive cube)
- ✅ RobotDataPanel.cs — world-space XR panel showing robot status, live joint angles (rad→deg), end-effector pose relative to base_link, connection status, TF axes toggle state
- ✅ JointTFVisualizer.cs — draws XYZ axes on all robot joint links + auto-discovers non-static environment objects, toggleable via X button
- ✅ RadialMenu.cs — radial options menu hovering above left controller (Y button toggle), styled to match data panel, right controller ray + trigger to select buttons
- ✅ BinDetector.cs — trigger collider system detecting when objects are placed inside bins
- ✅ BinStatusPanel.cs — separate floating XR panel showing bin occupancy status
- ✅ Quest 2 deployment working — APK builds and installs, full pipeline tested (Quest → ROS → RViz)
- ✅ Velocity controller mode — `forward_velocity_controller` must be activated after UR driver launch (replaces `scaled_joint_trajectory_controller`)
- ✅ End-to-end teleoperation verified with fake hardware — joystick moves robot, RViz shows movement in sync
- ✅ Environment models added (Object_Bin FBX files in Assets/Environment_Models/)
- ✅ BinDetector working — drop cube in bin, BinStatusPanel updates count
- ✅ RadialMenu fully functional — TF Axes / Data Panel / Bin Status / Coach toggles, text visible (font renderQueue fix)
- ✅ CoachingPanel created — 6-page tutorial styled to match other panels
- ✅ PanelPlacer working — panels grabbable via XRGrabInteractable (right trigger ray)
- ✅ Selected joint highlighted in RobotDataPanel (gold + `>` prefix), replaces standalone RobotHUD function
- ✅ PassthroughToggle.cs — switches between MR (passthrough) and VR (skybox + virtual environment), wired into radial menu as "Passthru" button
- ✅ VR mode setup — VRSkybox material (Skybox/Procedural shader) + VirtualEnvironment GameObject containing ground plane
- ✅ CoachingPanel page navigation — clickable `<` `>` buttons at bottom, ray-selected with right trigger
- ⬜ Test MoveIt Servo mode end-to-end
- ⬜ Test full teleoperation loop with real robot
- ⬜ Make ground plane solid (currently objects fall through — needs collider verification + correct physics layers)
- ⬜ Fix trolley pink material — assign URP Lit material to UR3eTrolley meshes
- ⬜ Trolley parts separating during play — likely a child has its own Rigidbody; investigate hierarchy

## Repository Layout

```
seb/
  ros2_ws/          — ROS 2 workspace
    src/
      ROS-TCP-Endpoint/              — Unity-ROS TCP bridge (cloned from source)
      Universal_Robots_ROS2_Driver/  — UR driver (cloned from source, fixes apt segfault)
      holoassist_manager/            — tf_marker_bridge node (republishes Unity markers)
  Unity/My project/ — Unity 6.3 project
    Assets/
      Scripts/JointStateSubscriber.cs  — subscribes to /joint_states, drives robot joints
      Scripts/RobotController.cs       — robot control (Direct Joint + MoveIt Servo modes), TF toggle (X button)
      Scripts/RobotBasePlacer.cs       — hand-tracking pinch-to-grab robot placement
      Scripts/RobotHUD.cs              — floating HUD showing mode + selected joint
      Scripts/RobotDataPanel.cs        — world-space panel showing joint angles, EE pose, connection status
      Scripts/ROSObjectPublisher.cs    — publishes GameObject pose to ROS as TF + marker
      Scripts/JointTFVisualizer.cs     — draws TF axes on robot joints + auto-discovers moving objects
      Scripts/RadialMenu.cs            — radial options menu above left controller (4 toggle buttons)
      Scripts/BinDetector.cs           — trigger-based bin occupancy detection (auto-sized child trigger)
      Scripts/BinStatusPanel.cs        — floating panel showing bin status (auto-discovers BinDetectors)
      Scripts/CoachingPanel.cs         — 6-page tutorial panel with clickable `<`/`>` page nav
      Scripts/PanelPlacer.cs           — XRGrabInteractable wrapper for movable panels
      Scripts/PassthroughToggle.cs     — toggles MR passthrough vs VR (skybox + virtual env)
      Scripts/RobotControlActions.inputactions — Unity Input System bindings for robot control
      Environment_Models/              — FBX models (bins, etc.)
      URDF/                            — ur3e.urdf + meshes
  ROS-TCP-Connector/  — Unity package (local, referenced via file:// in manifest.json)
  URDF-Importer/      — Unity package (local, referenced via file:// in manifest.json)
  holoassist.rviz     — RViz config with TF, Marker, RobotModel displays
  QUICKSTART.md       — copy-paste terminal commands for testing
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

See [QUICKSTART.md](QUICKSTART.md) for copy-paste commands.

```bash
# Check laptop IPs first (Ethernet for robot, WiFi for Quest)
hostname -I

# Terminal 1 — UR driver
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=false

# Terminal 2 — Unity-ROS bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 3 — TF marker bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run holoassist_manager tf_marker_bridge

# Terminal 4 — Activate velocity controller (REQUIRED — run after Terminal 1 is ready)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 2}"

# Terminal 5 — RViz
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
rviz2 -d holoassist.rviz

# Terminal 6 (only for MoveIt Servo mode)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false launch_servo:=true
```

Then on the teach pendant: load and run External Control program (Host IP: `192.168.0.100`).
Then deploy APK to Quest and launch app.

**Important:** The velocity controller switch (Terminal 4) must be run after the UR driver is ready. It resets every time Terminal 1 is restarted. Without it, the joystick won't move the robot.

**For fake hardware testing**, replace Terminal 1 with:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=xxx use_fake_hardware:=true launch_rviz:=false
```

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
- **Build target:** Android (Meta Quest 2/3)
- **Scene:** `Assets/Scenes/SampleScene.unity`
- **ROS IP:** Set to laptop's WiFi IP for Quest builds (e.g., `172.19.115.245`), Port: 10000 (configured in Robotics → ROS Settings). Use `0.0.0.0` for ros_tcp_endpoint so it listens on all interfaces.

### Key scripts

- `Assets/Scripts/JointStateSubscriber.cs` — attach to root `ur` GameObject; subscribes to `/joint_states` and sets joint rotations directly via `Transform.localRotation` (no ArticulationBody physics). Disables all ArticulationBody components on Start. Contains `rosToUnity` name mapping dictionary.
- `Assets/Scripts/RobotBasePlacer.cs` — attach to an **empty parent wrapper** (e.g. `RobotRig`) that contains `ur` (and optionally the trolley) as children. Uses XR hand tracking — pinch to grab, drag to move. Pure transform movement, no physics.
- `Assets/Scripts/RobotController.cs` — attach to any GameObject; uses **Unity Input System** (not OVRInput). Requires `RobotControlActions.inputactions` dragged into the Inspector's **Input Actions** field. Also has `tfVisualizer` field for TF axes toggle. Two control modes switched via Menu button:
  - **Direct Joint mode** (default): A/B to cycle joints, right stick Y for smooth proportional velocity control of selected joint. Publishes velocity commands to `/forward_velocity_controller/commands` (requires velocity controller to be activated). Publishes continuously (even at zero input) so the robot holds position.
  - **MoveIt Servo mode**: left stick = XY translation, right stick = Z translation + yaw. Publishes `TwistStamped` to `/servo_node/delta_twist_cmds`. **Requires MoveIt Servo running**. Sends commands continuously (including zeros) to prevent servo 100ms idle timeout.
  - **X button** (left controller): toggles TF axes on robot joints and environment objects via JointTFVisualizer.
  - On Start, automatically disables conflicting XRI Default Input Actions (locomotion maps, Jump, Rotate Manipulation, UI Scroll) so thumbsticks and A/B buttons are not consumed by teleport/turn/move.
  - Exposes `CurrentPositions` and `JointNames` for RobotDataPanel to read live joint data.
- `Assets/Scripts/RobotControlActions.inputactions` — Unity Input System action asset with bindings: LeftStick, RightStick, NextJoint (A), PrevJoint (B), ToggleMode (Menu). Deadzone handled via StickDeadzone processor.
- `Assets/Scripts/ROSObjectPublisher.cs` — attach to any GameObject to publish its pose to ROS. Publishes TF frame to `/tf` and visualization marker to `/unity_markers`. Creates toggleable XYZ axes on the object. Pose is computed relative to `robotBase` (auto-finds `base_link`). Coordinate conversion: Unity (X-right, Y-up, Z-forward) → ROS (X-forward, Y-left, Z-up).
- `Assets/Scripts/RobotDataPanel.cs` — world-space floating XR panel. Auto-reads joint angles from RobotController, end-effector pose from tool0 relative to base_link, connection status, and TF toggle state. **Highlights selected joint in gold with `>` prefix when in Direct Joint mode** (replaces standalone RobotHUD functionality). Fields: `robotController`, `endEffectorTransform` (tool0), `robotBase` (base_link), `tfVisualizer` (ur3e_robot). Stores both label and value TextMeshPro refs in `valueTexts` dict (key `joint_N` and `joint_N_label`).
- `Assets/Scripts/JointTFVisualizer.cs` — attach to robot root (`ur3e_robot`). Draws RGB XYZ axes on all joint links. Auto-discovers non-static environment objects and adds axes to them. `linkedPublishers` array toggles ROSObjectPublisher axes too (e.g. cube). Toggle via `SetVisible()`/`Toggle()`.
- `Assets/Scripts/RadialMenu.cs` — radial options menu hovering above left controller. Y button toggles open/close. Right controller ray + trigger selects buttons. Styled to match RobotDataPanel. Currently has TF Axes toggle button. `RegisterButton()` API for adding buttons at runtime.
- `Assets/Scripts/BinDetector.cs` — attach to bin GameObjects. Auto-creates a child `BinTrigger` GameObject sized to the bin's mesh bounds (×1.2 multiplier by default), with kinematic Rigidbody. Trigger events relayed via `BinTriggerRelay` since bin's own collider may not be a trigger. Reports to BinStatusPanel. Auto-finds BinStatusPanel via `FindFirstObjectByType` if not assigned. Has Y offset for trigger zone (raise above bin floor).
- `Assets/Scripts/BinStatusPanel.cs` — separate floating XR panel showing bin occupancy. Auto-discovers all BinDetector components on Start. Shows "Empty" (grey) or "N object(s)" (green) per bin.
- `Assets/Scripts/CoachingPanel.cs` — multi-page tutorial panel styled to match RobotDataPanel. 6 pages (Getting Started, Direct Joint, Servo, Visualization, Interaction, Options). Toggleable via radial menu.
- `Assets/Scripts/PanelPlacer.cs` — makes any panel grabbable using `XRGrabInteractable`. Auto-adds BoxCollider + Rigidbody (kinematic, frozen). On grab: disables follow-camera. On release: freezes Rigidbody constraints to lock position. Sized via `colliderWidth/Height/Depth` fields.
- `Assets/Scripts/PassthroughToggle.cs` — flips between MR and VR. MR mode: camera clearFlags=SolidColor, alpha 0 background → reveals Quest passthrough. VR mode: clearFlags=Skybox with assigned material + virtualEnvironment GameObject enabled (e.g. ground plane). Wired into RadialMenu as "Passthru" button.

### Controller mapping (Quest 2/3)

| Input | Direct Joint Mode | MoveIt Servo Mode |
|---|---|---|
| **Menu button** (left) | Toggle to Servo | Toggle to Direct Joint |
| **A button** (right) | Next joint | — |
| **B button** (right) | Previous joint | — |
| **X button** (left) | Toggle TF axes | Toggle TF axes |
| **Y button** (left) | Toggle radial menu | Toggle radial menu |
| **Right stick Y** | Jog selected joint (proportional velocity) | End-effector up/down (Z) |
| **Right stick X** | — | End-effector yaw |
| **Left stick Y** | — | End-effector forward/back (X) |
| **Left stick X** | — | End-effector left/right (Y) |
| **Right trigger** | — (XR grab) | — (radial menu select when open) |

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
- **Unity 6 SurfaceFlinger bug** — `Build And Run` fails with "Unable to query OpenGL version" on Quest. Workaround: do `File → Build` first, then `Build And Run`. Or build then `adb install -r`.
- **XR Simulation asset conflict** — `Assets/XR/Temp/` contains duplicate assets that clash with `Assets/XR/Resources/`. Delete `Assets/XR/Temp/` before each build (Unity recreates it).
- **URDF-Importer assimp.dll conflict** — win/x86 and win/x86_64 `assimp.dll` plugins enabled for "Any" platform, causing Android build errors. Fixed: set `Any: enabled: 0` in both `.meta` files.
- **Active Input Handling "Both"** — unsupported on Android. Changed `ProjectSettings.asset` `activeInputHandler` from `2` to `1` (Input System Package only).
- **Velocity controller not active by default** — UR driver starts with `scaled_joint_trajectory_controller` active, but `RobotController.cs` publishes to `/forward_velocity_controller/commands`. Must switch controllers after launch (see Terminal 4 in launch sequence). Resets on every UR driver restart.
- **ROS IP for Quest builds** — must be laptop's WiFi IP (check `hostname -I`), not `127.0.0.1`. Quest is a separate device on WiFi.
- **PPtr cast failed build error** — corrupted script cache. Fix: `Assets → Reimport All` in Unity, then rebuild.
- **FBX models appear pink/magenta** — missing URP material. Create a URP Lit material and assign it.
- **Git LFS** — `.dae`, `.apk`, and `.fbx` files tracked via LFS. Run `git lfs track "*.fbx"` before adding FBX files.
- **TextMeshPro renderQueue ordering** — TMP's default font material uses renderQueue 3000, same as transparent quads. With `ZWrite=0` on quads, text can render BEHIND quads unpredictably. Fix: clone the font material and set `renderQueue = 3500` so text always renders on top. Applied in RadialMenu.cs (button labels, status, title).
- **Panel readability when tilted/moved** — Z-fighting between background quads, separators, and text when panel is at oblique angles. Fix: differentiate render queues — Background uses 2950, separators/accents use 3000, text uses 3500. Background renders first, then accents, then text always on top.
- **TMP fontSize in 3D mode** — fontSize is in TMP font units, NOT world meters. RobotDataPanel uses fontSize 0.25 in a 0.385m-wide rect; for smaller rects, scale fontSize proportionally. For radial menu buttons (~0.1m rects), fontSize 0.08 with explicit `sizeDelta` works. Auto-sizing didn't work reliably in builds.
- **PanelPlacer + XRGrabInteractable** — uses `MovementType.Instantaneous`, `throwOnDetach=false`, kinematic Rigidbody with `FreezeAll` constraints when not grabbed. Constraints are temporarily released during grab so panel can move. Disables panel's `MonoBehaviour` (RobotDataPanel/BinStatusPanel/CoachingPanel) on grab to stop camera-follow logic from fighting the user's hand.
- **BinDetector trigger zone** — bins have a Mesh/Box Collider for physics already. Adding a trigger to the same GameObject conflicts. Fix: BinDetector creates a separate `BinTrigger` child GameObject with its own kinematic Rigidbody and BoxCollider sized to the bin's mesh bounds. Trigger events on the child are relayed back via `BinTriggerRelay` component to the parent's `BinDetector.OnChildTriggerEnter/Exit`.
- **Quest 3 vs Quest 2 install conflict** — `INSTALL_FAILED_UPDATE_INCOMPATIBLE` when installing on a different headset that has a previously-installed APK with a different signing key. Fix: click "Yes" in Unity's prompt to remove the old install. Not a Quest 2/3 incompatibility.
- **Manifest.json paths from other branches** — Nic's branch has hardcoded paths like `file:/home/nic/git/...` in `Packages/manifest.json`. When cloning Nic's branch, copy ROS-TCP-Connector and URDF-Importer locally and update manifest paths to your machine.
- **Passthrough vs VR mode** — Camera clear flags control passthrough on Meta Quest with the MR template:
  - **MR mode**: `clearFlags = SolidColor`, `backgroundColor.a = 0` (alpha 0 reveals passthrough)
  - **VR mode**: `clearFlags = Skybox` (uses RenderSettings.skybox material) OR `SolidColor` with opaque color
  - PassthroughToggle.cs handles both cases. Original camera state is saved on Start so toggling restores correctly.
- **Trolley pink/magenta** — UR3eTrolley FBX imports without materials assigned. Fix: create a URP Lit material, assign to mesh renderers manually. The trolley has multiple sub-meshes — assign material to all of them.
- **Trolley parts separating during play** — the trolley GameObject hierarchy may have child rigidbodies or be affected by gravity from VR mode. Investigate `RobotRig → UR3eTrolley(1)` children — typically should be fully kinematic (no Rigidbody on parts).
- **Ground plane physics** — Unity's Plane primitive has a MeshCollider by default. If objects fall through, verify: (1) the plane has a Collider component enabled, (2) falling objects have non-trigger colliders + Rigidbody with gravity, (3) physics layers are not in conflict in `Edit → Project Settings → Physics → Layer Collision Matrix`.
- **CoachingPanel page nav buttons** — uses world-space raycast from right controller forward direction onto panel plane. If clicks don't register: (1) verify `rightControllerOverride` field is assigned, (2) button hit radius scales with panel size; for very small panels increase the radius multiplier.
