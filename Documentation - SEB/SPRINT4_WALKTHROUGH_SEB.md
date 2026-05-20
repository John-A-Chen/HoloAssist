# Subsystem 4 — XR Scene & Visualisation

**Owner:** Sebastian  
**Sprint 4 Technical Documentation — subsystem section**

---

## Purpose

This subsystem is the operator's **window onto the HoloAssist system**. It runs inside the Meta Quest 3 headset and gives the human operator everything they need to see and control without removing the headset:

- A **Robot Data Panel** showing live joint angles, end-effector pose, control mode, gripper percentage, and EE-lock state.
- A **Bin Status Panel** showing real-time sorting progress per bin.
- A **Radial Menu** (Y button) for toggling UI elements and switching robot control modes.
- **Bin detection** — Unity physics trigger zones that update the Bin Status Panel as objects enter or leave each bin.
- **Toggleable TF axes** at every robot joint and on every non-static scene object.
- A **ROS object publisher** broadcasting the pose of an interactive Unity GameObject to `/tf` and `/unity_markers`, so it appears in RViz alongside the real robot.
- A portal test-chamber **3D environment** providing spatial context around the robot.
- An **Editor setup menu** (`GameObject → HoloAssist → Setup All UI Features`) that wires the full scene hierarchy in one click.

---

## Role in the Overall System

HoloAssist has four subsystems: **Perception** (John), **Autonomous Sorting** (Oliver), **Teleoperation & Interaction** (Nic), and **XR Scene & Visualisation** (this one). All four share a single ROS 2 graph.

This subsystem does **not** drive the robot or detect real objects with the depth camera — those are Nic's and John's responsibilities. It **consumes** robot state from `/joint_states` and **produces** an interactive scene for the operator. Its only outbound ROS contribution is the `/tf` + `/unity_markers` pose of an interactive Unity object that the perception subsystem can later overwrite when it has its own detected-object pose ready.

---

## Key Files, Components & Interfaces

Files grouped by role. For each script I list the GameObject it attaches to, what it reads/writes, and which methods other components call on it.

### Display components (operator-visible UI)

**`Assets/Scripts/RobotDataPanel.cs`**
- **Attached to:** dedicated empty GameObject `RobotDataPanel` in the scene. Builds its quad + TextMeshPro children procedurally on `OnEnable`.
- **Reads from:** `JointStateSubscriber.LastJointAnglesRad[6]`, `JointStateSubscriber.HasReceivedJointState`, gripper `finger_width` (cached). Optional: `RobotController` (mode + EE-lock state), `OVRManager.instance.isInsightPassthroughEnabled` (passthrough state).
- **Public interface:** `ForceRefresh()` — called by `RadialMenu` to push toggle changes immediately, without waiting for the next `LateUpdate`.
- **Marked `[ExecuteInEditMode]`** so the panel renders in the Editor without entering Play.

**`Assets/Scripts/BinStatusPanel.cs`**
- **Attached to:** dedicated empty GameObject `BinStatusPanel`.
- **Reads from:** auto-discovers all `BinDetector` components via `FindObjectsByType<BinDetector>()` on `Start`. One row per discovered bin.
- **Public interface:** `SetBinStatus(string binName, string status)` — called by every `BinDetector` whenever its trigger zone gains or loses an object.
- Row colour reflects state: grey (empty), green (occupied).

**`Assets/Scripts/RadialMenu.cs`**
- **Attached to:** dedicated empty GameObject `RadialMenu`. Builds the 2-page button ring procedurally above the left controller.
- **Inputs:** Y button (left controller) via Unity Input System (with K key as keyboard fallback). Right-controller forward ray for selection (plane-raycast — see *Implementation Notes §5*).
- **Inspector references it can toggle:** `RobotDataPanel`, `BinStatusPanel`, `JointTFVisualizer`, `PassthroughToggle`, `CoachingPanel`. Falls back to `FindObjectsByType<>()` if Inspector fields are null.
- **Mode-switching calls:** `RobotController.ToggleMode()`, `ToggleRMRCSubMode()`, `ToggleEELockDown()`, joint navigation (page 2).
- **Public interface:** `RegisterButton(label, initialState, onToggle)` for runtime button injection.

**`Assets/Scripts/CoachingPanel.cs`**
- **Attached to:** dedicated GameObject `CoachingPanel`. Lightweight floating overlay for operator instructions during demo runs.

### Detection components

**`Assets/Scripts/BinDetector.cs`** (one per bin)
- **Attached to:** the bin GameObject (typically the FBX root, e.g. `BinA`, `BinB`).
- **Inspector fields:** `binName`, `triggerSizeMultiplier` (1.2), `triggerYOffset` (0.05 m), `buildContainerColliders` (true), `wallThickness` (0.01 m), `wallInset` (0), `disableBlockingMeshColliders` (true), `detectLayers` (Everything), `statusPanel` (auto-found if blank).
- **Outputs:** writes to `BinStatusPanel.SetBinStatus(binName, status)` on every `OnTriggerEnter`/`Exit`.
- **Public properties:** `int ObjectCount`, `bool HasObjects`, `string Status`.
- **Auto-builds on `Start`:** child `BinTrigger` GameObject (BoxCollider trigger + kinematic Rigidbody) and child `BinContainerColliders` GameObject (5 BoxCollider walls — floor + 4 sides).
- **Helper class:** `BinTriggerRelay` (in the same file) — auto-added child component that forwards `OnTriggerEnter`/`Exit` events from the trigger child up to the parent `BinDetector`.

### Publishing components

**`Assets/Scripts/ROSObjectPublisher.cs`** (one per object to publish)
- **Attached to:** the GameObject whose pose to broadcast (e.g. the interactive Unity cube).
- **Inspector fields:** `frameName` (`"unity_cube"`), `parentFrame` (`"base_link"`), `robotBase` (Transform — auto-found by name if blank), `publishRate` (10 Hz), `publishMarker` (true), `markerColor`, `showPoseAxes`, `axisLength`, axis material slots (X/Y/Z).
- **Outputs:** publishes `tf2_msgs/TFMessage` to `/tf` and `visualization_msgs/Marker` to `/unity_markers` at `publishRate` Hz.
- **Public methods:** `CreatePoseAxes()`, `SetAxesVisible(bool)`, `ToggleAxes()` — used by `JointTFVisualizer` to delegate axis visibility globally.

### Visualisation toggles

**`Assets/Scripts/JointTFVisualizer.cs`**
- **Attached to:** the root robot GameObject `ur3e_rg2` (must contain links named `base_link`, `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`, `tool0`).
- **Inspector fields:** `axisLength` (0.08 m), `axisThickness` (0.003 m), `envAxisLength` (0.12 m), `envAxisThickness` (0.004 m), `xColor`/`yColor`/`zColor`, `linkedPublishers` (array of `ROSObjectPublisher`s to toggle together), `autoDiscoverMovingObjects` (true), `showAxes`.
- **Public methods:** `SetVisible(bool)`, `Toggle()`, `RefreshDiscovery()`, `GetDiscoveredObjects()`.
- **Behaviour on `Start`:** builds RGB cylinder axes at each named link. If `autoDiscoverMovingObjects` is true, also adds axes to every non-static `MeshRenderer` in the scene, skipping UI/system objects via a hard-coded skip list.

**`Assets/Scripts/PassthroughToggle.cs`**
- **Attached to:** dedicated GameObject `PassthroughToggle`. Wraps Meta XR's `OVRManager.instance.isInsightPassthroughEnabled` flag and the underlying camera's clear-flag setup.
- **Public methods:** `Toggle()`, public bool `PassthroughEnabled`.

### Editor tooling

**`Assets/Scripts/Editor/HoloAssistSetup.cs`**
- **Editor-only**, not part of the runtime build.
- Adds menu item `GameObject → HoloAssist → Setup All UI Features`.
- Creates the GameObjects for `RobotDataPanel`, `BinStatusPanel`, `RadialMenu`, `JointTFVisualizer`, `PassthroughToggle` and wires their Inspector references in one click.

### Scene assets (no scripts)

| Path | Contents |
|---|---|
| `Assets/PortalMesh/` | Portal-style test-chamber meshes, materials, textures. Provides spatial context around the robot. |
| `Assets/Environment_Models/` | Bin FBX models (sorting destinations) and Intel RealSense FBX (visual stand-in for the real depth camera). |
| `Assets/Scenes/SampleScene.unity` | Single scene containing all of the above — robot, bins, panels, radial menu, environment. |

### Inter-component interfaces (Unity-side)

Components in this subsystem talk to each other via plain method calls, not ROS. Documenting these explicitly because they're the contract between scripts:

| Caller | Callee | Method | When |
|---|---|---|---|
| `BinTriggerRelay` (child) | `BinDetector` (parent) | `OnChildTriggerEnter(Collider)`, `OnChildTriggerExit(Collider)` | Object enters/leaves bin trigger |
| `BinDetector` | `BinStatusPanel` | `SetBinStatus(string, string)` | Object count changes |
| `RadialMenu` | `RobotDataPanel` | `gameObject.SetActive(bool)`, `ForceRefresh()` | Data-panel toggle button pressed |
| `RadialMenu` | `BinStatusPanel` | `gameObject.SetActive(bool)` | Bin-status toggle button pressed |
| `RadialMenu` | `JointTFVisualizer` | `Toggle()` | TF-axes toggle button pressed |
| `RadialMenu` | `PassthroughToggle` | `Toggle()` | Passthrough toggle button pressed |
| `RadialMenu` | `RobotController` | `ToggleMode()`, `ToggleRMRCSubMode()`, `ToggleEELockDown()` | Page-2 mode buttons |
| `JointTFVisualizer` | `ROSObjectPublisher` (linked) | `SetAxesVisible(bool)` | TF axes globally toggled |
| `RobotDataPanel` | `JointStateSubscriber` | reads `LastJointAnglesRad[]`, `HasReceivedJointState`, `currentFingerWidth` | Every `LateUpdate` |

---

## ROS Topics / Interfaces

| Topic | Direction | Type | Rate | Publisher / Subscriber | Description |
|---|---|---|---|---|---|
| `/joint_states` | Subscribe | `sensor_msgs/JointState` | ~125 Hz (driver-determined) | Published by `ur_onrobot_control` (laptop). Subscribed by `JointStateSubscriber` (on `ur3e_rg2` GameObject — Nic's subsystem). | Joint angles (rad) for the 6 arm joints + `finger_width` (m) for the gripper. |
| `/tf` | Publish | `tf2_msgs/TFMessage` | 10 Hz (configurable) | `ROSObjectPublisher` (on the interactive cube GameObject). | Transform of the `unity_cube` frame parented to `base_link`. |
| `/unity_markers` | Publish | `visualization_msgs/Marker` | 10 Hz (configurable) | `ROSObjectPublisher`. | CUBE marker — namespace `unity_objects`, id `1`. Used by RViz. |

### `/joint_states` field usage

`JointStateSubscriber.OnJointState(JointStateMsg msg)` reads:

- `msg.name[i]` — looked up in a `rosToUnity` dictionary (e.g. `shoulder_pan_joint` → Unity `shoulder_link`).
- `msg.position[i]` — clamped to per-joint URDF limits, written into `LastJointAnglesRad[idx]`.
- `name == "finger_width"` — special-cased into `currentFingerWidth` (metres). Drives gripper-joint animation through 6 mimic joints (`finger_joint` + 5 mimics).

`HasReceivedJointState` flips to `true` after the first valid message. The data panel uses this to switch its status row from *"Waiting for ROS"* to *"Connected"*.

### `/tf` published fields

`ROSObjectPublisher.PublishTF()` populates:

- `header.frame_id` = `parentFrame` (default `"base_link"`)
- `header.stamp` — wall clock via `System.DateTimeOffset.UtcNow` (avoids RViz stale-message rejection)
- `child_frame_id` = `frameName` (default `"unity_cube"`)
- `transform.translation.{x,y,z}` — Unity-to-ROS coordinate-converted position relative to the robot base
- `transform.rotation.{x,y,z,w}` — corresponding rotation

### `/unity_markers` published fields

`ROSObjectPublisher.PublishMarker()` populates:

- `header.frame_id`, `header.stamp` — same as `/tf`
- `ns = "unity_objects"`, `id = 1`, `type = MarkerMsg.CUBE`, `action = MarkerMsg.ADD`
- `pose.position`, `pose.orientation` — same conversion as `/tf`
- `scale.{x,y,z}` — derived from `transform.lossyScale` (axis-swapped to ROS convention)
- `color.{r,g,b,a}` — from Inspector `markerColor`

### ROS Services / Actions

This subsystem does **not** expose any ROS services or actions, and it does not call any. All inter-process communication is over topics. Robot control commands (velocity, gripper) are published by Nic's teleoperation subsystem on different topics (`/forward_velocity_controller/commands`, `/finger_width_controller/commands`).

### Internal interfaces

See *Inter-component interfaces (Unity-side)* under [Key Files, Components & Interfaces](#key-files-components--interfaces) for the method calls between Unity scripts.

---

## Implementation Notes

These are the non-obvious mechanisms behind the components. They explain *why* the code is structured the way it is.

### 1. Why `RobotDataPanel` reads from `JointStateSubscriber`, not from `/joint_states` directly

The panel does not subscribe to ROS itself. Instead, `JointStateSubscriber` (which is part of Nic's teleoperation subsystem and already in the scene) caches the most recent angles in a public array `LastJointAnglesRad[6]`. The panel reads that array every frame in `LateUpdate()`.

**Reason:** if the panel subscribes during `Start()`, the `Subscribe<JointStateMsg>` call sometimes fires before `ROSConnection` has finished its TCP handshake with `ros_tcp_endpoint`. The first batch of messages is silently dropped. Polling a cached value avoids this entirely — the data is ready whenever the subscriber has it.

### 2. Self-healing subscription pattern

Components that *do* subscribe (e.g. the panel itself, when no central subscriber is present) wrap registration in a guarded helper:

```csharp
void SubscribeToJointStates() {
    if (rosSubscribed) return;          // idempotent
    ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", OnJointState);
    rosSubscribed = true;
}

void LateUpdate() {
    if (!rosSubscribed) SubscribeToJointStates();   // retry every frame until it sticks
    if (cam == null)   cam = FindXRCamera();        // same for camera
}
```

This handles the "Disable Domain Reload" Unity setting (skips `Start()` on Editor restart) and the case where `ROSConnection` isn't ready yet at scene load.

### 3. Procedural panel rendering — no Unity Canvas

Both panels are built from `GameObject.CreatePrimitive(PrimitiveType.Quad)` + child `TextMeshPro` components. `[ExecuteInEditMode]` lets them render in the Editor without entering Play. `OnEnable()` destroys and rebuilds children before recreating, so script recompiles do not produce duplicate quads.

A Unity Canvas would be simpler but introduces dependency on the EventSystem and CanvasScaler, which interact poorly with XR cameras. World-space quads + TMP keep the panels independent of UI plumbing.

### 4. Camera find with fallback chain

The MR template's eye-anchor camera is often *not* tagged `MainCamera`. `FindXRCamera()` tries:
1. `Camera.main`
2. Named search: `"Main Camera"`, `"CenterEyeAnchor"`, `"Camera"`, `"XR Camera"`
3. Any active `Camera` component in the scene

Camera follow uses `Vector3.Lerp` + `Quaternion.Slerp` in `LateUpdate()` (after the camera has moved that frame) to smooth motion at `followSpeed * Time.deltaTime`.

### 5. `RadialMenu` selection — plane raycast, no colliders

Each radial button is a quad with no `Collider` component. Selection works as follows:
- Project the right-controller forward ray onto the menu's local XY plane (`Plane.Raycast`).
- Compute the angle of the hit point relative to the menu centre.
- Find the button whose angular slice contains that angle.

This avoids registering button colliders with the XR Interaction Toolkit (which would conflict with grab/teleport interactions) and avoids physics-layer setup.

If the right controller transform is `null` at `Start()` (which happens when the XR rig spawns late), `RadialMenu` retries each frame and falls back to `FindObjectsByType<XRBaseController>` to locate it.

### 6. `BinDetector` trigger-zone construction

A bin's FBX mesh has its own `MeshCollider`. Convex MeshColliders fill the entire interior volume — objects dropped from above bounce off the "lid" instead of falling inside.

`BinDetector.Start()` runs three passes:
1. Compute world-space `Bounds` from all child `Renderer`s (excluding helpers).
2. Create a child `BinTrigger` GameObject with a `BoxCollider` (trigger) sized to `bounds × triggerSizeMultiplier`, plus a kinematic `Rigidbody` so trigger events fire.
3. If `disableBlockingMeshColliders == true`, walk all child `MeshCollider`s and disable any that are convex.
4. If `buildContainerColliders == true`, build 5 non-trigger `BoxCollider`s — floor + 4 walls — sized from the same bounds, so dropped objects rest inside the bin.

`OnTriggerEnter`/`Exit` are relayed from the trigger child up to the parent `BinDetector` via a `BinTriggerRelay` component, so all detection logic lives on the parent.

### 7. Unity → ROS coordinate conversion in `ROSObjectPublisher`

Unity is left-handed (X-right, Y-up, Z-forward). ROS is right-handed (X-forward, Y-left, Z-up).

```csharp
rx = uPos.z;   ry = -uPos.x;   rz = uPos.y;
qx = -uRot.z;  qy =  uRot.x;   qz = -uRot.y;   qw = uRot.w;
```

The pose is computed *relative to the robot base* (`base_link`), so the cube appears in RViz at the right place even if the operator drags the robot around in mixed reality. Timestamps use `System.DateTimeOffset.UtcNow` (wall clock) — Unity's simulation time is not synchronised with ROS time, and using it makes RViz reject messages as stale.

### 8. URP shader fallback for Android builds

Android shader stripping can remove `Universal Render Pipeline/Unlit` from the build. Every procedurally-created material uses a fallback chain:

```csharp
Shader.Find("Universal Render Pipeline/Unlit") ??
Shader.Find("Unlit/Color") ??
Shader.Find("Sprites/Default")
```

If panels still render pink/magenta on Quest, add the shader to `Project Settings → Graphics → Always Included Shaders`.

---

## Dependencies & Setup Requirements

### Hardware

| Item | Spec / Requirement |
|---|---|
| Meta Quest 3 headset | Standalone Android-based VR/MR headset. Must be in **developer mode** with **USB debugging enabled**. |
| Laptop | Ubuntu 22.04 LTS (native — not WSL). Recommended: 16+ GB RAM, modern GPU for Unity Editor performance. |
| Robot router | Provides a shared `192.168.0.x` subnet for the laptop WiFi and the Quest WiFi. (We use the UR3e's bundled router; any 2.4 / 5 GHz dual-band router works.) |
| USB-C cable | Quest-to-laptop, for initial APK deployment via `adb install`. After first deploy, OTA WiFi deploy via `adb connect <quest-ip>` is possible. |

### Quest 3 enablement (one-time, per Quest)

1. **Create a Meta developer organisation** at https://dashboard.oculus.com/. Free; required to enable developer mode.
2. **Pair the Quest** with the Meta Horizon mobile app (iOS/Android) under the same account.
3. In the mobile app: **Settings → Developer Mode → On**.
4. Plug the Quest into the laptop via USB-C. On the Quest, accept the **Allow USB Debugging** prompt.
5. Verify on the laptop: `adb devices` should list the Quest's serial number with status `device`.

### Software

| Component | Version | Verification | Notes |
|---|---|---|---|
| Unity Hub | latest | open Hub | — |
| Unity Editor | 6.3 LTS (`6000.3.9f1`) | Unity Hub → Installs | Install with the **Android Build Support** module (includes OpenJDK + Android SDK + NDK). |
| Meta XR All-in-One SDK | v85.0.0 | Package Manager | Installed via Unity Package Manager inside the project. |
| ROS-TCP-Connector | local clone | `cat ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector/package.json` | Clone separately; referenced via `file://` in `Packages/manifest.json`. **Apply the reconnect patch** — see [Installation §2](#2-patch-ros-tcp-connector-required). |
| URDF-Importer | local clone | same pattern | — |
| TextMeshPro | built-in to Unity | Package Manager | Font asset **must be explicitly bundled** for Android builds (see Limitations). |
| Ubuntu | 22.04 LTS | `lsb_release -a` | Required by ROS 2 Humble. |
| ROS 2 | Humble Hawksbill | `ros2 --version` | Native install (`ros-humble-desktop`). Run `sudo rosdep init` + `rosdep update` once after install. |
| `ros_tcp_endpoint` | from `Unity-Technologies/ROS-TCP-Endpoint`, branch `main-ros2` | `ros2 pkg list \| grep ros_tcp_endpoint` | Built from source in `ros2_ws/src/`. |
| `ur_onrobot_control` | from `UR_OnRobot_ROS2` (Tony Le) | `ros2 pkg list \| grep ur_onrobot` | Combined UR + OnRobot driver. Built from source in `ros2_ws/src/`. Required for the gripper `finger_width` field. |
| `colcon` | any | `colcon --help` | `sudo apt install python3-colcon-common-extensions` |
| `adb` | any | `adb version` | `sudo apt install android-tools-adb` |

### Network requirements

- **Subnet:** the laptop WiFi adapter and the Quest must both be on `192.168.0.0/24` (or whatever your robot router uses). The Quest cannot reach the laptop across different subnets without explicit routing.
- **Port:** TCP `10000` open on the laptop for `ros_tcp_endpoint`. No special firewall rules are needed on a default Ubuntu install.
- **Listen address:** `ros_tcp_endpoint` must bind to `0.0.0.0` (all interfaces), not `localhost`. The launch command sets this with `--ros-args -p ROS_IP:=0.0.0.0`.
- **Quest IP:** assigned by the router via DHCP. Find it on the Quest in **Settings → WiFi → \[network\] → Details**, or on the laptop via `arp -a | grep -i quest` after the Quest has connected.
- **Laptop IP:** find with `hostname -I`. Pick the WiFi-side IP (the `192.168.0.x` one) — that's what goes into Unity's ROS Settings.

### Pre-flight checklist (before first run)

Tick these off in order — most "it doesn't work" failures come from a missed step here.

1. `lsb_release -a` shows Ubuntu 22.04
2. `ros2 --version` shows Humble
3. `ros2 pkg list | grep ros_tcp_endpoint` returns the package
4. `ros2 pkg list | grep ur_onrobot` returns the packages
5. `cd ros2_ws && colcon build --symlink-install` exits cleanly
6. `adb devices` shows the Quest as `device` (not `unauthorized`)
7. Laptop and Quest are on the same `192.168.0.x` subnet (`hostname -I` on the laptop, Quest's WiFi network details)
8. ROS-TCP-Connector cloned at the repo root **and the reconnect patch applied**
9. URDF-Importer cloned at the repo root
10. `Packages/manifest.json` references both packages via `file://`
11. Unity Editor opens the project without Console errors
12. `Robotics → ROS Settings` shows the laptop's WiFi IP as ROS IP, port `10000`

---

## Installation

### 1. Clone the repository and packages

```bash
git clone git@github.com:John-A-Chen/HoloAssist.git HoloAssist_Main
cd HoloAssist_Main

# Unity packages are gitignored — clone separately at the repo root
git clone git@github.com:Unity-Technologies/ROS-TCP-Connector.git
git clone git@github.com:Unity-Technologies/URDF-Importer.git
```

### 2. Patch ROS-TCP-Connector (required)

Upstream v0.7.0 has a reconnect bug: after `ros_tcp_endpoint` restarts, Unity does not re-register subscribers. Symptom — `/joint_states` stops arriving after the first Play/Stop cycle even though the connection is shown as healthy.

Edit `ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector/Runtime/TcpConnector/RosTopicState.cs`. Find `OnConnectionEstablished` and remove the `!SentSubscriberRegistration` guard so registrations are re-sent on every new connection:

```csharp
internal void OnConnectionEstablished(NetworkStream stream)
{
    if (m_SubscriberCallbacks.Count > 0)
    {
        m_ConnectionInternal.SendSubscriberRegistration(
            m_Topic, m_RosMessageName, stream);
        SentSubscriberRegistration = true;
    }
    // ... rest unchanged
}
```

Re-apply this patch every time the package is freshly cloned.

### 3. Open in Unity

1. Launch Unity Hub → **Open Project** → select `Unity/My project/`.
2. Wait for the package import spinner to finish.
3. Open `Assets/Scenes/SampleScene.unity`.

### 4. Set the ROS IP

`Robotics → ROS Settings`:

- **ROS IP Address:** the laptop's WiFi IP (e.g. `192.168.0.102`)
- **ROS Port:** `10000`

### 5. (Optional) Run the scene-setup menu

If you started from a fresh scene, click `GameObject → HoloAssist → Setup All UI Features`. This creates all panel GameObjects and wires their references automatically.

### 6. Build and deploy to Quest 3

`File → Build Settings → Android → Build and Run`. The APK installs and launches on the headset.

If panels render pink/magenta on the Quest, also do:  
`Project Settings → Graphics → Always Included Shaders` → add `Universal Render Pipeline/Unlit`.

---

## Launch Files

The ROS 2 infrastructure that the XR scene depends on is shared across all four subsystems. This section documents each launch file: where it lives, what it starts, and every argument that matters.

### `launch.sh` and `launch.py` — one-command launcher

| File | Path |
|---|---|
| Shell wrapper | `launch.sh` (repo root) |
| Python launcher | `launch.py` (repo root) |

`launch.sh` does only three things: sources `/opt/ros/humble/setup.bash`, sources `ros2_ws/install/setup.bash`, then calls `launch.py "$@"` with all CLI arguments forwarded. Its sole purpose is to avoid having to manually source ROS before running.

`launch.py` runs the full startup sequence in a single process that owns all children:

| Step | What happens |
|---|---|
| 1 | Detects best WiFi IP — prefers any interface on `192.168.0.x`; falls back to an external socket trick |
| 2 | Calls `ros2 launch ur_onrobot_control start_robot.launch.py` with the right arguments |
| 3 | Waits **8 seconds** for the driver to finish loading |
| 4 | Calls `/controller_manager/switch_controller` service (via `ros2 service call`) to activate velocity controllers |
| 5 | Calls `ros2 run ros_tcp_endpoint default_server_endpoint` for the Unity bridge |
| 6 | Starts `beacon.py` (UDP multicast) so `ROSAutoConnect.cs` on the Quest can find the laptop automatically |

`Ctrl+C` sends `SIGTERM` to `launch.py`, which then terminates all children in reverse-start order and waits for clean shutdown.

**CLI arguments:**

| Argument | Default | Description |
|---|---|---|
| `--robot-ip <IP>` | *(none)* | If supplied, passes `robot_ip:=<IP>` to the driver and uses real hardware. If omitted, uses `use_fake_hardware:=true`. |
| `--ros-ip <IP>` | `0.0.0.0` | IP passed as `ROS_IP` to `ros_tcp_endpoint`. Leave at `0.0.0.0` so the bridge listens on all interfaces. |
| `--no-rviz` | *(not set)* | Passes `launch_rviz:=false` to the driver. Use on low-resource machines. |

**Example invocations:**

```bash
./launch.sh                              # fake hardware, RViz on, all interfaces
./launch.sh --robot-ip 192.168.0.194     # real UR3e
./launch.sh --robot-ip 192.168.0.194 --no-rviz   # real robot, lighter
```

---

### `start_robot.launch.py` — combined UR + OnRobot driver

| Item | Value |
|---|---|
| **File** | `ros2_ws/src/ur_onrobot/ur_onrobot_control/launch/start_robot.launch.py` |
| **Package** | `ur_onrobot_control` |
| **Direct invocation** | `ros2 launch ur_onrobot_control start_robot.launch.py [args]` |

This is the main driver launch file. It starts everything needed to talk to (or simulate) the UR3e + OnRobot RG2 gripper.

**Nodes it starts:**

| Node | When | Role |
|---|---|---|
| `ros2_control_node` (controller_manager) | fake hardware only | Runs the ros2_control loop in software simulation |
| `ur_ros2_control_node` (ur_robot_driver) | real hardware only | Connects to the physical UR3e via Ethernet, drives the real arm |
| `robot_state_publisher` | always | Broadcasts URDF to `/robot_description`; publishes TF tree from joint states |
| `controller spawner` | always | Spawns all listed controllers (active and inactive) into the controller_manager |
| `dashboard_client` | real hardware only | Connects to UR's web dashboard for mode/safety status |
| `robot_state_helper` | real hardware only | Monitors UR safety state; re-enables after faults |
| `rviz2` | if `launch_rviz:=true` | Opens RViz with the `view_robot.rviz` config from `ur_onrobot_description` |

**Key launch arguments:**

| Argument | Default | Required? | Description |
|---|---|---|---|
| `ur_type` | `ur3e` | No | Robot model. Choices: `ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur16e`, `ur20`, `ur30`. |
| `onrobot_type` | `rg2` | No | Gripper model. Choices: `rg2`, `rg6`. |
| `robot_ip` | `192.168.56.101` | **Yes, for real hardware** | Ethernet IP of the UR3e. For HoloAssist: `192.168.0.194`. Ignored when `use_fake_hardware:=true`. |
| `use_fake_hardware` | `false` | No | If `true`, substitutes a software simulation; no physical robot needed. |
| `launch_rviz` | `true` | No | Whether to open RViz. Set to `false` to save memory. |
| `kinematics_config` | packaged `ur3e_calibration.yaml` | No | Per-robot calibration YAML. Only matters if the physical robot's kinematics deviate from nominal. |
| `tf_prefix` | `""` | No | Prefix prepended to all TF frame names. Used only in multi-robot setups. |
| `headless_mode` | `false` | No | Run without teach-pendant interaction (bypasses the safety/mode UI on the pendant). |

**Controllers it spawns:**

*Active on start:*  
`joint_state_broadcaster`, `io_and_status_controller`, `speed_scaling_state_broadcaster`, `force_torque_sensor_broadcaster`, `ur_configuration_controller`, `finger_width_trajectory_controller` (and `tcp_pose_broadcaster` on real hardware).

*Inactive on start (must be switched in for teleoperation):*  
`forward_velocity_controller`, `finger_width_controller`, `scaled_joint_trajectory_controller`, and others.

> **Why does the controller switch matter?**  
> The driver starts `scaled_joint_trajectory_controller` (trajectory mode) as the active joint controller. Teleoperation and XR visualisation need `forward_velocity_controller` (direct velocity commands). These two controllers both claim the same hardware resource and cannot be active simultaneously. The controller switch in step 4 of `launch.py` — or the manual `ros2 control switch_controllers` command — deactivates trajectory mode and activates velocity mode. Without this switch, publishing velocity commands to `/forward_velocity_controller/commands` has no effect because the controller is inactive.

---

### `ros_tcp_endpoint` — ROS-Unity bridge

| Item | Value |
|---|---|
| **Package** | `ros_tcp_endpoint` (built from source in `ros2_ws/src/ROS-TCP-Endpoint/`) |
| **Command** | `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0` |
| **Default port** | `10000` |

This node bridges the ROS 2 DDS network and Unity's `ROSConnection` over a persistent TCP socket. All ROS traffic between the Quest and the laptop flows through this single connection.

**`ROS_IP` must be `0.0.0.0`** — not the laptop's specific IP. The endpoint needs to accept connections from the Quest (WiFi interface) as well as any local Unity Editor connections (loopback). Setting `ROS_IP` to a specific interface address blocks connections from other interfaces.

If the endpoint needs to be restarted after Unity Play/Stop (due to the known v0.7.0 race condition), re-run the exact same command in a new terminal — the port is freed immediately on process exit.

---

## Running the System

### Quickest path — launcher script

```bash
./launch.sh                                    # fake hardware (no real robot needed)
./launch.sh --robot-ip 192.168.0.194           # real UR3e
```

`launch.sh` sources ROS, starts the `ur_onrobot` driver, switches to velocity controllers, and runs `ros_tcp_endpoint` — all in one terminal.

### Manual — three terminals (if `launch.sh` isn't available)

```bash
# Terminal 1 — robot driver (fake hardware)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=true

# Terminal 2 — switch to velocity controllers (after Terminal 1 finishes loading)
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers \
  --activate forward_velocity_controller finger_width_controller \
  --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller

# Terminal 3 — ROS-Unity bridge
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### In Unity

Press **Play** in the Editor, or deploy the APK and put on the Quest 3.

### Expected behaviour

| What you see | What it means |
|---|---|
| Two floating panels appear left and right of view | Panels built; camera follow is working |
| Status row shows **"Waiting for ROS"** | Unity reached the bridge but no `/joint_states` yet |
| Status row shows **"Connected"**, joint angles update live | `/joint_states` is arriving correctly |
| Press **Y** on left controller | Radial menu opens above the left controller |
| Point right controller at a button, pull trigger | Button activates (green = ON, red = OFF) |
| Drop an object into a bin | That bin's row turns green: "1 object(s)" |
| Toggle **TF Axes** in the radial menu | RGB axes appear at every robot joint and scene object |

---

## Configurable Settings

All adjustable in the Unity Inspector without recompiling.

### `RobotDataPanel`

| Parameter | Default | Effect | How to verify |
|---|---|---|---|
| `distanceFromCamera` | `1.8` m | Panel distance from operator's eyes | Change at runtime — panel slides closer/further |
| `offset` | `(-0.45, 0, 0)` | Lateral/vertical position relative to forward view | Negative X = left of view |
| `followSpeed` | `2.5` | Camera-follow smoothing — lower = laggier | Turn head quickly; panel should glide, not snap |
| `panelWidth` | `0.8` m | Panel width in world space | Visible width changes; text auto-rewraps |
| `fontSize` | `0.25` | Text size for data rows | Editor-only — TMP rebuilds on enable |

### `BinStatusPanel`

| Parameter | Default | Effect |
|---|---|---|
| `offset` | `(0.45, 0, 0)` | Right side, mirrors the data panel |
| `distanceFromCamera` | `1.8` m | Symmetric with data panel |

### `BinDetector` (per bin)

| Parameter | Default | Effect |
|---|---|---|
| `binName` | `"Bin"` | Display name in `BinStatusPanel` |
| `triggerSizeMultiplier` | `1.2` | Trigger zone size relative to mesh bounds — increase if detection misses objects near edges |
| `buildContainerColliders` | `true` | Auto-builds floor + 4 walls so objects don't fall through |
| `disableBlockingMeshColliders` | `true` | Disables convex MeshColliders that would block the bin interior |
| `wallThickness` | `0.01` m | Container wall thickness |
| `triggerYOffset` | `0.05` m | Vertical offset of trigger zone (positive = raised slightly above bin floor) |

To verify the trigger zone, select the bin in the Editor — `OnDrawGizmosSelected()` draws the zone in yellow (empty) or green (objects inside).

### `ROSObjectPublisher`

| Parameter | Default | Effect |
|---|---|---|
| `frameName` | `"unity_cube"` | TF child frame name (visible in RViz TF tree) |
| `parentFrame` | `"base_link"` | TF parent frame — must match the robot's base frame |
| `publishRate` | `10` Hz | Pose broadcast frequency |
| `publishMarker` | `true` | Whether to also publish a `/unity_markers` cube |
| `markerColor` | pink, semi-transparent | Marker tint in RViz |

To verify: `ros2 topic echo /tf | grep -A 5 unity_cube` should print updates at the publish rate.

### `JointTFVisualizer`

| Parameter | Default | Effect |
|---|---|---|
| `showAxes` | `false` | Toggle axes at runtime (also toggleable from Radial Menu) |
| `axisLength` | `0.08` m | Axis length on robot links |
| `envAxisLength` | `0.12` m | Axis length on non-robot scene objects |
| `autoDiscoverMovingObjects` | `true` | Add axes to any non-static scene object with a renderer |

### `RadialMenu`

| Parameter | Default | Effect |
|---|---|---|
| `menuRadius` | `0.1` m | Radius of the button ring |
| `hoverHeight` | `0.12` m | Height of menu above left controller |

---

## Future Integration: Perception Subsystem

This section describes how Subsystem 4 (XR Scene & Visualisation) would change when the Perception subsystem (John's subsystem) is fully integrated. All topic names are drawn directly from the actual perception codebase at `ros2_ws/src/holo_assist_depth_tracker/` and `ros2_ws/src/holo_assist_depth_tracker_sim/`.

### Current state — manually-placed stub

At present, `ROSObjectPublisher` acts as a placeholder. An operator drags a Unity cube to an approximate real-world position; that pose is broadcast outward to `/tf` and `/unity_markers`. This is a one-way **outbound** signal — Subsystem 4 produces a pose, it does not consume one.

When Perception is integrated, this flow **reverses**: Perception detects real objects from the RealSense camera, publishes their poses to ROS, and Subsystem 4 subscribes to those poses to drive the virtual object overlay on the Quest.

---

### New ROS Inputs (Subsystem 4 would subscribe)

#### Real hardware path

These topics come from `workspace_perception_node.py` in the `holo_assist_depth_tracker` package, which fuses RealSense depth data with AprilTag detections to localise objects in the workspace.

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/holoassist/perception/object_pose_workspace` | `geometry_msgs/PoseStamped` | ~10–20 Hz | Primary output — detected object pose in `workspace_frame`. This is the topic Unity would subscribe to for placing virtual objects. |
| `/holoassist/perception/object_pose` | `geometry_msgs/PoseStamped` | ~10–20 Hz | Same pose in the camera optical frame (before workspace transform). Less useful to Unity directly. |
| `/holoassist/perception/object_marker` | `visualization_msgs/Marker` | ~10–20 Hz | RViz CUBE marker for the detected object. Unity can cross-check this but does not need to consume it. |

#### Simulation path (per-cube topics)

The sim perception node (`sim_cube_perception_node.py`) models four AprilTag cubes and publishes a separate topic set per cube. Cube names: `april_cube_1` (red), `april_cube_2` (green), `april_cube_3` (blue), `april_cube_4` (orange).

| Topic pattern | Type | Description |
|---|---|---|
| `/holoassist/sim/perception/{name}_pose` | `geometry_msgs/PoseStamped` | Per-cube detected pose in `workspace_frame`. Unity would subscribe to all four and spawn/move the corresponding virtual prefab. |
| `/holoassist/sim/perception/{name}_status` | `CubePerceptionStatus` (custom) | Richer per-cube state: `visible_now`, `within_fov`, `within_range`, `last_seen_available`, `last_seen_age`. Useful for driving UI indicators (e.g., dimming a virtual object when it's out of camera view). |

#### TF frames

The perception pipeline broadcasts `apriltag_cube_1_sim` through `apriltag_cube_4_sim` into the TF tree, parented under `workspace_frame`. These arrive in Unity via the `/tf` subscription already bridged through `ros_tcp_endpoint`. A `PerceptionObjectSubscriber` in Unity would look up these frames relative to `base_link` each frame to position virtual objects.

---

### New ROS Outputs (Subsystem 4 would publish)

With perception integration, Subsystem 4 would add one new outbound topic:

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/holoassist/unity/bin_events` | `std_msgs/String` (or custom) | Event-driven | Published by `BinDetector` when an object enters or leaves a bin trigger zone. Payload: JSON string with `bin_name`, `object_count`, `event` (`entered`/`exited`). Consumed by the Autonomous Sorting subsystem (Oliver) to confirm successful placement. |

The existing `/tf` (`unity_cube` frame) and `/unity_markers` outputs would be **retired** — replaced by the inbound perception subscription. The interactive cube would no longer be manually placed; it would be driven by the perception pose.

---

### Required changes in Unity

| Component | Current behaviour | Change needed |
|---|---|---|
| `ROSObjectPublisher.cs` | Broadcasts the manually-dragged cube's pose out to `/tf` + `/unity_markers` | **Replace** with a new `PerceptionObjectSubscriber.cs` that subscribes to `/holoassist/perception/object_pose_workspace` and moves the virtual GameObject to the received pose each frame. |
| Virtual object mapping | No mapping — one cube, manually placed | New component (e.g., `XRObjectMapper.cs`) — maps perception cube ID or name (`april_cube_1` → `april_cube_4`) to the corresponding Unity virtual prefab (bomb, safe item, etc.) and spawns/destroys accordingly. |
| `BinDetector.cs` | Calls `BinStatusPanel.SetBinStatus()` only | Add a `ROSBinEventPublisher.cs` component (or extend `BinDetector`) to publish `/holoassist/unity/bin_events` when trigger counts change. |
| `RobotDataPanel.cs` | Unchanged | Could optionally show a "perception active" indicator using the `last_seen_available` field from `CubePerceptionStatus`. |

---

### Coordinate frame alignment

This is the main technical challenge for integration. The perception pipeline publishes poses in `workspace_frame` — a frame established by John's workspace board calibration node (`workspace_alignment_camera_tf_node`). Unity works in the robot base frame (`base_link`).

**Option A — resolve in ROS (recommended):**  
The workspace alignment node already publishes the `workspace_frame` → `base_link` transform into `/tf`. The `PerceptionObjectSubscriber` in Unity requests the pose via a standard TF lookup through the bridge. Since `ros_tcp_endpoint` already bridges all `/tf` messages, no extra wiring is needed on the ROS side.

**Option B — configure the perception node:**  
Set the perception node's `workspace_frame` parameter to `base_link`. The node will publish poses directly in `base_link` — simpler for Unity, no TF arithmetic required. Requires coordination with John to confirm the parameter is exposed.

---

### What does not change

The rest of Subsystem 4 is not affected by perception integration:

- `BinDetector`, `BinStatusPanel`, `RobotDataPanel`, `JointTFVisualizer`, `RadialMenu` — all respond to Unity physics state and `/joint_states`; neither comes from perception.
- `JointStateSubscriber` and its `LastJointAnglesRad[]` cache — unchanged.
- The ROS-TCP bridge (`ros_tcp_endpoint`), launch procedure, and controller setup — unchanged.
- All visualisation toggle behaviour via the radial menu — unchanged.

---

## Known Limitations

- **Bin detection requires Unity physics objects.** `BinDetector` fires on Unity `Rigidbody` colliders entering the trigger zone. It cannot detect real objects seen through Quest passthrough — those objects exist only in the camera image, not the Unity scene. For the demo, sorted objects must be Unity GameObjects with a `Rigidbody`.
- **`ROSObjectPublisher` publishes a manually-placed cube, not a perception-detected object.** The interactive cube is positioned by hand (or by dragging in XR). Once Perception (John) publishes detected-object poses to `/tf`, this subsystem will switch to consuming those rather than producing its own.
- **Portal mesh is missing some textures after the latest merge.** The test-chamber geometry renders but several materials are grey. This is cosmetic; functionality is unaffected.
- **`ros_tcp_endpoint` must be restarted after every Unity Play/Stop cycle.** Known v0.7.0 race condition. Restart with: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`.
- **Gripper percentage requires the `ur_onrobot` driver.** The panel's gripper field reads `finger_width` from `/joint_states`, which is published by `ur_onrobot_control` but not by the vanilla UR driver alone.
- **TextMeshPro font may not bundle in the Android build by default.** If text is invisible on device, assign a `TMP_FontAsset` explicitly in the Inspector or place it under `Resources/`.

---

## Troubleshooting & FAQs

**Panel appears but all joint angles show 0.0°**

- ROS connected: run `ros2 topic echo /joint_states`. If nothing prints, the controllers aren't active — run the `switch_controllers` command. If `/joint_states` is publishing but the panel still shows 0°, check that `JointStateSubscriber` is on the `ur3e_rg2` root GameObject.
- ROS not running: 0.0° is correct (URDF rest pose). The status row reads "Waiting for ROS" to distinguish this from an error.

---

**Panel doesn't move with the headset**

The XR camera wasn't found. Check the Console for `[RobotDataPanel]` log lines. The MR template's eye-anchor camera is often not tagged `MainCamera`; the panel tries four named fallbacks (see *Implementation Notes §4*) and finally any active `Camera`. If still not found, drag the XR camera Transform into the panel's Inspector field directly.

---

**Radial menu opens but trigger doesn't select buttons**

The right-controller transform was `null` at `Start()`. Check Console for `[RadialMenu] Could not find left/right controller`. Drag the right-hand controller Transform into `RadialMenu → rightControllerOverride`.

---

**All panel quads render pink/magenta on Quest**

URP shader stripping. Fix: `Project Settings → Graphics → Always Included Shaders` → add `Universal Render Pipeline/Unlit`.

---

**TextMeshPro text is invisible on Quest**

The default TMP font wasn't bundled in the Android build. Assign a `TMP_FontAsset` to the panel components, or place the asset under `Resources/`.

---

**Bin Status Panel always says "No bins found"**

Either no `BinDetector` is in the scene, or all bin GameObjects were inactive when `BinStatusPanel.Start()` ran. Activate them in the hierarchy and press Play again.

---

**Objects bounce off the top of bins instead of falling inside**

The bin FBX has a convex `MeshCollider` filling the interior volume. `disableBlockingMeshColliders = true` should turn it off; check the Console for `[BinDetector] BinA: disabled convex MeshCollider on '...'`. If that log line is missing, the collider wasn't found — flip the toggle on in the Inspector and verify.

---

**`/tf` frames for the interactive cube don't appear in RViz**

`ros2 topic echo /tf | grep unity_cube` — if messages arrive but RViz shows nothing, set RViz's Fixed Frame to `base_link` and confirm the robot driver is running (so `base_link` exists in the TF tree). If no messages arrive, check the Unity Console for `[ROSObjectPublisher]` warnings (e.g. parent-frame GameObject not found).

---

**`ros_tcp_endpoint` crashes on Unity Play/Stop**

Known race condition in v0.7.0. Restart the endpoint: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0`.

---

**Latency between robot motion and panel display feels noticeable**

Expected when running over WiFi to the Quest. The chain is: real robot → UR driver → `/joint_states` → `ros_tcp_endpoint` → TCP over WiFi → `JointStateSubscriber.OnJointState` → `LastJointAnglesRad[]` → next frame `LateUpdate` reads it. Each hop adds milliseconds; total is typically 30–80 ms. The panel display itself adds at most one frame (~14 ms at 72 Hz Quest refresh).
