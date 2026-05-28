# URSim + Quest 3 Testing — Handover

Reproduce real-hardware behaviour for the XR side without an actual UR3e or OnRobot RG2 in the lab. Use URSim (PolyScope simulator) as a stand-in for the robot and Quest 3 as the actual headset client.

This document assumes ROS 2 Humble, Unity 6.3 LTS, the HoloAssist repo at `~/git/rs2/HoloAssist_Main`, and the ros2_ws already built (`colcon build` once).

## Architecture

```
 ┌─ Quest 3 (uni WiFi / hotspot) ──────┐
 │  Unity APK                          │
 │  • RobotController.cs               │
 │  • JointStateSubscriber.cs          │
 │  • CubePoseSubscriber.cs            │
 │  • RobotHUD.cs / RobotDataPanel.cs  │
 └──────────────┬──────────────────────┘
                │ TCP 10000
                ▼
 ┌─ Laptop (uni WiFi / hotspot) ───────────────────────────────┐
 │                                                             │
 │  ros_tcp_endpoint  (default_server_endpoint, port 10000)    │
 │            │                                                │
 │            │  ROS 2 topics (joint_states, /forward_…/cmds)  │
 │            ▼                                                │
 │  ur_robot_driver  (ros2_control_node)                       │
 │            │                                                │
 │            │  RTDE protocol, ports 30001-30004 + 50002      │
 │            ▼                                                │
 │  ┌─ URSim Docker (192.168.56.0/24 host-only network) ──┐    │
 │  │  PolyScope at 192.168.56.101                        │    │
 │  │  External Control URCap → 192.168.56.1:50002        │    │
 │  └─────────────────────────────────────────────────────┘    │
 │                                                             │
 └─────────────────────────────────────────────────────────────┘
```

Critical: the Quest never talks to URSim directly. It only talks to `ros_tcp_endpoint` on the laptop. URSim's `192.168.56.x` subnet is internal to Docker and irrelevant to the Quest.

## Critical gotcha — DO NOT use the OnRobot wrapper launch

The default project launcher (`./launch.sh` → `launch.py`) calls `ur_onrobot_control/start_robot.launch.py`. That launch loads the OnRobot RG2 ros2_control plugin, which does a **blocking Modbus read** to `/tmp/ttyUR`. URSim does not expose the UR tool-communication port (54321), so the plugin's `on_init` hangs forever and stalls the entire 500 Hz `controller_manager` update loop. Net effect:

- PolyScope's External Control program connects, enters speed mode, then aborts with `socket_read_binary_integer: timeout` because the driver can't push velocity commands fast enough.
- Arm doesn't move regardless of what Unity publishes.
- Symptom in `ros2 control list_hardware_components`: `OnRobotGripper state: id=0 label=unknown`.

**Workaround:** launch the bare upstream `ur_robot_driver/ur_control.launch.py` instead. No OnRobot plugin, no stall. The Unity gripper visual won't animate (no `finger_width` joint in `/joint_states`) but everything else works.

There's a project commit (`a506113`) that adds a `--fake-gripper` flag intended to address this, but the underlying xacro (`ros2_ws/src/ur_onrobot/...` and `ros2_ws/src/onrobot_description/...`) doesn't actually declare `use_fake_gripper_hardware`, so the flag is a no-op right now. Don't rely on it.

## Prerequisites

```bash
# Required apt packages (one-time):
sudo apt install ros-humble-ros2controlcli ros-humble-ur-client-library
# socat is no longer needed since we drop the OnRobot wrapper entirely.

# Build the workspace (one-time, or after pulling new commits):
cd ~/git/rs2/HoloAssist_Main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Runbook

Open four terminals. Each terminal needs ROS sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/git/rs2/HoloAssist_Main/ros2_ws/install/setup.bash
```

### Terminal 1 — URSim

```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

Wait for `URSim is up`. Open `http://localhost:6080/vnc.html` in a browser → PolyScope appears.

**PolyScope first-time setup** (config persists in `~/.ursim/`):
1. Bottom-left red icon → **ON** → **START** (releases brakes).
2. **File → Load Program** → `external_control.urp`.
   - If missing: **File → New Program → Structure → URCaps → External Control**, then **File → Save As** `external_control.urp`.
3. **Installation → URCaps → External Control**:
   - Host IP: `192.168.56.1`
   - Custom port: `50002`
4. Press **Play ▶** at the bottom. The bottom bar should show "Robot Program" with a spinner.

If PolyScope's program stops on its own with `Receive program failed`, see "Common failures" below.

### Terminal 2 — Bare UR driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true \
  headless_mode:=true
```

Wait for:
```
Robot connected to reverse interface. Ready to receive control commands.
```

`headless_mode:=true` makes the driver push the URScript automatically via the dashboard, so PolyScope auto-runs without you re-pressing Play after every restart. The manual Play in Step 1 is still needed for the first connection in some PolyScope versions; if the program is already running, the driver just hooks in.

### Terminal 3 — Switch to velocity control

```bash
ros2 control switch_controllers \
  --activate forward_velocity_controller \
  --deactivate scaled_joint_trajectory_controller
ros2 control list_controllers
```

Verify `forward_velocity_controller … active`. Unity's `RobotController.cs` publishes `Float64MultiArray` to `/forward_velocity_controller/commands`. If the trajectory controller is active instead, the arm won't move regardless of Unity's commands.

### Terminal 4 — Unity TCP bridge

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

`ROS_IP:=0.0.0.0` binds to all interfaces. The Quest will connect from whichever subnet the laptop is also on.

## Unity / Quest setup

1. **Get the laptop's WiFi IP:**
   ```bash
   hostname -I
   ```
   Pick the WiFi address (not `172.17.0.1` Docker, not `192.168.56.1` URSim host-only). Examples:
   - Uni WiFi → `172.19.x.x`
   - iPhone hotspot → `172.20.10.x`
   - Robot dedicated router → `192.168.0.x`

2. **In Unity Editor → Robotics → ROS Settings:**
   - ROS IP Address: laptop WiFi IP from step 1.
   - ROS Port: `10000`.

3. **`ROSAutoConnect` script in the scene:**
   - For uni WiFi / hotspot testing: **DISABLE** the component (Inspector → uncheck the script). It scans `192.168.0.101–110` only, won't find a non-robot-router subnet.
   - For real-hardware testing on the robot router (`192.168.0.x`): re-enable it.

4. **Build & Run** APK to Quest 3 (Android target, Quest plugged in via USB).

5. **Quest WiFi:** join the same WiFi as the laptop.

6. Launch the app on the Quest.

## Verification checklist

Before testing controls, run these in a spare terminal:

```bash
# Driver is alive and listening for PolyScope:
ss -tlnp 2>/dev/null | grep 50002
# Expected: LISTEN ... 0.0.0.0:50002 ... ur_ros2_control_node

# UR hardware is active:
ros2 control list_hardware_components
# Expected: ur3e ... state=active, all 6 joints/velocity = [available][claimed]

# Joint states streaming:
ros2 topic hz /joint_states
# Expected: ~500 Hz

# ros_tcp_endpoint can be reached from the laptop's WiFi address:
nc -zv $(hostname -I | awk '{print $1}') 10000
# Expected: "succeeded"

# Unity is connected (after Quest app launch, check Terminal 4 log):
# Expected: [UnityEndpoint]: Connection from <quest_ip>
#           RegisterSubscriber(/joint_states, ...) OK
#           RegisterPublisher(/forward_velocity_controller/commands, ...) OK

# Velocity commands flowing when you move the right stick:
ros2 topic echo /forward_velocity_controller/commands
# Expected: non-zero Float64MultiArray.data when the user touches the stick
```

If all checks pass, the right stick on Quest (in RMRC mode, the default) should move the arm in URSim and RViz.

## Sanity tests on the Quest

| Test | How | Pass criteria |
|---|---|---|
| Digital twin | Look at virtual UR3e in Quest | Joints animate matching URSim / RViz |
| RMRC Translate | Right stick + left stick | EE moves in world XYZ + yaw |
| RMRC sub-mode toggle | Left **X** button | HUD switches Translate ↔ Rotate |
| RMRC Rotate | Right stick + left stick X | Wrists 1/2/3 jog |
| Direct Joint | Menu cycle to Direct Joint, A/B select, right stick Y | Selected joint moves |
| Hand Guide | Menu cycle to Hand Guide, hold right grip | EE tracks controller position |
| EE Lock toggle | Left thumbstick **click** | HUD shows "LOCK ▼", tool tilts down |
| Radial menu | Hold left primary button → mode buttons | Mode switches, HUD updates |
| Gripper trigger | Right index trigger | Trigger value in HUD; **no gripper motion** in URSim (expected) |
| HUD/DataPanel updates | Move the robot via VNC's Move tab | Joint angles + EE pos in panels track within 1 s |

## Network scenarios summary

| Network | Quest joins | Unity ROS IP | ROSAutoConnect |
|---|---|---|---|
| Uni WiFi (laptop on `172.19.x.x`) | Same uni WiFi | Manual: laptop's WiFi IP | Disabled |
| iPhone/Android hotspot (`172.20.10.x` / `192.168.43.x` etc.) | Same hotspot | Manual: laptop's hotspot IP | Disabled |
| Robot dedicated router (`192.168.0.x`) | Robot router | Leave blank (autoconnect overrides) | **Enabled** |

When a teammate hosts the stack instead of you:
- They run `hostname -I` and tell you the WiFi IP.
- That IP goes into Unity ROS Settings on your machine.
- You rebuild the APK.
- Both the teammate's laptop and your Quest must join the same WiFi.
- Watch for "AP/client isolation" on some phone hotspots — devices on the same WiFi can't talk to each other. Test with `nc -zv <their-ip> 10000` from your laptop first.

## Common failures

| Symptom | Cause | Fix |
|---|---|---|
| PolyScope: `Receive program failed: 192.168.56.1:50002 connection refused` | Driver in Terminal 2 not running, or still initialising | Wait for `Robot connected to reverse interface...`. If driver crashed, restart it. |
| PolyScope: program starts, then aborts with `Socket timed out waiting for command on reverse_socket` | Gripper plugin stalled controller_manager — you used the OnRobot wrapper launch instead of bare `ur_robot_driver` | Re-launch with the **bare** `ur_robot_driver/ur_control.launch.py` (see Terminal 2). |
| `ros2 control list_controllers` shows `forward_velocity_controller inactive` | Default trajectory controller still active | Re-run the switch in Terminal 3. After every driver restart. |
| Quest UI shows initial joint angles but never updates | `RobotHUD`/`RobotDataPanel` bound to a stale `JointStateSubscriber` whose joint map is empty | Fixed in code via freshness check (`IsJssFresh` in both panels). Rebuild & redeploy the APK. Also check for duplicate `RobotController` / `JointStateSubscriber` GameObjects in the scene — there should be exactly one of each active. |
| Quest disconnects mid-test, then can't reconnect | `ros_tcp_endpoint` race condition on disconnect/reconnect (`InvalidHandle` exception) | Ctrl+C Terminal 4 and re-launch. Quest may also need a force-quit + relaunch. |
| `sequence size exceeds remaining buffer` flooding Terminal 4 | Unity-side serialization mismatch on a large outbound topic (usually `/headset/image_compressed`) | Disable `HeadsetStreamPublisher` for the test. JPEG payloads can intermittently exceed bridge buffer expectations. |
| `ros2 control` command says "invalid choice: 'control'" | `ros2controlcli` package not installed | `sudo apt install ros-humble-ros2controlcli`. |
| `nc -zv <laptop-ip> 10000` from another device fails on same WiFi | Hotspot has AP/client isolation, or laptop firewall blocking | Switch hotspot source. If `sudo ufw status` shows active: `sudo ufw allow 10000/tcp`. |

## Files of interest

| Path | Purpose |
|---|---|
| `Unity/My project/Assets/Scripts/RobotController.cs` | Mode state, input bindings, publishes velocity to `/forward_velocity_controller/commands`. EE Lock = left thumbstick click. X button = RMRC sub-mode toggle. |
| `Unity/My project/Assets/Scripts/JointStateSubscriber.cs` | Subscribes to `/joint_states`, drives ArticulationBody transforms (digital twin), exposes `LastJointAnglesRad` + `LastReceivedRealtime`. |
| `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` | Subscribes to `/holoassist/unity/cube_N_pose`, spawns / animates cube objects. Has pose averaging + outlier rejection + optional ObjectClass tagging. |
| `Unity/My project/Assets/Scripts/RobotHUD.cs` | Floating HUD panel. Picks the "live" RobotController (with InputActionAsset assigned) and the "fresh" JointStateSubscriber (received update in last 1 s). Falls back to controller's CurrentPositions, then to its own subscription. |
| `Unity/My project/Assets/Scripts/RobotDataPanel.cs` | World-space data panel. Same picker logic for RobotController + JSS. Re-derives `tool0` and `base_link` from the wired JSS hierarchy. |
| `Unity/My project/Assets/Scripts/ROSAutoConnect.cs` | Scans `192.168.0.101–110` on port 10000 for the robot router. Disable for non-robot-router subnets. |
| `Unity/My project/Assets/Resources/ROSConnectionPrefab.prefab` | Holds ROS IP/port. Used by `ROSConnection.GetOrCreateInstance()`. |
| `ros2_ws/src/onrobot_driver/src/RG.cpp` | OnRobot RG2 driver. Default grip force = 5 N. Blocks on connection retry (cause of the URSim stall). |
| `ros2_ws/src/onrobot_description/urdf/onrobot_macro.xacro` | Where the gripper ros2_control plugin is declared. Doesn't currently support split arm-real / gripper-fake. |
| `launch.py` (repo root) | Project's Python launcher (`./launch.sh` → `launch.py`). **Don't use for URSim** — it routes through the OnRobot wrapper. |

## Where the recent UI fixes live

These are the panels' "find the live data source" pickers, added 2026-05-26 because branch merges had created stale duplicate `RobotController` and `JointStateSubscriber` components in the scene. The picker logic:

1. **`PickBestController` in `RobotHUD.cs`** — prefers a `RobotController` with `inputActions != null` (i.e. the one wired to the InputActionAsset in Inspector). Stale duplicates that just got auto-instantiated typically have it null.
2. **`PickLiveJointStateSubscriber` in `RobotDataPanel.cs` and equivalent in `RobotHUD.cs`** — prefers a `JointStateSubscriber` that received a `/joint_states` update within the last 1 s (`IsJssFresh`). `HasReceivedJointState` alone is "sticky" — once true, never false — so a stale duplicate that got one message at scene-start and went silent would still pass. Freshness catches that.
3. **`tool0` and `base_link`** in `RobotDataPanel` are re-derived from the live JSS's transform when the JSS rebinds, so the EE pose tracks the same robot.

If the panels go stale again, first check `adb logcat -s Unity:* | grep -iE "RobotHUD|RobotDataPanel"` — you should see the bind log with `fresh=True`. If `fresh=False`, no JSS is receiving messages; check `ros2 topic hz /joint_states` is non-zero.

## Shutdown order

1. Quest: Oculus button → Quit app.
2. Ctrl+C terminals 4 → 3 → 2 (any order, but driver last is cleanest).
3. URSim: Ctrl+C Terminal 1, or `docker stop ursim` from another terminal.

## Memory of decisions

Repo persistence:
- This document (`Subsystem-4-Documentation/ursim-quest3-testing-handover.md`)

Claude session memory at `~/.claude/projects/-home-sebastian-git-rs2-HoloAssist-Main/memory/`:
- `ursim_quest3_setup.md` — the runbook in compact form
- `unity_ros_autoconnect_state.md` — current scene state of `ROSAutoConnect` (disabled), with reminder to re-enable for hardware testing
