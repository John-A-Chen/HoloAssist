# Integration Changes — Perception + Autonomous + Teleop + XR

Integrates John's perception pipeline and Oliver's autonomous sorting (already merged together) with Nic's teleoperation subsystem and Seb's XR scene/visualisation.

---

## Architecture Overview

```
                         Quest 3 "Auto Mode" button
                                   │
                    OperatingModeController.cs
                    publishes /holoassist/mode_command
                                   │
                          ┌────────▼─────────┐
                          │    Dashboard      │
                          │ ros_interface.py  │
                          │ switch_controllers│
                          └────────┬─────────┘
                                   │
               publishes /holoassist/mode_status
                                   │
                    ┌──────────────▼──────────────┐
                    │    auto_sort.py              │
                    │ monitors cube poses          │
                    │ triggers pick-place service  │
                    └──────────────┬──────────────┘
                                   │
             /holoassist/pick_cube_to_bin service
                                   │
                    ┌──────────────▼──────────────┐
                    │ pick_place_sequencer         │
                    │ approach→grip→lift→bin→release│
                    └──────────────┬──────────────┘
                                   │
    Perception tracks cube → cube_pose_relay → Unity CubePoseSubscriber
                                                    │
                                        Virtual cube moves in XR
                                                    │
                                    BinDetector triggers → BinStatusPanel
```

### Controller Ownership

| Mode | Active Controllers | Inactive Controllers |
|---|---|---|
| **TELEOP** | `forward_velocity_controller`, `finger_width_controller` | `scaled_joint_trajectory_controller`, `finger_width_trajectory_controller` |
| **MOVEIT** | `scaled_joint_trajectory_controller`, `finger_width_trajectory_controller` | `forward_velocity_controller`, `finger_width_controller` |

Only one set is active at a time. Switching is done atomically by the dashboard via `ros2 control switch_controllers`.

### ROS Topic Contract

| Topic | Type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `/holoassist/mode_command` | `std_msgs/String` | Unity `OperatingModeController.cs` | Dashboard `ros_interface.py` | Quest 3 requests mode change ("TELEOP" or "MOVEIT") |
| `/holoassist/mode_status` | `std_msgs/String` | Dashboard `ros_interface.py` (2 Hz) | Unity `OperatingModeController.cs`, `auto_sort.py` | Current confirmed operating mode |
| `/holoassist/perception/april_cube_{1-4}_pose` | `geometry_msgs/PoseStamped` | John's `cube_pose_node` (workspace_frame) | `cube_pose_relay.py`, `auto_sort.py` | Detected cube positions |
| `/holoassist/unity/cube_{1-4}_pose` | `geometry_msgs/PoseStamped` | `cube_pose_relay.py` (base_link) | Unity `CubePoseSubscriber.cs` | Cube positions in base_link for Unity |
| `/pick_place/command` | `std_msgs/String` (JSON) | `auto_sort.py` / `pick_place_service_node` | `pick_place_sequencer` | Pick-place commands |
| `/pick_place/mode` | `std_msgs/String` | Dashboard `ros_interface.py` | `pick_place_sequencer` | "run" or "pause" — controls sequencer |
| `/holoassist/pick_cube_to_bin` | Service (`PickCubeToBin`) | `auto_sort.py` | `pick_place_service_node` | Fire-and-forget sort command |

---

## New Files

### `CubePoseSubscriber.cs` (Unity)

**Path:** `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` (206 lines)

Subscribes to `/holoassist/unity/cube_{1-4}_pose` (PoseStamped in base_link frame, published by `cube_pose_relay.py`). For each cube:
- Converts ROS coordinates to Unity world coordinates relative to the robot base
- Spawns a virtual object (from prefab array or auto-generated coloured cube)
- Smoothly interpolates position/rotation each frame
- Hides objects after a configurable timeout (default 3s) if no pose updates arrive

Inspector fields:
- `robotBase` — the robot's base_link Transform (required)
- `cubePrefabs` — optional array of prefabs for gamified object mapping
- `objectScale` — default 0.04 (4 cm, matching physical AprilTag cubes)
- `smoothing` — 0.0 (snap) to 0.95 (very smooth)
- `poseTimeout` — seconds before hiding stale objects

Public API: `IsCubeVisible(int)`, `GetCubePosition(int)` for other scripts to query.

### `OperatingModeController.cs` (Unity)

**Path:** `Unity/My project/Assets/Scripts/OperatingModeController.cs` (55 lines)

Bridges operating mode between Quest 3 and the dashboard:
- Publishes to `/holoassist/mode_command` when the operator toggles mode
- Subscribes to `/holoassist/mode_status` to stay in sync with the dashboard's confirmed state
- Exposes `currentMode` (string), `IsTeleop`, `IsMoveit` properties
- Public methods: `SetMode("TELEOP"|"MOVEIT")`, `ToggleMode()`

Called by RadialMenu's new "Auto Mode" button.

### `cube_pose_relay.py` (ROS node)

**Path:** `nic/cube_pose_relay.py` (128 lines)

Transforms cube poses from `workspace_frame` (perception's output frame) to `base_link` (Unity's reference frame) using TF2:
- Subscribes to `/holoassist/perception/april_cube_{1-4}_pose`
- Looks up `workspace_frame → base_link` transform via `tf2_ros`
- Applies translation + rotation, republishes as `/holoassist/unity/cube_{1-4}_pose`
- Silently drops poses when the TF transform is unavailable (e.g. board not yet locked)

No external dependencies beyond standard ROS 2 (`tf2_ros`, `geometry_msgs`). Pure numpy for quaternion math — no scipy needed.

### `auto_sort.py` (ROS node)

**Path:** `nic/auto_sort.py` (207 lines)

Autonomous sorting orchestrator. When the system is in MOVEIT mode:
1. Monitors `/holoassist/perception/april_cube_{1-4}_pose` for detected cubes
2. Waits `settle_time` (default 3s) for the area to be clear and pose to stabilize
3. Calls `/holoassist/pick_cube_to_bin` service (or falls back to topic-based command)
4. Tracks which cubes have been sorted to avoid re-sorting
5. Waits `cooldown_after_sort` (default 5s) between sorts

Cube-to-bin mapping (default): cube_1→bin_1, cube_2→bin_2, cube_3→bin_3, cube_4→bin_4.

Resets sorted-cube tracking when mode switches back to MOVEIT (so cubes can be re-sorted in a new session).

### `UnityChanges.md`

**Path:** `nic/UnityChanges.md` (98 lines)

Step-by-step instructions for all changes that must be made inside the Unity Editor (creating GameObjects, attaching scripts, wiring Inspector fields). See that file for the full checklist.

---

## Modified Files

### `launch.py` (rewritten)

**Path:** `nic/launch.py` (305 lines)

Previously: started UR driver + velocity controller switch + ros_tcp_endpoint + beacon.

Now: unified launcher for the full integrated stack. Starts (in order, with timed delays):

| Phase | Delay | What |
|---|---|---|
| 1 | t=0s | UR + OnRobot driver |
| 2 | t=10s | Controller switch to TELEOP (default) |
| 3 | t=10s | ROS-TCP endpoint + beacon + rosbridge |
| 4 | t=13s | MoveIt 2 (move_group) |
| 5 | t=16s | Perception pipeline + cube pose relay |
| 6 | t=18s | Workspace scene manager |
| 7 | t=20s | Coordinate listener + pick-place sequencer + service + auto-sort |

New CLI flags:
- `--no-perception` — skip camera/AprilTag pipeline
- `--no-moveit` — skip MoveIt and all autonomous sorting nodes
- `--no-rosbridge` — skip rosbridge WebSocket server
- `--no-autosort` — skip the auto-sort orchestrator
- `--rosbridge-port` — default 9090

Fake hardware mode (`--robot-ip` omitted) still works — uses `use_fake_hardware:=true`.

### `dashboard/ros_interface.py`

**Path:** `nic/dashboard/ros_interface.py` (749 lines, was ~700)

Changes:

**New publishers:**
- `/holoassist/mode_status` (String, 2 Hz) — broadcasts current operating mode so Quest 3 and auto_sort stay in sync
- `/pick_place/mode` (String) — sends "run"/"pause" to the pick-place sequencer

**New subscription:**
- `/holoassist/mode_command` (String) — receives mode switch requests from Quest 3's OperatingModeController

**E-stop updated (`emergency_stop()`):**
- Now pauses the pick-place sequencer (publishes "pause" to `/pick_place/mode`)
- Deactivates **all four** controllers (both velocity and trajectory) regardless of current mode
- In TELEOP mode, still burst-publishes velocity zeros + runs safety publish loop

**Resume updated (`resume()`):**
- Detects current operating mode and reactivates the correct controller pair
- In MOVEIT mode, also publishes "run" to the pick-place sequencer after controller activation

**New internal methods:**
- `_mode_command_cb()` — handles mode requests from Quest 3
- `_publish_mode_status()` — 2 Hz timer callback
- `_publish_pick_place_pause()` — sends pause to sequencer
- `_deactivate_all_controllers()` — replaces old `_deactivate_controller()`, stops all four
- `_activate_for_mode(mode)` — replaces old `_activate_controller()`, mode-aware

### `RadialMenu.cs` (Unity)

**Path:** `Unity/My project/Assets/Scripts/RadialMenu.cs` (687 lines, was ~677)

Changes:
- Added `public OperatingModeController modeController` field (Inspector-assignable)
- Added "Auto Mode" button on page 1 (Robot Controls page) that calls `modeController.ToggleMode()`
- Button toggles between TELEOP and MOVEIT operating modes

---

## Communication Bridges

Both run simultaneously:

| Bridge | Port | Consumer | Purpose |
|---|---|---|---|
| `ros_tcp_endpoint` | 10000 | Unity (ROS-TCP-Connector) | Joint states, velocity commands, cube poses, mode topics |
| `rosbridge_websocket` | 9090 | Any WebSocket client | John's perception XR streaming path, future integrations |

---

## How to Run

### Full stack (fake hardware)
```bash
./launch.sh
# Dashboard in separate terminal:
./dashboard.sh --fullscreen
```

### Full stack (real robot)
```bash
./launch.sh --robot-ip 192.168.0.194
./dashboard.sh --fullscreen
```

### Teleop only (no perception, no MoveIt)
```bash
./launch.sh --no-perception --no-moveit
```

### Switching modes at runtime
- **Dashboard:** click TELEOP or MOVEIT buttons at bottom of screen
- **Quest 3:** open RadialMenu (Y button) → page 1 → "Auto Mode" button
- **E-stop:** works in both modes — deactivates all controllers, pauses sequencer

---

## Demo Flow

1. Launch full stack: `./launch.sh --robot-ip 192.168.0.194`
2. Start dashboard: `./dashboard.sh --fullscreen`
3. Hit Play in Unity, put on Quest 3
4. **Teleop phase:** operator uses RMRC/Hand Guide to sort objects manually
5. **Switch to autonomous:** press "Auto Mode" in RadialMenu (or MOVEIT on dashboard)
6. Place AprilTag cube on workspace — perception detects it, virtual object appears in XR
7. Auto-sort waits 3s for stability, then triggers pick-place
8. Robot autonomously picks cube and places in assigned bin
9. BinDetector in Unity confirms placement, BinStatusPanel shows result
10. Switch back to TELEOP at any time via dashboard or RadialMenu
