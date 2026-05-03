# Pose Handoff Contract — Perception → Motion

This document is the interface specification between perception outputs and the motion execution side. It covers two handoff patterns: continuous streaming (teleop/hover mode) and service-triggered pick-place.

---

## 1. Motion-side consumers

### coordinate_listener (`MoveItCoordinateTopicControl`)

Subscribed topics:

1. `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`) — explicit pose goal in planning frame
2. `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`) — position-only goal; listener selects orientation by policy
3. `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`) — legacy format

Default planning frame: `base_link`
Default end-effector: `gripper_tcp` (or `tool0` depending on launch config)

### pick_place_sequencer

Subscribed topics:

- `/pick_place/command` (`std_msgs/String`) — JSON: `{"block_id": "april_cube_1", "x": 0.15, "y": -0.30, "z": 0.020, "bin_id": "bin_1"}`
- `/pick_place/mode` (`std_msgs/String`) — `"run"` or `"pause"`

The sequencer drives the coordinate_listener through the full pick-place cycle: pregrasp hover → grasp descent → grip → lift → move above bin → place descent → release.

---

## 2. Handoff patterns

### Pattern A — Continuous streaming (teleop / hover mode)

Used for real-time following of a selected cube (e.g. HoloLens teleop, RViz selection).

Flow:
1. Something selects a cube name and publishes to `/holoassist/teleop/selected_cube`
2. The adapter (`selected_cube_to_moveit_target_node`) subscribes to the corresponding cube pose
3. Adapter TF-transforms position from `workspace_frame` to `base_link`
4. Adapter applies configurable XYZ hover offsets
5. Adapter throttles by minimum movement delta and minimum time interval
6. Publishes `/moveit_robot_control/target_pose` and `/moveit_robot_control/target_point`

Config (via `full_holoassist_sim.yaml` or `full_holoassist_hw.yaml`):
```yaml
holoassist_selected_cube_to_moveit_target:
  ros__parameters:
    target_z_offset_m: 0.10   # hover 10 cm above cube centre
    target_roll_rad: 3.14159  # gripper facing down
    target_pitch_rad: 0.0
    target_yaw_rad: 0.0
```

### Pattern B — Service-triggered pick (automated pick-place)

Used for explicit "pick cube N and put it in bin M" commands.

Flow:
1. Caller invokes `/holoassist/pick_cube_to_bin` service
2. `pick_place_service_node` reads the cube's latest pose from perception topics
3. Node TF-transforms position from `workspace_frame` to `base_link`
4. Node publishes `"run"` to `/pick_place/mode`
5. Node publishes JSON command to `/pick_place/command`
6. Service returns immediately (fire-and-forget — pick takes 30–60 s)
7. `pick_place_sequencer` executes the full pick/place cycle

Service definition:
```
string cube_name    # "april_cube_1" through "april_cube_4", or "1"–"4"
string bin_id       # "bin_1" through "bin_4", or "1"–"4"
---
bool success
string message
```

Cube pose source:
- **Sim**: `/holoassist/sim/truth/april_cube_N_pose` (configurable via `cube_pose_topic_prefix`)
- **Hardware**: `/holoassist/perception/april_cube_N_pose` (default prefix)

---

## 3. Frame conventions

| Frame | Role |
|---|---|
| `workspace_frame` | Perception outputs — cube poses are `PoseStamped` in this frame |
| `base_link` | Motion planning frame — all MoveIt goals must be in this frame |

Frame conversion happens in:
- `selected_cube_to_moveit_target_node` (Pattern A)
- `pick_place_service_node` (Pattern B)

Neither the perception nodes nor the motion nodes do cross-frame conversion themselves.

---

## 4. Coordinate conventions

Cube positions in `workspace_frame`:
- Z = 0 corresponds to the workspace surface (board top)
- Cube centres are at Z ≈ +0.020 m (half of 4 cm cube sitting on surface)
- Both hardware (`cube_pose_node`) and sim (`sim_cube_truth_node`) report cube **centres**, not top faces

Pick-place Z offsets (pick_place_sequencer params):
```
pregrasp_z_offset:    +0.10 m  (approach 10 cm above cube centre)
grasp_z_offset:        0.0 m  (descend to cube centre height)
place_above_z_offset: +0.15 m  (approach above bin)
place_z_offset:       +0.05 m  (final place height above bin bottom)
place_descent_enabled: true    (gripper descends before releasing)
```

---

## 5. Adapter reference implementation

`holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/selected_cube_to_moveit_target_node.py`

Key behaviour:
- Subscribes to `selected_cube_topic` (String, cube name) and `input_pose_topic` (PoseStamped)
- Throttles by `min_position_delta_m` (default 0.02 m) and `min_republish_period_sec` (default 1.0 s)
- On selection change, forces immediate republish
- Uses `tf2_ros.Buffer.lookup_transform` with `Time()` (latest available) for frame conversion

`holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/pick_place_service_node.py`

Key behaviour:
- Configurable `cube_pose_topic_prefix` parameter (default `/holoassist/perception` for hardware; set to `/holoassist/sim/truth` for sim)
- Caches latest pose per cube; returns error if no pose received yet
- TF transform uses 1.0 s timeout; returns error if TF unavailable
- Returns immediately after publishing command (does not wait for pick completion)

---

## 6. Listener feedback (integration health)

Per goal lifecycle:
```
QUEUED → PLANNING → PLANNED → EXECUTING → COMPLETE
                                         → FAILED (with reason in debug JSON)
                                         → INVALID (bad goal values)
```

Topics:
- `/moveit_robot_control/state` (`std_msgs/String`, transient local QoS)
- `/moveit_robot_control/status` (`std_msgs/String`, human-readable)
- `/moveit_robot_control/debug` (`std_msgs/String`, JSON)
- `/moveit_robot_control/complete` (`std_msgs/String`)

---

## 7. Failure modes

1. **No workspace TF** — TF lookup from `workspace_frame` to `base_link` fails silently; on hardware this means board is not yet locked (all 4 corner tags must be visible).
2. **No cube pose** — pick_place_service returns `success=false` if no pose received yet; cube_pose_node is silent until workspace locks.
3. **Stale quaternion** — adapter passes pose through unchanged; ensure cube_pose_node is producing valid quaternions.
4. **Wrong Z offset** — if gripper consistently misses high/low, tune `grasp_z_offset` in pick_place_sequencer params.
5. **High publish rate** — adapter throttles internally; do not bypass throttle or replanning churn will starve execution.
6. **Controller not active** — on hardware, `require_controller_check=true` will attempt to activate `scaled_joint_trajectory_controller`; if it fails, check UR driver status.

---

## 8. Verification commands

Detection and workspace:
```bash
ros2 topic echo /detections_all --once
ros2 topic echo /holoassist/perception/workspace_mode --once
```

Cube poses (hardware):
```bash
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/perception/april_cube_1_status --once
```

Cube poses (sim):
```bash
ros2 topic echo /holoassist/sim/truth/april_cube_1_pose --once
```

TF checks:
```bash
ros2 run tf2_ros tf2_echo base_link workspace_frame
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1
```

Service call:
```bash
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

MoveIt state:
```bash
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/debug --once
```

Workspace re-lock (hardware):
```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```
