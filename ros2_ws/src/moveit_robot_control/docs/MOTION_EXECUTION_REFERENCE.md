# Motion Execution Reference (`moveit_robot_control`)

This document covers the motion execution side of the `j0hn` branch: the coordinate listener, pick-place sequencer, pick-place service, and workspace management nodes.

---

## 1. `coordinate_listener` (MoveItCoordinateTopicControl)

**Executable:** `coordinate_listener`
**File:** `moveit_robot_control_node/moveit_robot_control.py`
**Class:** `MoveItCoordinateTopicControl`

### Goal input topics

| Topic | Type | Behaviour |
|---|---|---|
| `/moveit_robot_control/target_pose` | `geometry_msgs/msg/Pose` | Explicit position + orientation |
| `/moveit_robot_control/target_point` | `geometry_msgs/msg/Point` | Position only; orientation from policy |
| `/moveit_robot_control/target` | `moveit_robot_control_msgs/msg/TargetRPY` | Legacy format |

Goals are validated and queued in a deque; processed one-at-a-time.

### Planning strategy

For each goal:

1. Determine orientation candidates (from pose, or via `orientation_mode` policy)
2. Try Cartesian path with each candidate (`cartesian_max_step=0.005`, `min_cartesian_fraction=0.999`)
3. If Cartesian fails and `allow_pose_goal_fallback=true`: use OMPL pose-goal planning
4. In `auto` mode only: final fallback with free-orientation position-only planning

`orientation_mode` options:
- `auto` — robot tilts to reach target based on radius + height from base
- `current` — keep current end-effector orientation
- `fixed` — use `roll_deg`, `pitch_deg`, `yaw_deg` parameters

### Key parameters

| Parameter | Sim default | Hardware default | Description |
|---|---|---|---|
| `trajectory_topic` | `/joint_trajectory_controller/joint_trajectory` | `/scaled_joint_trajectory_controller/joint_trajectory` | Controller to publish trajectories to |
| `require_robot_status` | `false` | `true` | Check UR driver status before executing |
| `require_controller_check` | `false` | `true` | Activate scaled_joint_trajectory_controller if inactive |
| `allow_pose_goal_fallback` | `true` | `true` | Use OMPL if Cartesian fails |
| `velocity_scale` | `0.05` | `0.05` | Trajectory velocity scaling (0–1) |
| `pose_goal_planning_time` | `5.0` | `5.0` | OMPL planning timeout (s) |
| `avoid_flange_forearm_clamp` | `true` | `true` | Reject trajectories entering UR protective stop zone |
| `orientation_mode` | `auto` | `auto` | Orientation selection policy |

### Safety gates

Before execution:
1. MoveIt collision check along trajectory
2. Floor collision object in planning scene
3. UR flange-to-forearm clamp risk check (if `avoid_flange_forearm_clamp=true`)
4. Robot status checks (if `require_robot_status=true`): program running, robot mode, safety mode
5. Controller activation check (if `require_controller_check=true`): activates `scaled_joint_trajectory_controller` if needed

### Status outputs

| Topic | Type | Contents |
|---|---|---|
| `/moveit_robot_control/state` | String, transient local | `QUEUED → PLANNING → PLANNED → EXECUTING → COMPLETE/FAILED/INVALID` |
| `/moveit_robot_control/status` | String | Human-readable current action |
| `/moveit_robot_control/debug` | String | JSON: plan stats, trajectory length, failure reason |
| `/moveit_robot_control/complete` | String | Completion marker per goal |

---

## 2. `pick_place_sequencer`

**File:** `moveit_robot_control_node/pick_place_sequencer.py`

### Role

Higher-level pick-place state machine that drives the coordinate_listener through the full cycle for a block-to-bin operation.

### Command interface

**Receive a command:**
```bash
ros2 topic pub --once /pick_place/command std_msgs/String \
  "{data: '{\"block_id\": \"april_cube_1\", \"x\": 0.15, \"y\": -0.30, \"z\": 0.020, \"bin_id\": \"bin_1\"}'}"
```

Mode control:
```bash
ros2 topic pub --once /pick_place/mode std_msgs/String "{data: 'run'}"
ros2 topic pub --once /pick_place/mode std_msgs/String "{data: 'pause'}"
```

### Execution sequence

1. **Pregrasp** — move to `(x, y, z + pregrasp_z_offset)` above the block
2. **Grasp** — move to `(x, y, z + grasp_z_offset)`
3. **Close gripper** — grip block
4. **Lift** — return to pregrasp height
5. **Move above bin** — move to bin XY position at approach height (`place_above_z_offset`)
6. **Place descent** (if `place_descent_enabled=true`) — lower to `place_z_offset`
7. **Open gripper** — release block
8. **Retreat** — lift back to safe height

### Key parameters

| Parameter | Sim value | Hardware value | Description |
|---|---|---|---|
| `pregrasp_z_offset` | 0.10 | 0.10 | Approach height above cube centre (m) |
| `grasp_z_offset` | 0.0 | 0.0 | Grasp height relative to cube centre (m) |
| `place_above_z_offset` | 0.15 | 0.15 | Height above bin before descent (m) |
| `place_z_offset` | 0.05 | 0.05 | Place height above bin bottom (m) |
| `place_descent_enabled` | true | true | Descend to place_z_offset before releasing |
| `initial_mode` | run | run | Mode on startup |
| `orientation_mode` | auto | auto | EE orientation policy |

**Z offset convention:** Both sim (`sim_cube_truth_node`) and hardware (`cube_pose_node`) report cube **centres** at Z ≈ +0.020 m. `grasp_z_offset = 0.0` descends to the cube centre height; the gripper jaws bracket the cube at that height. Adjust if gripper consistently misses by a fixed vertical amount.

### Bin positions

Defined in `config/bin_poses.json`. Four bins: `bin_1` through `bin_4`.

---

## 3. `pick_place_service_node` (PickCubeToBin service bridge)

**File:** `holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/pick_place_service_node.py`
**Package:** `holo_assist_depth_tracker_sim`

### Role

Bridges the `/holoassist/pick_cube_to_bin` service call to the pick_place_sequencer command topic. Fire-and-forget: returns immediately after queuing the command; the pick takes 30–60 seconds asynchronously.

### Service

```
/holoassist/pick_cube_to_bin
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin
    string cube_name   # "april_cube_1"–"april_cube_4" or "1"–"4"
    string bin_id      # "bin_1"–"bin_4" or "1"–"4"
    ---
    bool success
    string message
```

### Cube pose source

Configurable via `cube_pose_topic_prefix` parameter:

| Mode | Prefix | Topics subscribed |
|---|---|---|
| Hardware | `/holoassist/perception` (default) | `/holoassist/perception/april_cube_N_pose` |
| Sim | `/holoassist/sim/truth` | `/holoassist/sim/truth/april_cube_N_pose` |

The sim launch (`full_holoassist_moveit_sim.launch.py`) explicitly sets `cube_pose_topic_prefix: /holoassist/sim/truth`.

### Example call

```bash
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

---

## 4. `workspace_scene_manager`

**File:** `moveit_robot_control_node/workspace_scene_manager.py`

### Role

- Publishes trolley mesh as a RViz marker array (`/workspace_scene/markers`)
- Optionally applies table collision object into MoveIt planning scene (`apply_table_collision`)
- Accepts JSON scene commands and PoseStamped block spawns

### Key parameters (from config YAML)

```yaml
workspace_scene_manager:
  ros__parameters:
    frame_id: base_link
    publish_table_mesh: true
    apply_table_collision: false
    table_mesh_resource: package://moveit_robot_control/meshes/UR3eTrolley_decimated.dae
    table_mesh_xyz: [0.031, -0.210, -1.05]   # mesh origin in base_link
    table_mesh_rpy_deg: [0.0, 0.0, 0.0]
    table_mesh_scale: [1.0, 1.0, 1.0]
```

---

## 5. `workspace_frame_tf`

**File:** `moveit_robot_control_node/workspace_frame_tf.py`

**Sim only.** Broadcasts a static `base_link → workspace_frame` TF. On hardware, `workspace_frame` is published dynamically by `workspace_board_node`.

Current sim values (`full_holoassist_sim.yaml`):
```yaml
holoassist_workspace_frame_tf:
  ros__parameters:
    parent_frame: base_link
    child_frame: workspace_frame
    x_m: 0.0       # centred on robot X axis
    y_m: -0.315    # 315 mm in front of robot base
    z_m: 0.02      # 20 mm above base_link origin (board surface height)
    roll_rad: 0.0
    pitch_rad: 0.0
    yaw_rad: 0.0
```

---

## 6. Launch files

### Simulation

```bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

Key sim-specific settings passed to coordinate_listener:
```
require_robot_status: false
require_controller_check: false
trajectory_topic: /joint_trajectory_controller/joint_trajectory
```

### Hardware

```bash
ros2 launch moveit_robot_control full_holoassist_hardware.launch.py robot_ip:=<ip>
```

Key hardware settings:
```
require_robot_status: true
require_controller_check: true
trajectory_topic: /scaled_joint_trajectory_controller/joint_trajectory
```

Startup timing (hardware):
```
t=0s   UR driver, MoveIt, perception (all independent, start together)
t=8s   workspace_scene_manager
t=10s  coordinate_listener + pick_place_sequencer
t=12s  pick_place_service + selected_cube_adapter
t=15s  RViz
```

### Coordinate listener only

```bash
ros2 launch moveit_robot_control coordinate_listener.launch.py
```

All parameters can be overridden as launch arguments.

---

## 7. Verification commands

```bash
# State and status
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/status --once
ros2 topic echo /moveit_robot_control/debug --once

# Send a manual point goal
ros2 topic pub --once /moveit_robot_control/target_point \
  geometry_msgs/msg/Point "{x: 0.30, y: 0.00, z: 0.20}"

# Send a manual pose goal
ros2 topic pub --once /moveit_robot_control/target_pose \
  geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.00, z: 0.20}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}"

# Trigger a pick via service
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"

# Workspace scene markers visible?
ros2 topic echo /workspace_scene/markers --once
```
