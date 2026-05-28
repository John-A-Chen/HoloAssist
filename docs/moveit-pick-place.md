# MoveIt Pick-and-Place — Reference

**Package:** `holoassist_movement`  
**Entrypoint:** `./scripts/launch.sh --robot-ip <IP> --perception --moveit`

---

## Architecture

```
/holoassist/pick_cube_to_bin (PickCubeToBin service)
          │  pick_place_service_node
          ▼
   pick_place_sequencer  ←→  /pick_place/status
          │
   coordinate_listener (Cartesian + pose-goal fallback)
          │
   MoveIt move_group
          │
   scaled_joint_trajectory_controller
          │
   UR3e + OnRobot RG2
```

---

## Trigger a Pick

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

Accepts short forms: `cube_name: '1'` and `bin_id: '1'` are normalized automatically.

---

## State Machine Sequence

```
PREGRASP   → move to hover above cube (pregrasp_z_offset above cube Z)
GRASP      → gripper open → descend → gripper close
LIFT       → ascend back to hover (cube attached in planning scene)
MOVE_TO_BIN → joint-space move above bin → Cartesian descent
PLACE      → gripper open → cube released
RETREAT    → ascend to safe pose
```

Cartesian path attempted first; falls back to MoveIt pose-goal (OMPL) on failure.

---

## Key Parameters (`full_holoassist_hardware.launch.py`)

| Param | Default | Effect |
|---|---|---|
| `pregrasp_z_offset` | `0.10 m` | Hover height above cube before descent |
| `grasp_z_offset` | `0.0 m` | Additional Z offset at grasp pose |
| `grasp_z_absolute` | `0.05 m` | Absolute Z for pick descent (use `-1.0` for offset mode) |
| `place_above_z_offset` | `0.15 m` | Hover height above bin |
| `place_z_offset` | `0.05 m` | Z height when releasing |
| `velocity_scale` | `0.05` | MoveIt velocity scaling (0–1) |
| `pose_goal_planning_time` | `5.0 s` | OMPL planning time budget |
| `cube_pose_topic_prefix` | `/holoassist/perception` | Prefix for `april_cube_N_pose` topics |

---

## Bin Poses (`config/bin_poses.json`)

All positions in `base_link` frame, gripper pointing down (`rpy_deg: [180, 0, 0]`):

| Bin | X | Y | Z |
|---|---|---|---|
| `bin_1` | 0.30 | 0.00 | 0.05 |
| `bin_2` | -0.30 | 0.00 | 0.05 |
| `bin_3` | 0.30 | -0.20 | 0.05 |
| `bin_4` | 0.30 | -0.10 | 0.05 |

To add a bin: edit `config/bin_poses.json` and relaunch — no code changes needed.

---

## Workspace Scene (`full_holoassist_hw.yaml`)

The `workspace_scene_manager` adds the trolley mesh to the MoveIt planning scene:

| Setting | Value |
|---|---|
| Mesh | `meshes/UR3eTrolley_decimated.dae` |
| Position | `[0.031, -0.210, -1.05]` (in `base_link`) |
| Scale | `[1.0, 1.0, 1.0]` |

---

## ROS Topics

**Monitor:**
```bash
ros2 topic echo /holoassist/movement/state    # lifecycle state
ros2 topic echo /pick_place/status             # step-level progress
ros2 topic echo /holoassist/movement/debug    # JSON planning details
```

**Manual target (no service):**
```bash
# Point only
ros2 topic pub --once /holoassist/movement/target_point \
  geometry_msgs/msg/Point "{x: 0.20, y: -0.30, z: 0.15}"

# Full pose
ros2 topic pub --once /holoassist/movement/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.20, y: -0.30, z: 0.025}, orientation: {w: 1.0}}}"
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Service returns `success: false` immediately | No cube pose on topic | Check `--perception` is running, cube visible |
| State stuck at PLANNING | Goal unreachable or in collision | Open RViz, inspect planning scene; increase `pose_goal_planning_time` |
| Robot plans but doesn't move | Controller inactive or UR program not running | `ros2 control list_controllers`; check teach pendant |
| Gripper closes but cube falls | Grasp too high or `close_width` too loose | Reduce `grasp_z_absolute`; check cube pose accuracy |
| Wrong bin / wrong location | `bin_poses.json` wrong frame or values | Verify poses in RViz against physical bin locations |
