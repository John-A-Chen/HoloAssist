# Motion Execution Reference (`moveit_robot_control`)

This document explains the `ollie`-lineage motion side used in the merged `j0hn` branch.

It focuses on:

- how goal topics are consumed,
- how planning/execution decisions are made,
- what outputs/diagnostics are available for debugging.

## 1. Main execution node

Executable:

- `coordinate_listener`

Implementation file:

- `moveit_robot_control_node/moveit_robot_control.py`

Primary class:

- `MoveItCoordinateTopicControl`

## 2. Goal input interfaces

Default subscribed topics:

1. `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`)
2. `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`)
3. `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`, enabled only if msg package is available)

Queue behavior:

- Inputs are validated and queued (`pending_coordinates` deque).
- Goals are processed one-at-a-time.

## 3. Planning strategy

For each queued goal, listener attempts:

1. Determine orientation candidates:
   - explicit pose/rpy orientation from message, or
   - `orientation_mode` policy (`auto`, `current`, `fixed`)
2. Try Cartesian planning to target with each candidate orientation.
3. If Cartesian fails and fallback is enabled:
   - use general MoveIt pose-goal planning.
4. In `auto` mode only, if sampled orientations fail:
   - final free-orientation position-only planning attempt.

Default notable planner parameters:

- `cartesian_max_step = 0.005`
- `auto_waypoint_distance = 0.10`
- `cartesian_jump_threshold = 5.0`
- `min_cartesian_fraction = 0.999`
- `pose_goal_planning_time = 5.0`
- `pose_goal_position_tolerance = 0.005`
- `pose_goal_orientation_tolerance = 0.05`

## 4. Safety and validity gates

Before execution, the node includes:

1. MoveIt state validity/collision checks along trajectory.
2. Floor collision object injection into planning scene.
3. Optional UR-specific flange-to-forearm clamp risk rejection:
   - uses URDF from `move_group` parameter service,
   - predicts clearance and rejects unsafe paths.
4. Robot status checks (can be disabled for fake hardware):
   - program running, robot mode, safety mode, active scaled controller.

## 5. Execution behavior

Execution topic:

- `/scaled_joint_trajectory_controller/joint_trajectory`

Node publishes selected trajectory and waits for final joint convergence using configurable timeout scaling:

- `execution_timeout_scale` (default 20.0)
- `execution_timeout_padding` (default 5.0)
- `joint_goal_tolerance` (default 0.1 rad)

## 6. Status and telemetry outputs

Human-readable output:

- `/moveit_robot_control/status` (`std_msgs/String`)

State machine output:

- `/moveit_robot_control/state` (`std_msgs/String`, transient local)

Detailed structured debug:

- `/moveit_robot_control/debug` (`std_msgs/String`, JSON payload)

Completion marker:

- `/moveit_robot_control/complete` (`std_msgs/String`)

Typical successful lifecycle:

1. `QUEUED`
2. `PLANNING`
3. `PLANNED`
4. `EXECUTING`
5. `COMPLETE`

Failure lifecycle states include `INVALID` or `FAILED` with reason in debug payload.

## 7. Supporting nodes in this package

## `workspace_frame_tf`

File:

- `moveit_robot_control_node/workspace_frame_tf.py`

Role:

- Publishes static `parent_frame -> workspace_frame` transform.
- Used heavily in full sim bringup.

## `workspace_scene_manager`

File:

- `moveit_robot_control_node/workspace_scene_manager.py`

Role:

- Publishes table mesh marker(s) for RViz.
- Optionally applies table/block collision objects into MoveIt scene.
- Accepts JSON scene commands and PoseStamped block spawns.

## `pick_place_sequencer`

File:

- `moveit_robot_control_node/pick_place_sequencer.py`

Role:

- Higher-level pick/place workflow that publishes motion goals into listener topics.

## 8. Launch files and when to use them

Single node:

- `launch/coordinate_listener.launch.py`

Pick-place bundle:

- `launch/pick_place_system.launch.py`

Full integrated sim:

- `launch/full_holoassist_moveit_sim.launch.py`

The full integrated sim launch also starts perception simulation and the selected-cube adapter that publishes directly into listener input topics.

## 9. Integration contract with perception side

Perception-side code should treat this node as a topic API:

Inputs expected:

- `Point` or `Pose` targets in listener planning frame (`base_link` unless changed).

Outputs provided:

- status/state/debug/complete topics for orchestration and observability.

See shared handoff contract:

- `ros2_ws/docs/POSE_HANDOFF_CONTRACT.md`

## 10. Fast verification commands

```bash
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/status --once
ros2 topic echo /moveit_robot_control/debug --once
```

Send a point goal:

```bash
ros2 topic pub --once /moveit_robot_control/target_point geometry_msgs/msg/Point "{x: 0.30, y: 0.00, z: 0.20}"
```

Send a pose goal:

```bash
ros2 topic pub --once /moveit_robot_control/target_pose geometry_msgs/msg/Pose "{position: {x: 0.30, y: 0.00, z: 0.20}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}"
```
