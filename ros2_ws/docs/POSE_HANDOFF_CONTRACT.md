# Pose Handoff Contract (Perception -> MoveIt)

This file documents the exact interface between perception outputs and the MoveIt coordinate listener.

Use it as the source of truth when wiring `john`-side pose producers into `ollie`-side motion execution.

## 1. Consumer expectations (MoveIt side)

The coordinate listener (`moveit_robot_control/coordinate_listener`) accepts three goal inputs:

1. `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`)
2. `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`)
3. `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`, optional legacy path)

Default planning frame parameter:

- `frame: base_link`

Default end-effector parameter:

- `ee_link: tool0` (or launch override such as `gripper_tcp`)

## 2. Producer expectations (Perception/adapter side)

A pose producer must either:

- publish directly to `/moveit_robot_control/target_pose` and/or `/moveit_robot_control/target_point`, or
- publish to an intermediate topic consumed by an adapter node that republishes into those topics.

Mandatory rules:

1. Position values must be finite real numbers.
2. Pose orientation quaternion must be finite and non-zero magnitude.
3. Frame conversion to listener planning frame must be done before publish (or guaranteed by listener input frame assumptions).

## 3. Recommended handoff pattern

### Preferred command type

- Use `/moveit_robot_control/target_pose` when orientation should be explicit.
- Use `/moveit_robot_control/target_point` when orientation can be selected by listener policy (`auto/current/fixed`).

### Adapter reference

Reference implementation:

- `holo_assist_depth_tracker_sim/selected_cube_to_moveit_target_node.py`

Behavior:

1. Subscribes to selected object pose (`PoseStamped`).
2. TF-transforms position into `target_frame` (`base_link` by default).
3. Applies configurable XYZ hover offsets.
4. Publishes:
   - point goal (`Point`)
   - pose goal (`Pose`) with configured fixed orientation
5. Throttles republish by minimum time and minimum movement delta.

## 4. Topic/frame conventions in this branch

Common perception frame:

- `workspace_frame`

Common motion frame:

- `base_link`

Default object pose topic from April cube stack:

- `/holoassist/perception/april_cube_pose` (`PoseStamped`, frame usually `workspace_frame`)

Typical conversion path:

1. Read object pose in `workspace_frame`.
2. Lookup TF `base_link <- workspace_frame`.
3. Transform position into `base_link`.
4. Add command offsets (for hover/pre-grasp).
5. Publish MoveIt goal topic.

## 5. Listener feedback channels (for integration health checks)

The listener publishes:

- `/moveit_robot_control/status` (`std_msgs/String`, human-readable text)
- `/moveit_robot_control/state` (`std_msgs/String`, lifecycle state)
- `/moveit_robot_control/debug` (`std_msgs/String`, JSON facts)
- `/moveit_robot_control/complete` (`std_msgs/String`, completion marker)

Expected flow per accepted goal:

1. `QUEUED`
2. `PLANNING`
3. `PLANNED`
4. `EXECUTING`
5. `COMPLETE` or `FAILED`/`INVALID`

## 6. Failure modes to guard against

1. Missing TF between producer frame and `base_link`.
2. Publishing stale or invalid quaternions.
3. Orientation mismatch causing unreachable wrist poses.
4. Offsets not matching physical setup (hover below table or too far from object).
5. Very high publish rates causing unnecessary replanning churn.

## 7. Minimal verification commands

```bash
ros2 topic echo /moveit_robot_control/target_point --once
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/debug --once
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo base_link workspace_frame
ros2 run tf2_ros tf2_echo workspace_frame base_link
```
