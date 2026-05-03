# j0hn Branch Merged Architecture Guide

This document explains how the merged `j0hn` branch combines:

- the `john` perception side (camera + AprilTags + workspace reasoning), and
- the `ollie` motion side (topic-driven MoveIt planning + execution).

It is intended as the top-level map before drilling into package-level docs.

## 1. System split by responsibility

### Perception (john lineage)

Primary package:

- `ros2_ws/src/holo_assist_depth_tracker`

Key responsibilities:

- Detect AprilTags and publish `/detections_all`.
- Solve/lock `workspace_frame` from board tags.
- Estimate cube poses from AprilTag face groups.
- Optionally produce depth-derived workspace/object reasoning.

Important nodes:

- `holoassist_workspace_board_node`
- `holoassist_cube_pose_node`
- `holoassist_workspace_perception` (from `workspace_perception_node.py`)
- `holoassist_detection_merge_node` (if two detection streams must be merged)

### Motion execution (ollie lineage)

Primary package:

- `ros2_ws/src/moveit_robot_control`

Key responsibilities:

- Listen for incoming point/pose goals on topic APIs.
- Plan Cartesian paths with MoveIt first.
- Fall back to pose-goal planning when needed.
- Execute selected trajectories on UR controllers.
- Publish motion lifecycle/status/debug streams.

Important nodes:

- `coordinate_listener` (`MoveItCoordinateTopicControl`)
- `workspace_scene_manager`
- `workspace_frame_tf`

### Adapter layer (handoff between perception and motion)

Current reference adapter:

- `ros2_ws/src/holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/selected_cube_to_moveit_target_node.py`

This node transforms selected cube poses into:

- `/moveit_robot_control/target_point` (`geometry_msgs/Point`)
- `/moveit_robot_control/target_pose` (`geometry_msgs/Pose`)
- optional legacy mirror: `/moveit_robot_control/target`

Even though this adapter currently lives in the sim package, it defines the handoff contract used by the merged architecture.

## 2. End-to-end topic flow (merged behavior)

### AprilTag + workspace pipeline

1. Camera stream feeds `apriltag_ros/apriltag_node`.
2. `apriltag_node` publishes `/detections_all` and tag TF frames.
3. `holoassist_workspace_board_node` solves and locks `workspace_frame`.
4. `holoassist_cube_pose_node` publishes cube center poses:
   - `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
   - legacy aggregate: `/holoassist/perception/april_cube_pose`

### Pose handoff pipeline

1. A selector/source chooses a cube (or object) pose.
2. Adapter converts pose into robot planning frame (`base_link` by default).
3. Adapter publishes MoveIt command topics:
   - `/moveit_robot_control/target_point`
   - `/moveit_robot_control/target_pose`

### Motion planning pipeline

1. `coordinate_listener` receives `Point` or `Pose` goal.
2. Listener validates values and queues the goal.
3. Planner tries Cartesian path first.
4. If Cartesian fails and fallback is enabled, listener uses pose-goal planning.
5. Trajectory is collision-validated and then executed.
6. Node publishes status/state/debug and `/moveit_robot_control/complete`.

## 3. Core frame model

### Planning frame

- `base_link` is the default MoveIt planning frame.

### Workspace frame

- `workspace_frame` is solved from AprilTag board geometry or statically set (sim bringup).
- Static default in full sim config:
  - `base_link -> workspace_frame = (-0.100, -0.314, +0.015)` meters, zero rotation.

### Why this matters

- Perception often works naturally in `workspace_frame`.
- MoveIt goal execution expects a planning frame (`base_link`).
- The adapter/TF bridge is the strict boundary where frame conversion must be correct.

## 4. Recommended launch entry points

### Full merged simulation stack

- `ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py`

Starts:

- MoveIt bringup
- workspace static TF publisher
- workspace scene manager
- coordinate listener
- perception simulation stack
- selected-cube to MoveIt target adapter

### Hardware-oriented AprilTag stack

- `ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py`

Starts:

- AprilTag detection pipeline
- workspace board solver/locker
- cube pose solver
- optional overlay + tracker

## 5. Interfaces that should remain stable

For a safe merge and future refactors, keep these interfaces stable:

- `workspace_frame` semantics and transform direction.
- MoveIt input topics:
  - `/moveit_robot_control/target_point`
  - `/moveit_robot_control/target_pose`
  - optional `/moveit_robot_control/target` (legacy `TargetRPY`)
- Completion/status outputs:
  - `/moveit_robot_control/complete`
  - `/moveit_robot_control/status`
  - `/moveit_robot_control/state`
  - `/moveit_robot_control/debug`

If any of these are changed, update adapter nodes and launch defaults together.

## 6. Deeper docs

- Perception internals:
  - `ros2_ws/src/holo_assist_depth_tracker/docs/PERCEPTION_PIPELINE_REFERENCE.md`
- Motion internals:
  - `ros2_ws/src/moveit_robot_control/docs/MOTION_EXECUTION_REFERENCE.md`
- Handoff contract:
  - `ros2_ws/docs/POSE_HANDOFF_CONTRACT.md`
