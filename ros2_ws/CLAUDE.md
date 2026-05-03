# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

HoloAssist is a robotic manipulation system combining AprilTag-based perception with MoveIt motion planning for a UR3e arm with OnRobot gripper. The `j0hn` branch merges two development streams: John's perception side and Ollie's motion execution side.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
```

`libnet1-dev` is a required system dependency for `onrobot_driver`.

## Running the System

**Full integrated simulation (primary entry point):**
```bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```
Starts MoveIt, workspace scene, coordinate listener, perception sim, and the cube-to-target adapter in one RViz session.

**Hardware AprilTag pipeline:**
```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

**Simulation perception only:**
```bash
ros2 launch holo_assist_depth_tracker_sim sim_april_cube_moveit.launch.py
```

**Hardware robot stack (separate terminals):**
```bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<ip>
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
ros2 launch moveit_robot_control coordinate_listener.launch.py
ros2 launch moveit_robot_control pick_place_system.launch.py
```

## Tests

```bash
pytest src/holo_assist_depth_tracker/test/test_perception_algorithms.py
```

## Architecture

The system has three layers:

### 1. Perception (`holo_assist_depth_tracker`)
- `apriltag_ros` detects tags and publishes `/detections_all` + TF frames
- `holoassist_workspace_board_node` solves and locks `workspace_frame` from board corner tags (IDs 0–3)
- `holoassist_cube_pose_node` estimates cube centers from face tag groups:
  - Cube 1: IDs 10–15, Cube 2: 16–21, Cube 3: 22–27, Cube 4: 28–33
- Outputs: `/holoassist/perception/april_cube_[1-4]_pose` (PoseStamped in `workspace_frame`)

### 2. Adapter layer (`selected_cube_to_moveit_target_node.py` in sim package)
The handoff boundary between perception and motion — see `docs/POSE_HANDOFF_CONTRACT.md` for full contract. In brief:
1. Subscribes to selected cube pose in `workspace_frame`
2. TF-transforms into `base_link` (the MoveIt planning frame)
3. Applies configurable hover offsets
4. Throttles by minimum time/movement delta to avoid replanning churn
5. Publishes to MoveIt input topics

### 3. Motion execution (`moveit_robot_control`)
- `coordinate_listener` (`MoveItCoordinateTopicControl`): receives goals, tries Cartesian path first, falls back to pose-goal planning, executes on UR controllers
- `workspace_scene_manager`: manages collision objects in the MoveIt scene
- `workspace_frame_tf`: broadcasts the static workspace TF in simulation

## Stable Interfaces — Do Not Change Without Updating Adapters

**MoveIt inputs:**
- `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`) — position-only goal; listener selects orientation by policy
- `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`) — explicit pose goal
- `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`) — legacy format

**MoveIt outputs:**
- `/moveit_robot_control/state` — lifecycle: `QUEUED → PLANNING → PLANNED → EXECUTING → COMPLETE/FAILED`
- `/moveit_robot_control/status` — human-readable string
- `/moveit_robot_control/debug` — JSON payload
- `/moveit_robot_control/complete` — completion marker

**`workspace_frame`** semantics and transform direction must remain stable. All perception publishes in `workspace_frame`; all motion planning uses `base_link`. The adapter/TF bridge is the only place frame conversion should occur.

## Frame Model

| Frame | Purpose |
|---|---|
| `base_link` | MoveIt planning frame |
| `workspace_frame` | Perception frame; solved from board tags or statically set in sim |
| `camera_color_optical_frame` | RealSense D435i optical center |
| `tag36h11:<id>` | Published by `apriltag_ros` per detected tag |
| `apriltag_cube_[1-4]` | Published by cube pose solver |

Static sim default: `base_link → workspace_frame = (-0.100, -0.314, +0.015)` meters, zero rotation.

## Key Config Files

- `src/holo_assist_depth_tracker/config/` — `apriltag_all.yaml`, `workspace.yaml`, `cubes.yaml`, `tracker_params.yaml`
- `src/moveit_robot_control/config/full_holoassist_sim.yaml` — workspace TF, scene manager, and adapter offsets for the integrated sim
- `src/moveit_robot_control/config/bin_poses.json` — gripper bin locations for pick-place

## Deeper Reference Docs

- `docs/J0HN_MERGED_ARCHITECTURE.md` — top-level integration map
- `docs/POSE_HANDOFF_CONTRACT.md` — perception-to-motion interface spec and failure modes
- `src/holo_assist_depth_tracker/docs/PERCEPTION_PIPELINE_REFERENCE.md` — perception internals
- `src/moveit_robot_control/docs/MOTION_EXECUTION_REFERENCE.md` — motion planner internals
- `src/moveit_robot_control/INTEGRATION_AND_MERGE.md` — merge guide with tuning notes
