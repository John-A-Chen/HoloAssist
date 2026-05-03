# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

HoloAssist is a robotic manipulation system combining AprilTag-based perception with MoveIt motion planning for a UR3e arm with OnRobot RG2 gripper. The `j0hn` branch merges two development streams: John's perception side (camera + AprilTags + workspace reasoning) and Ollie's motion execution side (topic-driven MoveIt planning + execution), and extends both with a pick-place service layer, hardware launch integration, and an XR streaming path via rosbridge.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
```

`libnet1-dev` is a required system dependency for `onrobot_driver`.

## Running the System

### Simulation (primary development entry point)

```bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

Starts: fake-hardware robot + MoveIt, static workspace TF, workspace scene manager, sim perception pipeline (truth cubes + fake camera + visibility sim), cube-to-MoveIt adapter, pick-place sequencer, pick-place service, RViz.

To trigger an automated pick via service:
```bash
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

### Hardware — XR perception only (no robot required)

```bash
ros2 launch holo_assist_depth_tracker holoassist_full_xr.launch.py
```

Starts: RealSense camera, apriltag_ros, workspace_board_node (solves workspace_frame from board tags), cube_pose_node (tracks all 4 cubes), rosbridge WebSocket server (port 9090). Unity/HoloLens connects to `ws://<host>:9090`.

### Hardware — full stack (robot + perception + XR)

```bash
ros2 launch moveit_robot_control full_holoassist_hardware.launch.py robot_ip:=<ip>
```

Starts: UR3e driver + OnRobot gripper, MoveIt (scaled_joint_trajectory_controller), full perception pipeline (camera + board + cubes), workspace scene manager, coordinate listener, pick-place sequencer, pick-place service, rosbridge, RViz.

### Hardware — perception only (legacy)

```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

### Hardware — robot stack (separate terminals, manual)

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

The system has four layers:

### 1. Perception (`holo_assist_depth_tracker`)

- `apriltag_ros` detects tags and publishes `/detections_all` + TF frames
- `holoassist_workspace_board_node` solves and locks `workspace_frame` from board corner tags (IDs 0–3) using SVD/Kabsch algorithm. Also publishes `workspace_frame → ur3e_base_link0` at the known robot position.
- `holoassist_cube_pose_node` estimates cube centers from face tag groups:
  - Cube 1: IDs 10–15, Cube 2: 16–21, Cube 3: 22–27, Cube 4: 28–33
  - Consensus-based center estimation from multiple visible faces
  - Requires `workspace_frame` to be in TF (silently waits until board locks)
- Outputs: `/holoassist/perception/april_cube_[1-4]_pose` (PoseStamped in `workspace_frame`)

### 2. Simulation perception (`holo_assist_depth_tracker_sim`)

- `sim_cube_truth_node`: draggable truth cubes + interactive markers + fake camera TF tree
- `sim_cube_perception_node`: visibility-based synthetic perception, last-seen memory
- `sim_cube_moveit_bridge_node`: publishes perceived cubes to MoveIt planning scene
- `selected_cube_to_moveit_target_node`: TF-transforms selected cube pose into `base_link`, applies hover offsets, publishes MoveIt goal topics
- `pick_place_service_node`: bridges `/holoassist/pick_cube_to_bin` service to pick_place_sequencer command topics

### 3. Adapter layer

The handoff boundary between perception and motion. In both sim and hardware modes:

1. Subscribes to cube pose in `workspace_frame`
2. TF-transforms into `base_link` (MoveIt planning frame)
3. Applies configurable hover offsets
4. Publishes to MoveIt input topics

See `docs/POSE_HANDOFF_CONTRACT.md` for the full contract.

### 4. Motion execution (`moveit_robot_control`)

- `coordinate_listener` (`MoveItCoordinateTopicControl`): receives goals, tries Cartesian path first, falls back to pose-goal planning, executes on UR controllers
- `pick_place_sequencer`: higher-level pick/place state machine — pregrasp → grasp → lift → move to bin → place descent → release. Driven by JSON commands on `/pick_place/command`.
- `workspace_scene_manager`: manages collision objects (trolley mesh) in the MoveIt scene
- `workspace_frame_tf`: broadcasts the static workspace TF in simulation

## Stable Interfaces — Do Not Change Without Updating Adapters

**MoveIt inputs:**
- `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`) — position-only goal
- `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`) — explicit pose goal
- `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`) — legacy format

**MoveIt outputs:**
- `/moveit_robot_control/state` — lifecycle: `QUEUED → PLANNING → PLANNED → EXECUTING → COMPLETE/FAILED`
- `/moveit_robot_control/status` — human-readable string
- `/moveit_robot_control/debug` — JSON payload
- `/moveit_robot_control/complete` — completion marker

**Pick-place service:**
- `/holoassist/pick_cube_to_bin` (`holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`) — fire-and-forget; returns immediately once command is queued

**Pick-place sequencer topics:**
- `/pick_place/command` (`std_msgs/String`) — JSON: `{"block_id": "april_cube_1", "x": x, "y": y, "z": z, "bin_id": "bin_1"}`
- `/pick_place/mode` (`std_msgs/String`) — `"run"` or `"pause"`

**`workspace_frame`** semantics and transform direction must remain stable. All perception publishes in `workspace_frame`; all motion planning uses `base_link`. The adapter/TF bridge is the only place frame conversion should occur.

## Frame Model

| Frame | Purpose |
|---|---|
| `base_link` | MoveIt planning frame |
| `workspace_frame` | Perception frame; solved from board tags on hardware or statically set in sim |
| `ur3e_base_link0` | Robot base position in workspace_frame (published by workspace_board_node on hardware) |
| `camera_color_optical_frame` | RealSense D435i optical center |
| `tag36h11:<id>` | Published by `apriltag_ros` per detected tag |
| `apriltag_cube_[1-4]` | Published by cube pose solver |

Sim workspace_frame: `base_link → workspace_frame = (0.0, -0.315, +0.020)` metres, zero rotation.

Hardware workspace_frame: dynamically solved by workspace_board_node. Origin = board corner (tag 0 centre), +X along board width (700 mm), +Y along board depth (500 mm), +Z up from surface.

## Key Config Files

- `src/holo_assist_depth_tracker/config/` — `apriltag_all.yaml` (all 34 tag IDs), `workspace.yaml` (board geometry + robot position), `cubes.yaml` (cube tag groups + face order)
- `src/moveit_robot_control/config/full_holoassist_sim.yaml` — workspace TF, scene manager, and adapter offsets for the integrated sim
- `src/moveit_robot_control/config/full_holoassist_hw.yaml` — scene manager and adapter offsets for hardware (no workspace TF — solved by board node)
- `src/moveit_robot_control/config/bin_poses.json` — gripper bin locations for pick-place
- `src/moveit_robot_control/config/sim_controllers.yaml` — controller config for fake hardware sim

## Deeper Reference Docs

- `docs/J0HN_MERGED_ARCHITECTURE.md` — top-level integration map and operational modes
- `docs/POSE_HANDOFF_CONTRACT.md` — perception-to-motion interface spec and failure modes
- `src/holo_assist_depth_tracker/docs/PERCEPTION_PIPELINE_REFERENCE.md` — perception internals, camera self-localisation, XR streaming
- `src/moveit_robot_control/docs/MOTION_EXECUTION_REFERENCE.md` — motion planner internals, pick-place sequencer, hardware vs sim params
- `src/moveit_robot_control/INTEGRATION_AND_MERGE.md` — completed integration notes, tuning reference, merge guide
