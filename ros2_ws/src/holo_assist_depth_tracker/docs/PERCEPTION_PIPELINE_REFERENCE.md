# Perception Pipeline Reference (`holo_assist_depth_tracker`)

This document explains the perception-side runtime in the `j0hn` merged branch.

It focuses on:

- AprilTag board/cube solving,
- workspace frame ownership,
- object pose outputs that downstream motion code consumes.

## 1. Main runtime modes

### Mode A: Board + cube AprilTag pipeline (most deterministic)

Launch:

- `ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py`

Primary nodes:

- `apriltag_ros/apriltag_node` -> `/detections_all`
- `holoassist_workspace_board_node`
- `holoassist_cube_pose_node`
- optional: `holoassist_overlay_node`, `holo_assist_depth_tracker_node`

### Mode B: Workspace perception from depth pointcloud + tags

Launch:

- `ros2 launch holo_assist_depth_tracker workspace_perception.launch.py`

Primary node:

- `holoassist_workspace_perception` (`workspace_perception_node.py`)

This mode can publish object poses from:

- pointcloud clusters (`object_source_mode=pointcloud`), or
- AprilTag object pose feed (`object_source_mode=apriltag`).

## 2. Key nodes and responsibilities

## `holoassist_workspace_board_node`

File:

- `holo_assist_depth_tracker/nodes/workspace_board_node.py`

Role:

- Solves `workspace_frame` from fixed board tag layout.
- Locks frame after first valid full-board solve.
- Re-publishes locked transform until explicit realign.
- Publishes workspace diagnostics + plane coefficients.
- Optionally publishes robot base pose marker/TF in workspace frame.

Config file:

- `config/workspace.yaml`

Important params:

- `board_tag_ids` (default `[0,1,2,3]`)
- `board_width_m` (0.700), `board_depth_m` (0.500)
- `board_tag_center_edge_offset_m` (0.016)
- `workspace_realign_service_name` (`/holoassist/perception/realign_workspace`)

## `holoassist_cube_pose_node`

File:

- `holo_assist_depth_tracker/nodes/cube_pose_node.py`

Role:

- Maps tag IDs into 4 physical cube groups.
- Converts visible face-tag detections into cube center candidates.
- Fuses candidates into stable cube center pose per cube.
- Publishes per-cube pose/marker/status topics.
- Publishes legacy aggregate pose topic for compatibility.

Config file:

- `config/cubes.yaml`

Important params:

- `april_cube_1_tag_ids` ... `april_cube_4_tag_ids`
- `april_cube_face_order` (default `["+X","-X","+Y","-Y","+Z","-Z"]`)
- `cube_edge_size_m` (0.040)
- `cube_tag_size_m` (0.032)
- `cube_face_offset_m` (0.020)
- `legacy_cube_pose_topic` (`/holoassist/perception/april_cube_pose`)

## `holoassist_workspace_perception`

File:

- `holo_assist_depth_tracker/workspace_perception_node.py`

Role:

- Consumes depth pointcloud.
- Derives workspace plane + basis from tags and/or plane fit.
- Publishes `workspace_frame` TF and debug markers/clouds.
- Publishes object pose in camera frame and workspace frame.
- Optional AprilTag object mode consumes `/holoassist/perception/april_cube_pose`.

Config file:

- `config/workspace_perception_params.yaml`

Important params for merged flow:

- `object_source_mode: apriltag`
- `apriltag_object_pose_topic: /holoassist/perception/april_cube_pose`
- `workspace_frame: workspace_frame`
- `lock_workspace_after_initial_two_tag_snap: true`
- `workspace_realign_service_name: /holoassist/perception/realign_workspace`

## `holoassist_detection_merge_node`

File:

- `holo_assist_depth_tracker/nodes/detection_merge_node.py`

Role:

- Merges two detection streams by tag ID freshness into one `/detections_all`.

Use only when multiple detector feeds exist.

## 3. Core outputs for downstream control

The most important outputs for motion-side integration are:

1. `workspace_frame` TF (board-referenced workspace geometry).
2. cube/object pose topics (typically `PoseStamped`):
   - `/holoassist/perception/april_cube_pose` (legacy single-topic feed)
   - `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
3. diagnostics:
   - `/holoassist/perception/workspace_mode`
   - `/holoassist/perception/workspace_diagnostics`

These outputs are intended to feed an adapter that publishes MoveIt goal topics.

## 4. Frame and geometry assumptions

The default model assumes:

- AprilTag printed size: `0.032 m`
- board size: `0.700 m x 0.500 m`
- board corner tags positioned by center-to-edge offset `0.016 m`
- cube body edge: `0.040 m`

If these physical assumptions drift from reality, workspace and cube poses will drift.

## 5. Integration boundary with MoveIt

This package does not directly command `moveit_robot_control/coordinate_listener`.

Handoff should happen through a dedicated adapter that:

1. transforms `PoseStamped` from `workspace_frame` to `base_link`,
2. applies any hover/pre-grasp offsets, and
3. publishes `/moveit_robot_control/target_point` and/or `/moveit_robot_control/target_pose`.

Reference adapter implementation:

- `holo_assist_depth_tracker_sim/selected_cube_to_moveit_target_node.py`

## 6. Operational checks

Detection and workspace:

```bash
ros2 topic echo /detections_all --once
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
```

Cube poses:

```bash
ros2 topic echo /holoassist/perception/april_cube_pose --once
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
```

Frame checks:

```bash
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1
ros2 run tf2_ros tf2_echo base_link workspace_frame
```

Workspace re-lock:

```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```
