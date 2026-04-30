# holo_assist_depth_tracker

Deterministic AprilTag-first perception stack for HoloAssist workspace tracking.

## 1) System Overview

This package now provides a deterministic dual-detector AprilTag pipeline:

- Board detector tracks workspace board tags `0..3` (size `0.15 m`) on `/detections_board`
- Cube detector tracks cube tags `10..33` (size `0.045 m`) on `/detections_cubes`
- Merge node outputs deduplicated union on `/detections_all`
- Workspace node locks `workspace_frame` from the first valid full 4-tag board solve
- Cube node publishes stable poses/TF/markers/status for 4 cubes
- Overlay and depth tracker use merged detections (`/detections_all`)

Legacy nodes/launch files are kept for backward compatibility.

## 2) Architecture Diagram

```text
RGB Image + CameraInfo
        |
        +---------------------------+
        |                           |
  apriltag_board               apriltag_cubes
(ids 0..3, 0.15m)           (ids 10..33, 0.045m)
        |                           |
 /detections_board           /detections_cubes
        +-----------+   +-----------+
                    |   |
            detection_merge_node
                    |
              /detections_all
               /      |      \
              /       |       \
 overlay_node   workspace_board_node   cube_pose_node
      |              |      \             |
 overlay image   workspace TF  robot TF   cube TF/pose/marker/status (x4)
                                 \
                             ur3e_base_link0
```

## 3) Node Descriptions

### `holoassist_detection_merge_node`
- Inputs: `/detections_board`, `/detections_cubes`
- Output: `/detections_all`
- Behavior:
  - Union by AprilTag ID
  - Per-ID latest timestamp wins
  - Removes duplicates
  - Prunes stale IDs by timeout

### `holoassist_workspace_board_node`
- Input: `/detections_board` + TF `tag36h11:0..3`
- Output:
  - TF `<camera_frame> -> workspace_frame`
  - TF `workspace_frame -> ur3e_base_link0` at `(0.450, 0.564, 0.0)`
  - `/holoassist/perception/ur3e_base_link0_pose`
  - `/holoassist/perception/ur3e_base_link0_marker`
  - `/holoassist/perception/workspace_mode`
  - `/holoassist/perception/workspace_diagnostics`
  - `/holoassist/perception/bench_plane_coefficients`
- Behavior:
  - Uses only board tags `0,1,2,3`
  - Enforces board geometry (0.700 x 0.500)
  - Locks on first valid full board solve
  - No drift updates after lock
  - `realign_workspace` service clears lock

### `holoassist_cube_pose_node`
- Input: `/detections_all` + TF `tag36h11:<id>`
- Cube groups:
  - Cube 1: `10..15`
  - Cube 2: `16..21`
  - Cube 3: `22..27`
  - Cube 4: `28..33`
- Output per cube `n=1..4`:
  - TF `workspace_frame -> apriltag_cube_<n>`
  - `/holoassist/perception/april_cube_<n>_pose`
  - `/holoassist/perception/april_cube_<n>_marker`
  - `/holoassist/perception/april_cube_<n>_status`
- Additional legacy alias:
  - `/holoassist/perception/april_cube_pose` (Cube 1)
- Behavior:
  - Sequential face-axis mapping per group: `+X,-X,+Y,-Y,+Z,-Z`
  - Robust to partial visibility
  - Stable orientation via sign continuity + multi-tag rotation fit
  - No obstacle-only fallback

### `holoassist_overlay_node`
- Input: RGB image + `/detections_all`
- Output: `/holoassist/perception/apriltag_overlay`
- Behavior:
  - Draws all merged detections
  - Footer text: `tags=<count> age=<seconds>`

### Existing `depth_tracker_node` (preserved)
- Updated defaults for AprilTag integration:
  - `apriltag_topic=/detections_all`
  - Tracking IDs default to `10..33` (cube tags only)

## 4) Topics

### Detector/Merge
- `/detections_board` (`apriltag_msgs/AprilTagDetectionArray`)
- `/detections_cubes` (`apriltag_msgs/AprilTagDetectionArray`)
- `/detections_all` (`apriltag_msgs/AprilTagDetectionArray`)

### Workspace/Robot
- `/holoassist/perception/workspace_mode` (`std_msgs/String`)
- `/holoassist/perception/workspace_diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- `/holoassist/perception/bench_plane_coefficients` (`std_msgs/Float32MultiArray`)
- `/holoassist/perception/ur3e_base_link0_pose` (`geometry_msgs/PoseStamped`)
- `/holoassist/perception/ur3e_base_link0_marker` (`visualization_msgs/Marker`)

### Cubes
- `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
- `/holoassist/perception/april_cube_1_marker` ... `_4_marker`
- `/holoassist/perception/april_cube_1_status` ... `_4_status`
- Legacy alias: `/holoassist/perception/april_cube_pose`

### Overlay
- `/holoassist/perception/apriltag_overlay` (`sensor_msgs/Image`)

## 5) TF Tree

```text
<camera_frame>
├── tag36h11:0
├── tag36h11:1
├── tag36h11:2
├── tag36h11:3
├── tag36h11:10 ... tag36h11:33
└── workspace_frame
    ├── ur3e_base_link0
    ├── apriltag_cube_1
    ├── apriltag_cube_2
    ├── apriltag_cube_3
    └── apriltag_cube_4
```

## 6) Launch Instructions

### Primary new launch

```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

Useful arguments:
- `start_camera:=false|true`
- `image_topic:=/camera/camera/color/image_raw`
- `camera_info_topic:=/camera/camera/color/camera_info`
- `start_tracker:=true|false`
- `start_overlay:=true|false`
- `board_params_file:=.../apriltag_board.yaml`
- `cubes_params_file:=.../apriltag_cubes.yaml`
- `workspace_params_file:=.../workspace.yaml`
- `cube_pose_params_file:=.../cubes.yaml`

### Realign workspace lock

```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```

### Backward-compatible launch files
Legacy launch files under `launch/` are preserved and still usable.

## 7) Parameter Highlights

### `config/apriltag_board.yaml`
- Board detector family/size/IDs (`0..3`, `0.15m`)

### `config/apriltag_cubes.yaml`
- Cube detector family/size/IDs (`10..33`, `0.045m`)

### `config/workspace.yaml`
- Workspace frame name and board tag IDs
- Lock/geometry tolerances
- Realign service name
- Fixed robot pose in workspace coordinates

### `config/cubes.yaml`
- Per-cube tag groups
- Cube size
- Timeout/TF lookup/timer rates
- Legacy alias topic

## 8) Testing Procedure

1. Build and source:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
colcon build --packages-select holo_assist_depth_tracker holoassist_foxglove
source install/setup.bash
```

2. Run launch:
```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

3. Verify detection streams:
```bash
ros2 topic hz /detections_board
ros2 topic hz /detections_cubes
ros2 topic echo /detections_all --once
```

4. Verify workspace lock and diagnostics:
```bash
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
```

5. Verify TF:
```bash
ros2 run tf2_ros tf2_echo workspace_frame ur3e_base_link0
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1
```

6. Verify overlay and cube outputs:
```bash
ros2 topic echo /holoassist/perception/april_cube_1_status --once
ros2 topic hz /holoassist/perception/apriltag_overlay
```

## 9) Failure Cases and Behavior

- Missing/stale board detections:
  - Workspace status publishes `INVALID`
  - No fallback to alternate geometry
- Insufficient board tags (`<4`):
  - Status `INVALID`
  - Existing lock (if any) is retained until realign
- Invalid board geometry (dimension/residual tolerance fail):
  - Status `INVALID`
  - No lock acquisition
- Cube tags partially visible:
  - Cube pose still solved from available tags where possible
- Cube has no valid tag transforms:
  - Cube status indicates failure and marker is deleted

## 10) Integration Notes

### RViz
- Set fixed frame to `workspace_frame` for board-relative visualization.
- Display TF, cube poses/markers, robot marker, and overlay image topic.

### Foxglove
- Subscribe to:
  - `/detections_all`
  - `/holoassist/perception/workspace_diagnostics`
  - `/holoassist/perception/april_cube_*_pose`
  - `/holoassist/perception/april_cube_*_status`
- Optional stack integration is available in `holoassist_stack.launch.py` via:
  - `enable_4tag_board_4cube_pipeline:=true`

### Unity
- Consume TF for:
  - `workspace_frame`
  - `ur3e_base_link0`
  - `apriltag_cube_1..4`
- The workspace frame is deterministic once locked; call realign service when board setup changes.
