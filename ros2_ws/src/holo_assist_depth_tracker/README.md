# holo_assist_depth_tracker

Perception package for HoloAssist.

This package provides the real-camera perception path used by the top-level launcher, including AprilTag detection integration, cube pose fusion, workspace diagnostics, and debug image overlays.

## Current Runtime Role

When `./launch.sh --perception` is used, this package provides:

- camera launch (RealSense path)
- webcam fallback publisher (for non-RealSense cameras)
- AprilTag + cube pose pipeline
- depth tracker debug overlay stream
- trolley scene marker publisher

## Launch Files in Active Use

### `camera_only.launch.py`

Starts RealSense camera (`realsense2_camera/rs_launch.py`) with configured stream profiles.

### `holoassist_4tag_board_4cube.launch.py`

Starts:

- AprilTag detector (`apriltag_ros/apriltag_node`)
- workspace board node (optional)
- cube pose node
- AprilTag overlay node
- depth tracker node

Also validates AprilTag size setting in the selected params file.

### `visualize_depth_tracker.launch.py`

Convenience launch that starts `camera_only` + `holoassist_4tag_board_4cube` + RViz profile.

## Camera Paths Used by Top-Level Launcher

Camera priority in `launch.py`:

1. RealSense
2. Logitech Brio
3. Generic UVC webcam
4. Sim fallback (only if fake hardware mode and no camera)

### RealSense path

`launch.py` runs:

```text
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py
```

### Webcam path

`launch.py` runs:

- `holo_assist_webcam_image_publisher` with calibrated/default HFOV parameters
- `holoassist_4tag_board_4cube.launch.py start_camera:=false start_workspace:=false`

Webcam publisher topic defaults are overridden to match downstream expectations:

- image: `/camera/camera/color/image_raw`
- camera info: `/camera/camera/color/camera_info`
- frame: `camera_color_optical_frame`

## Core Nodes

### `holo_assist_depth_tracker_node`

Publishes debug RGB overlay combining:

- AprilTag outlines
- cube wireframes projected from fused 3D cube poses

Primary output:

- `/holo_assist_depth_tracker/debug_image`

### `holoassist_cube_pose_node`

Consumes AprilTag detections and publishes fused cube poses/markers:

- `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
- marker topics per cube

Default cube model:

- edge length `0.040 m`
- face tag size `0.032 m`
- 6 tags per cube, 4 cubes total

### `holoassist_workspace_board_node`

Optional workspace geometry estimator and board-frame diagnostics.

### `holoassist_overlay_node`

Publishes AprilTag overlay image for debugging tag detections.

### `holoassist_trolley_scene_publisher`

Publishes persistent trolley mesh marker to:

- `/holoassist/scene/trolley`

Frame: `world`.

## Main Config Files

- `config/apriltag_all.yaml`: tag family `36h11`, tag size `0.032`, and board/cube tag IDs.
- `config/cubes.yaml`: cube dimensions and tag-to-face mapping.
- `config/tracker_params.yaml`: RGB topic selection, cube pose topic prefix, timeouts, and tag grouping.
- `config/workspace.yaml`: board dimensions and robot placement assumptions.

## Calibration Integration

Top-level `launch.py` reads:

- `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`

If present, it publishes static TF from `base_link` to calibration target frame before perception stack startup.

For webcam path, if calibration target is `camera_link`, launcher additionally publishes standard optical-frame rotation to `camera_color_optical_frame`.

## Build

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holo_assist_depth_tracker --symlink-install
source install/setup.bash
```

## Quick Runtime Checks

```bash
ros2 topic hz /holo_assist_depth_tracker/debug_image
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic list | rg detections
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:10
```

## Common Issues

### No detections

- confirm AprilTag size in `config/apriltag_all.yaml` is `0.032`
- verify camera topics are publishing
- verify lighting and tag visibility

### Cube pose jitter

- check `candidate_consensus_threshold_m` in `config/cubes.yaml`
- confirm tag stickers match configured face-order mapping

### Webcam stream starts then dies

`holo_assist_webcam_image_publisher` exits if repeated capture failures occur (intentional, so launcher can surface camera loss). Check cable/device index and retry.
