# holo_assist_depth_tracker

AprilTag-first perception stack for HoloAssist workspace tracking.

## Overview

This package uses one AprilTag detector configuration and one tag family/size.
Tag IDs define semantic roles:
- Workspace board corners: `0..3`
- Cube faces: `10..33`

AprilTag physical size standard in this repo:
- `AprilTag printed edge size = 0.032 m` for all tags

Primary detection stream:
- `/detections_all`

Core nodes:
- `holoassist_workspace_board_node`: solves/locks `workspace_frame` from workspace tag IDs
- `holoassist_cube_pose_node`: solves per-cube pose from configured cube ID groups
- `holoassist_overlay_node`: draws tag overlays from `/detections_all`
- `holo_assist_depth_tracker_node`: depth + debug overlay integration

## Pipeline

```text
RGB Image + CameraInfo
        |
    apriltag_node
 (single config + ID list)
        |
  /detections_all
     /   |    \
    /    |     \
overlay  workspace  cube_pose
```

## Launch

Primary launch:

```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

Useful arguments:
- `start_camera:=false|true`
- `image_topic:=/camera/camera/color/image_raw`
- `camera_info_topic:=/camera/camera/color/camera_info`
- `start_tracker:=true|false`
- `start_overlay:=true|false`
- `apriltag_params_file:=.../apriltag_all.yaml`
- `workspace_params_file:=.../workspace.yaml`
- `cube_pose_params_file:=.../cubes.yaml`

## Key Config Files

- `config/apriltag_all.yaml`: single detector config with all tracked IDs
- `config/workspace.yaml`: workspace solve/lock parameters
- `config/cubes.yaml`: cube ID groups and cube pose parameters
- `config/tracker_params.yaml`: depth tracker + overlay parameters

## Size Reference

- AprilTag detector size: `0.032 m`
- Workspace/board tag size params (`tag_size_m`, `workspace_tag_size_m`): `0.032 m`
- Cube body edge size (`cube_edge_size_m`): `0.040 m`
- Cube face-center to cube-center offset (`cube_face_offset_m`): `0.020 m`
- Board geometry: `0.700 m x 0.500 m`
- Board tag center offset from board edge: measured placement (`0.016 m` in current configs)

If tag detections are consistently too far/near relative to depth/pointcloud, the detector `size` is likely not `0.032 m`.

## April Cube Groups

The cube pose node tracks four physical April cubes with six face tags each:
- `april_cube_1`: IDs `[10, 11, 12, 13, 14, 15]`
- `april_cube_2`: IDs `[16, 17, 18, 19, 20, 21]`
- `april_cube_3`: IDs `[22, 23, 24, 25, 26, 27]`
- `april_cube_4`: IDs `[28, 29, 30, 31, 32, 33]`

Each six-tag group above is one physical cube, not six independent objects.

Default face mapping is configured once in `config/cubes.yaml`:
- `april_cube_face_order: ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]`

For each cube group, the first ID is `+X`, second `-X`, third `+Y`, fourth `-Y`, fifth `+Z`, sixth `-Z`.
If sticker placement changes, update `april_cube_face_order` or reorder each `april_cube_N_tag_ids` list.

Cube-centre model:
- Detector pose is the face tag centre.
- Cube edge = `0.040 m`, so face-centre to cube-centre offset = `0.020 m`.
- Per-face local offsets are:
- `+X -> [-0.020, 0.000, 0.000]`
- `-X -> [+0.020, 0.000, 0.000]`
- `+Y -> [0.000, -0.020, 0.000]`
- `-Y -> [0.000, +0.020, 0.000]`
- `+Z -> [0.000, 0.000, -0.020]`
- `-Z -> [0.000, 0.000, +0.020]`
- Candidate centre is computed as `cube_center = tag_center + tag_rotation * local_face_offset`.
- Multiple visible faces are fused into one cube centre using candidate-consensus (`0.05 m` default threshold).

Published topics:
- `/holoassist/perception/april_cube_1_pose`
- `/holoassist/perception/april_cube_2_pose`
- `/holoassist/perception/april_cube_3_pose`
- `/holoassist/perception/april_cube_4_pose`
- `/holoassist/perception/april_cube_1_marker`
- `/holoassist/perception/april_cube_2_marker`
- `/holoassist/perception/april_cube_3_marker`
- `/holoassist/perception/april_cube_4_marker`
- `/holoassist/perception/april_cube_1_status`
- `/holoassist/perception/april_cube_2_status`
- `/holoassist/perception/april_cube_3_status`
- `/holoassist/perception/april_cube_4_status`

Published cube TF child frames:
- `apriltag_cube_1`
- `apriltag_cube_2`
- `apriltag_cube_3`
- `apriltag_cube_4`

Marker colors:
- Cube 1: red `(1.0, 0.0, 0.0, 0.75)`
- Cube 2: green `(0.0, 1.0, 0.0, 0.75)`
- Cube 3: blue `(0.0, 0.3, 1.0, 0.75)`
- Cube 4: yellow/orange `(1.0, 0.7, 0.0, 0.75)`

Marker cube scale in RViz is fixed to `0.040 x 0.040 x 0.040 m` per cube.

AprilTag detector config for cubes must keep:
- `size: 0.032`
- `ids: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33]`

## Verify

```bash
ros2 topic hz /detections_all
ros2 topic echo /detections_all --once
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/april_cube_1_status --once
ros2 topic echo /holoassist/perception/april_cube_2_status --once
ros2 topic echo /holoassist/perception/april_cube_3_status --once
ros2 topic echo /holoassist/perception/april_cube_4_status --once
```

Per-cube pose checks:

```bash
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/perception/april_cube_2_pose --once
ros2 topic echo /holoassist/perception/april_cube_3_pose --once
ros2 topic echo /holoassist/perception/april_cube_4_pose --once
```

Per-cube marker checks:

```bash
ros2 topic echo /holoassist/perception/april_cube_1_marker --once
ros2 topic echo /holoassist/perception/april_cube_2_marker --once
ros2 topic echo /holoassist/perception/april_cube_3_marker --once
ros2 topic echo /holoassist/perception/april_cube_4_marker --once
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_2
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_3
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_4
ros2 topic echo /tf
```

Useful checks:

```bash
grep -R "tag_size\\|tag_size_m\\|workspace_tag_size_m\\|size:\\|apriltag\\|0.15\\|0.10\\|0.100\\|0.045\\|0.040\\|0.032\\|32\\|40\\|100\\|150" -n ros2_ws
ros2 param list
ros2 param get /apriltag size
ros2 param get /holoassist_workspace_perception tag_size_m
ros2 param get /holo_assist_depth_tracker workspace_tag_size_m
ros2 param get /holoassist_cube_pose cube_edge_size_m
ros2 param get /holoassist_cube_pose cube_tag_size_m
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:10
```

## Realign Workspace

```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```

## Gazebo Simulation (D435i + Trolley + Board + Cubes)

This package now includes a simulation launch that runs without RealSense hardware:

- Gazebo world: `worlds/holoassist_trolley_board.world`
- Sim launch: `launch/sim_holoassist_trolley.launch.py`
- Synthetic AprilTag publisher: `holoassist_sim_apriltag_publisher_node`
- Synthetic tag config: `config/sim_apriltag_params.yaml`

Run:

```bash
ros2 launch holo_assist_depth_tracker sim_holoassist_trolley.launch.py
```

Useful args:
- `gui:=true|false`
- `start_tracker:=true|false`
- `start_overlay:=true|false`
- `start_workspace_perception:=true|false`
- `sim_tag_params_file:=.../sim_apriltag_params.yaml`

Notes:
- Camera topics are published to match your existing params:
- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`
- `/camera/camera/imu`
- AprilTag detections/TF are synthetic for deterministic bring-up:
- `/detections_all`
- tag TF frames `tag36h11:<id>`

If you import your own cube/trolley meshes, keep the simulated tag/board geometry aligned by updating:
- `worlds/holoassist_trolley_board.world`
- `config/sim_apriltag_params.yaml`

## Textured Board And AprilTag Visual Plates

RViz digital twin assets live under `worlds/`:

- `worlds/urdf/holoassist_scene.urdf.xacro`
- `worlds/meshes/apriltags/tag_0.dae` ... `tag_3.dae`
- `worlds/textures/apriltags/tag36_11_00000.png` ... `tag36_11_00003.png`
- `worlds/rviz/holoassist_scene.rviz`

Geometry reference:

- Board: `0.700 x 0.500 x 0.010 m`
- Tag size: `0.032 x 0.032 m`
- Tag center-to-edge offset (flush): `0.016 m`
- Tag plate Z: `0.006 m` (1 mm above board top at `z=0.005`)

Important:

- RViz tag textures are cosmetic only.
- They are not camera detections by themselves in RViz.
- For image-based detection, use a simulator that publishes camera streams (Gazebo/Unity/Isaac) with these same tag planes/textures.

Launch:

```bash
ros2 launch holo_assist_depth_tracker view_holoassist_scene.launch.py
```

Validation:

```bash
ros2 run xacro xacro worlds/urdf/holoassist_scene.urdf.xacro > /tmp/holoassist_scene.urdf
check_urdf /tmp/holoassist_scene.urdf
ros2 run tf2_ros tf2_echo workspace_frame tag0_visual
ros2 run tf2_ros tf2_echo workspace_frame tag1_visual
ros2 run tf2_ros tf2_echo workspace_frame tag2_visual
ros2 run tf2_ros tf2_echo workspace_frame tag3_visual
```
