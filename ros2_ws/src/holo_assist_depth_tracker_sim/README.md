# holo_assist_depth_tracker_sim

Geometry-based HoloAssist backend simulation without XR hardware.

## What this sim does

- RViz digital twin scene (board, tags, robot base, camera frames).
- 4 movable truth cubes.
- Visibility-based synthetic perception using camera geometry.
- Last-seen perception behavior when cubes leave view.
- MoveIt bridge from perceived cube poses only.

## Fake D435i camera and frustum debugging

This sim now includes a fake Intel RealSense D435i-style frame model.

- This is **not** full RGB/depth image simulation.
- This is a TF + geometry visibility model.
- Visibility checks are done in `camera_color_optical_frame`.
- Frustum marker visualizes the camera view volume directly in RViz.

Optical frame convention used:

- `+Z` forward
- `+X` right
- `+Y` down

## Camera frame tree

Dynamic camera root:

- `workspace_frame -> camera_link`

Fake D435i child frames:

- `camera_link -> camera_color_frame -> camera_color_optical_frame`
- `camera_link -> camera_depth_frame -> camera_depth_optical_frame`
- `camera_link -> camera_imu_frame -> camera_accel_frame`
- `camera_link -> camera_imu_frame -> camera_gyro_frame`

## Core topics

Truth cubes:

- `/holoassist/sim/truth/april_cube_1_pose`
- `/holoassist/sim/truth/april_cube_2_pose`
- `/holoassist/sim/truth/april_cube_3_pose`
- `/holoassist/sim/truth/april_cube_4_pose`

Perception cubes:

- `/holoassist/sim/perception/april_cube_1_pose`
- `/holoassist/sim/perception/april_cube_2_pose`
- `/holoassist/sim/perception/april_cube_3_pose`
- `/holoassist/sim/perception/april_cube_4_pose`
- `/holoassist/sim/perception/april_cube_1_status`
- `/holoassist/sim/perception/april_cube_2_status`
- `/holoassist/sim/perception/april_cube_3_status`
- `/holoassist/sim/perception/april_cube_4_status`

Camera debug:

- `/holoassist/sim/truth/camera_pose`
- `/holoassist/sim/camera/body_marker`
- `/holoassist/sim/camera/optical_axis_marker`
- `/holoassist/sim/camera/frustum_marker`
- `/holoassist/sim/camera/info`
- `/holoassist/sim/camera/debug_status`

MoveIt bridge:

- `/planning_scene`
- `/holoassist/teleop/selected_cube`
- `/holoassist/teleop/selected_cube_pose`
- `/moveit_robot_control/target_point` (`geometry_msgs/Point`, selected-cube hover coordinate in `base_link`)
- `/moveit_robot_control/target_pose` (`geometry_msgs/Pose`, selected-cube pre-grasp hover pose in `base_link`)
- `/moveit_robot_control/target` (`geometry_msgs/Point`, optional legacy mirror for older listener variants)

## Services

- `/holoassist/sim/randomise_april_cubes` (`std_srvs/srv/Trigger`)
- `/holoassist/sim/reset_april_cubes` (`std_srvs/srv/Trigger`)
- `/holoassist/sim/set_april_cube_pose` (`holo_assist_depth_tracker_sim_interfaces/srv/SetAprilCubePose`)
- `/holoassist/sim/set_camera_pose` (`holo_assist_depth_tracker_sim_interfaces/srv/SetCameraPose`)
- `/holoassist/sim/reset_camera_pose` (`std_srvs/srv/Trigger`)
- `/holoassist/sim/reset_perception_memory` (`std_srvs/srv/Trigger`)
- `/holoassist/perception/realign_workspace` (`std_srvs/srv/Trigger`, sim stub)

## Launch

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch holo_assist_depth_tracker_sim sim_april_cube_moveit.launch.py
```

## Default camera pose

Configured in `config/sim_camera.yaml`:

- `x=0.0`
- `y=-0.55`
- `z=0.65`
- `roll=0.0`
- `pitch=0.85`
- `yaw=1.575`

This is used by `/holoassist/sim/reset_camera_pose`.

## Validation checklist

### A. Frame tree checks

```bash
ros2 run tf2_ros tf2_echo workspace_frame camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_frame
ros2 run tf2_ros tf2_echo camera_color_frame camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_link camera_depth_frame
ros2 run tf2_ros tf2_echo camera_depth_frame camera_depth_optical_frame
```

### B. Camera pose service check

```bash
ros2 service call /holoassist/sim/set_camera_pose holo_assist_depth_tracker_sim_interfaces/srv/SetCameraPose "{x: 0.0, y: -0.55, z: 0.65, roll: 0.0, pitch: 0.85, yaw: 1.575}"
ros2 run tf2_ros tf2_echo workspace_frame camera_link
```

### C. Frustum check

```bash
ros2 topic echo /holoassist/sim/camera/frustum_marker --once
```

Move camera and verify frustum marker moves in RViz.

### D. Cube visibility + last-seen check

```bash
ros2 service call /holoassist/sim/set_april_cube_pose holo_assist_depth_tracker_sim_interfaces/srv/SetAprilCubePose "{cube_name: april_cube_1, cube_id: 0, x: 0.0, y: 0.0, z: 0.02, yaw: 0.0}"
ros2 topic echo /holoassist/sim/perception/april_cube_1_status --once
ros2 topic echo /holoassist/sim/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/sim/truth/april_cube_1_pose --once
```

Expected behavior:

- in frustum: `visible_now: true`, perceived pose follows truth.
- outside frustum: `visible_now: false`, perceived pose holds last seen.
- back in frustum: perceived pose jumps to current truth.

### E. Selected cube + MoveIt bridge check

```bash
ros2 topic pub /holoassist/teleop/selected_cube std_msgs/msg/String "{data: 'april_cube_1'}" --once
ros2 topic echo /holoassist/teleop/selected_cube_pose --once
ros2 topic echo /planning_scene --once
ros2 topic echo /moveit_robot_control/target_point --once
ros2 topic echo /moveit_robot_control/target_pose --once
```

`selected_cube_pose` and planning scene objects are driven by perceived poses, not hidden truth poses.

### F. Coordinate listener integration check (first milestone)

Run MoveIt + Ollie coordinate listener separately, then verify this sim publishes targets:

```bash
ros2 launch holo_assist_depth_tracker_sim sim_april_cube_moveit.launch.py \
  moveit_target_frame:=base_link \
  moveit_target_topic:=/moveit_robot_control/target_point \
  moveit_target_pose_topic:=/moveit_robot_control/target_pose

ros2 topic pub /holoassist/teleop/selected_cube std_msgs/msg/String "{data: 'april_cube_1'}" --once
ros2 topic echo /moveit_robot_control/target_point --once
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/status --once
```

Point target meaning (`/moveit_robot_control/target_point`):

- The adapter publishes a **hover coordinate** above the selected perceived cube.
- Position source is `/holoassist/teleop/selected_cube_pose` (perception output, not truth pose).
- Point is transformed into `moveit_target_frame` (default and intended: `base_link`).
- Offsets are applied in target frame:
  - `target_x_offset_m` (default `0.0`)
  - `target_y_offset_m` (default `0.0`)
  - `target_z_offset_m` (default `+0.10`)
- If `moveit_target_frame` does not exist in TF yet (for example sim-only run without robot MoveIt stack), no point is published until TF is available.

Pre-grasp pose meaning (`/moveit_robot_control/target_pose`):

- Published type is `geometry_msgs/msg/Pose` (matches newer coordinate listener input directly).
- Position is the same transformed hover position used for `target_point`.
- Orientation is fixed by parameters (not inferred from cube yaw yet):
  - `target_roll_rad` (default `3.141592653589793`)
  - `target_pitch_rad` (default `0.0`)
  - `target_yaw_rad` (default `0.0`)
- This is a **pre-grasp hover pose**, not a final grasp pose.

Listener topic compatibility:

- Newer coordinate listener variants consume `/moveit_robot_control/target_point`.
- Newer coordinate listener variants also consume `/moveit_robot_control/target_pose`.
- Older Ollie listener variants consume `/moveit_robot_control/target` (also `geometry_msgs/Point`).
- For older listener variants, mirror publish with:

```bash
ros2 launch holo_assist_depth_tracker_sim sim_april_cube_moveit.launch.py \
  moveit_target_frame:=base_link \
  moveit_target_topic:=/moveit_robot_control/target_point \
  moveit_target_pose_topic:=/moveit_robot_control/target_pose \
  moveit_target_topic_legacy:=/moveit_robot_control/target
```

Validation sequence for perceived + last-seen behavior:

```bash
ros2 topic pub /holoassist/teleop/selected_cube std_msgs/msg/String "{data: 'april_cube_1'}" --once
ros2 topic echo /holoassist/teleop/selected_cube_pose --once
ros2 topic echo /moveit_robot_control/target_point --once
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/target --once
```

- Move a visible cube: point + pose should update.
- Hide cube from camera: selected perceived pose, point, and pose hold last-seen position.
- Bring cube back: outputs jump to current perceived pose.

## Limitations and future path

- RViz here is visualization/debug only.
- No synthetic RGB topic generation yet.
- No synthetic depth image generation yet.
- No image-based AprilTag detection yet.
- Next step for full camera simulation: Gazebo/Unity/Isaac rendering RGB/depth/camera_info from this same frame/layout model.
