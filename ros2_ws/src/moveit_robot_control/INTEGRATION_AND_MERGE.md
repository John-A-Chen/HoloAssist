# HoloAssist MoveIt Sim Integration and Merge Guide

This document describes:

1. The integrated simulation + MoveIt architecture on branch `j0hn`
2. How to run and validate the full pipeline end-to-end
3. Where to tune scene placement and targeting parameters
4. How to merge this branch safely into `main`

## 1. Integration scope

The integration goal is one coherent run sequence where a single RViz session shows:

- UR3e + MoveIt robot model
- Trolley/workspace scene mesh
- Workspace board + 4 tags
- Fake D435i camera + optical axis + frustum
- Truth cubes + perceived cubes
- Selected perceived target driving MoveIt target topics

Motion targeting uses:

- `/moveit_robot_control/target_pose` as the primary command path
- `/moveit_robot_control/target_point` retained for compatibility/debugging

Planning reference remains `base_link`.

## 2. Top-level launch

Primary launch entry point:

- `moveit_robot_control/launch/full_holoassist_moveit_sim.launch.py`

Run:

```bash
cd /home/john/git/RS2-HoloAssist/j0hn/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

Key behavior:

- Starts MoveIt bringup from `ur_onrobot_moveit_config` (fake-hardware defaults)
- Starts workspace scene manager (trolley mesh marker)
- Starts static `base_link -> workspace_frame` TF publisher
- Starts coordinate listener with `require_robot_status:=false` default
- Starts sim truth/perception/cube bridge stack
- Starts one RViz config for the full integrated view

## 3. Frame model and placement

### Planning root

- `base_link` is the planning frame for MoveIt targets.

### Workspace TF owner

- Node: `holoassist_workspace_frame_tf`
- File: `moveit_robot_control_node/workspace_frame_tf.py`
- Parameters are loaded from:
  - `moveit_robot_control/config/full_holoassist_sim.yaml`

Default transform:

- `base_link -> workspace_frame`
  - `x_m: -0.100`
  - `y_m: -0.314`
  - `z_m: 0.015`
  - `roll_rad: 0.0`
  - `pitch_rad: 0.0`
  - `yaw_rad: 0.0`

These defaults are chosen so workspace board/camera/cubes are placed relative to the robot base rather than identity overlay.

## 4. Parameter ownership (single source map)

## A. Integrated stack tuning

File: `moveit_robot_control/config/full_holoassist_sim.yaml`

- `holoassist_workspace_frame_tf.ros__parameters.parent_frame`
  - Parent frame for static TF (default `base_link`)
- `holoassist_workspace_frame_tf.ros__parameters.child_frame`
  - Child frame for static TF (default `workspace_frame`)
- `holoassist_workspace_frame_tf.ros__parameters.x_m|y_m|z_m|roll_rad|pitch_rad|yaw_rad`
  - Robot-to-workspace placement

- `workspace_scene_manager.ros__parameters.table_mesh_resource`
  - Trolley mesh URI
- `workspace_scene_manager.ros__parameters.table_mesh_xyz`
  - Trolley mesh translation in `base_link`
- `workspace_scene_manager.ros__parameters.table_mesh_rpy_deg`
  - Trolley mesh orientation in `base_link`
- `workspace_scene_manager.ros__parameters.table_mesh_scale`
  - Trolley mesh scale
- `workspace_scene_manager.ros__parameters.table_collision_*`
  - Optional collision box tuning

- `holoassist_selected_cube_to_moveit_target.ros__parameters.target_x_offset_m|target_y_offset_m|target_z_offset_m`
  - Hover/pre-grasp position offsets
- `holoassist_selected_cube_to_moveit_target.ros__parameters.target_roll_rad|target_pitch_rad|target_yaw_rad`
  - Pose orientation sent on `/moveit_robot_control/target_pose`

## B. Workspace board/cube geometry

File: `holo_assist_depth_tracker_sim/config/sim_scene.yaml`

- `workspace_frame`
- `board_width_m`
- `board_depth_m`
- `board_thickness_m`
- `cube_size_m`

## C. Fake camera model and visibility

File: `holo_assist_depth_tracker_sim/config/sim_camera.yaml`

- `camera_default_xyzrpy`
- `camera_frame`
- `horizontal_fov_deg`
- `vertical_fov_deg`
- `near_clip_m`
- `far_clip_m`
- `max_range_m`
- `frustum_color_rgba`
- `frustum_line_width`

## D. Cube default spawn and spacing

File: `holo_assist_depth_tracker_sim/config/sim_cubes.yaml`

- `default_cube_centers_xyz_yaw`
- `min_cube_spacing_m`
- `randomise_yaw`

## 5. Perception-to-motion data flow

1. `sim_cube_truth_node`
   - Publishes draggable truth cubes
   - Publishes camera TF tree and camera markers

2. `sim_cube_perception_node`
   - Computes visibility in `camera_color_optical_frame`
   - Publishes perceived cube poses only when visible
   - Preserves last-seen memory when hidden

3. `sim_cube_moveit_bridge_node`
   - Publishes perceived cubes to MoveIt planning scene
   - Publishes selected perceived cube pose to `/holoassist/teleop/selected_cube_pose`

4. `selected_cube_to_moveit_target_node`
   - Transforms selected perceived pose into `base_link`
   - Publishes `/moveit_robot_control/target_pose` and `/moveit_robot_control/target_point`

5. `coordinate_listener`
   - Consumes pose/point target topics
   - Plans and executes via MoveIt

No truth pose is used for robot target generation.

## 6. Legacy message robustness

`coordinate_listener` conditionally enables the legacy subscription:

- Topic: `/moveit_robot_control/target`
- Type: `moveit_robot_control_msgs/msg/TargetRPY`

If `moveit_robot_control_msgs` is absent:

- Node does not crash
- Warning is logged
- `Point` and `Pose` subscriptions remain active

## 7. RViz expectations

RViz config:

- `holo_assist_depth_tracker_sim/rviz/holoassist_moveit_full.rviz`

Included displays:

- RobotModel
- TF
- `/workspace_scene/markers` (trolley marker array)
- Workspace board marker
- Workspace tag marker
- Camera body / optical axis / frustum markers
- Truth cube markers
- Perceived cube markers
- Interactive markers for truth cube dragging

## 8. Validation checklist

After launching, verify:

```bash
ros2 topic list | egrep "workspace_scene/markers|moveit_robot_control/target_pose|moveit_robot_control/target_point|moveit_robot_control/state|moveit_robot_control/status"

ros2 topic echo /workspace_scene/markers --once
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/target_point --once
ros2 topic echo /moveit_robot_control/state --once
ros2 topic echo /moveit_robot_control/status --once

ros2 topic echo /holoassist/sim/camera/frustum_marker --once
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo base_link workspace_frame
ros2 run tf2_ros tf2_echo workspace_frame camera_link
```

Selection check:

```bash
ros2 topic pub --once /holoassist/teleop/selected_cube std_msgs/msg/String "{data: 'april_cube_1'}"
ros2 topic echo /holoassist/teleop/selected_cube_pose --once
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/target_point --once
```

## 9. Merge strategy (`j0hn` -> `main`)

Recommended sequence:

```bash
# Ensure local branch is up to date
git checkout j0hn
git fetch origin
git pull --ff-only origin j0hn

# Open merge target
git checkout main
git pull --ff-only origin main

# Merge integration branch
git merge --no-ff j0hn

# Resolve conflicts if any, then
colcon build --packages-select moveit_robot_control holo_assist_depth_tracker_sim --symlink-install

# Push main
git push origin main
```

Alternative if you prefer rebased history before merge:

```bash
git checkout j0hn
git fetch origin
git rebase origin/main
# resolve conflicts

git checkout main
git merge --ff-only j0hn
git push origin main
```

## 10. Conflict hot spots during merge

Watch these paths first:

- `holo_assist_depth_tracker_sim/launch/*.launch.py`
- `holo_assist_depth_tracker_sim/rviz/*.rviz`
- `holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/sim_cube_truth_node.py`
- `moveit_robot_control/moveit_robot_control_node/moveit_robot_control.py`
- `moveit_robot_control/README.md`

If conflicts appear in launch files, preserve:

- Full-stack launch entry point
- `publish_scene_state_publisher` integration flag
- `require_robot_status:=false` fake-hardware path
- `base_link` planning frame

## 11. Post-merge sanity checks

After merge to `main`:

1. Build succeeds for integration packages
2. Full launch starts without missing-node errors
3. RViz shows trolley + robot + workspace + camera + cubes
4. Selecting a perceived cube updates target pose/point topics
5. Coordinate listener remains functional when legacy `TargetRPY` package is unavailable

---

Maintainer note: this document intentionally centralizes parameter ownership to make future scene tuning and grasp-stage extension easier.
