# ROS API Reference

## Perception Topics

| Topic | Type | Publisher | Description |
|---|---|---|---|
| `/holoassist/perception/april_cube_1_pose` | `PoseStamped` | cube_pose_node | Cube 1 centre (tags 10–15) |
| `/holoassist/perception/april_cube_2_pose` | `PoseStamped` | cube_pose_node | Cube 2 centre (tags 16–21) |
| `/holoassist/perception/april_cube_3_pose` | `PoseStamped` | cube_pose_node | Cube 3 centre (tags 22–27) |
| `/holoassist/perception/april_cube_4_pose` | `PoseStamped` | cube_pose_node | Cube 4 centre (tags 28–33) |
| `/holoassist/perception/april_cube_N_marker` | `Marker` | cube_pose_node | RViz cube marker (N = 1–4) |
| `/holoassist/perception/april_cube_N_status` | `String` | cube_pose_node | Detection state + diagnostics |
| `/holoassist/perception/april_cube_N_bin_check` | `String` | cube_pose_node | Bin verification result |
| `/holoassist/perception/april_cube_pose` | `PoseStamped` | cube_pose_node | Legacy: cube 1 (or nearest) |
| `/detections_all` | `AprilTagDetectionArray` | apriltag_ros | Raw tag detections from camera |

### Bin check message format

```
cube_id=<N> sorted=<true|false> bin=<bin_name|none> distance_xy_m=<float> margin_m=<float>
```

### Status message format

```
cube_id=<N> state=<visible|stale|waiting_for_detections> reason=<string>
visible_tag_ids=<list> selected_tag_id=<int> rejected_tag_ids=<list>
candidate_count=<int> candidate_spread_m=<float> method=<string>
age_s=<float> frame_id=<string> position_m=(<x>,<y>,<z>)
marker_scale_m=(<x>,<y>,<z>) color_rgba=(<r>,<g>,<b>,<a>)
```

---

## Robot Driver Topics

| Topic | Type | Description |
|---|---|---|
| `/joint_states` | `JointState` | Arm + gripper joint positions |
| `/tcp_pose_broadcaster/pose` | `PoseStamped` | End-effector TCP pose (real robot only) |
| `/io_and_status_controller/robot_mode` | `RobotModeMsg` | UR robot mode |
| `/scaled_joint_trajectory_controller/joint_trajectory` | `JointTrajectory` | MoveIt trajectory input |
| `/forward_velocity_controller/commands` | `Float64MultiArray` | Teleop velocity input |
| `/finger_width_controller/commands` | `Float64MultiArray` | Gripper width (mm) |

---

## MoveIt / Planning Topics

| Topic | Type | Description |
|---|---|---|
| `/moveit_robot_control/target_pose` | `Pose` | Command arm to Cartesian pose |
| `/moveit_robot_control/target_point` | `Point` | Command arm to XYZ position |
| `/moveit_robot_control/status` | `String` | Execution status |
| `/planning_scene` | `PlanningScene` | MoveIt planning scene |

---

## Services

| Service | Type | Description |
|---|---|---|
| `/controller_manager/switch_controller` | `SwitchController` | Activate/deactivate controllers |
| `/holoassist/perception/realign_workspace` | `Trigger` | Re-run workspace board SVD solve |

---

## TF Frames

| Frame | Parent | Published by |
|---|---|---|
| `world` | — | robot_state_publisher |
| `base_link` | `world` | robot_state_publisher |
| `camera_color_optical_frame` | `base_link` | hand-eye calibration static TF |
| `apriltag_cube_1` .. `apriltag_cube_4` | `camera_color_optical_frame` | cube_pose_node |
| `tag36h11:<id>` | `camera_color_optical_frame` | apriltag_ros |

---

## Log Files

| Path | Content |
|---|---|
| `/tmp/holoassist_ur_onrobot_driver.log` | UR driver, RTDE, gripper |
| `/tmp/holoassist_cube_pose.log` | AprilTag detection + bin checks |
| `/tmp/holoassist_moveit.log` | MoveIt planning + execution |
| `/tmp/holoassist_bridge.log` | ROS-TCP bridge |
| `/tmp/holoassist_trolley_scene_publisher.log` | Trolley scene TF |
