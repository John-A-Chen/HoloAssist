# HoloAssist Runtime Unification Runbook (Ubuntu 22.04 / ROS 2 Humble)

Last updated: 2026-04-16  
Branch: `john`

This is the operator runbook for HoloAssist runtime bringup, Foxglove-first observability, perception validation, Unity bridge, and object localization debugging.

## 1) Objectives and Current System Model

Primary runtime goals:
- Keep Foxglove as the primary operations and debugging UI.
- Provide one single runtime launch entrypoint for no-hardware and full-hardware modes.
- Preserve existing subsystem interfaces and use incremental adapters.
- Track depth-derived objects in camera/workspace frames while maintaining human/blob safety gating.

Current integrated package set:
- `holoassist_foxglove`
- `holoassist_manager`
- `holoassist_unity_bridge`
- `holo_assist_depth_tracker`
- `holoassist_manipulation`
- `holoassist_servo_tools`
- `holoassist_movement`
- `ur3_keyboard_teleop`
- `ur3_joint_position_controller`

## 2) Dependencies (Exact Commands)

### 2.1 Apt prerequisites

```bash
sudo apt update
sudo apt install -y \
  git curl \
  python3-rosdep python3-colcon-common-extensions python3-vcstool \
  python3-numpy python3-opencv python3-pyqt5 \
  ros-humble-foxglove-bridge \
  ros-humble-realsense2-camera \
  ros-humble-cv-bridge \
  ros-humble-rviz2 \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ur-robot-driver \
  ros-humble-ur-dashboard-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-controller-manager-msgs \
  ros-humble-control-msgs \
  ros-humble-joint-trajectory-controller \
  ros-humble-trajectory-msgs
```

### 2.2 Unity ROS-TCP endpoint (source install in workspace)

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
if [ ! -d ROS-TCP-Endpoint ]; then
  git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
else
  cd ROS-TCP-Endpoint
  git fetch
  git checkout main-ros2
  git pull --ff-only
fi
```

### 2.3 rosdep

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd ~/git/RS2-HoloAssist/john/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys "ament_python"
```

### 2.4 Optional legacy React UI

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm ci
```

## 3) Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4) Launch Matrix

| Mode | Command |
|---|---|
| Unified stack (default no hardware) | `ros2 launch holoassist_foxglove holoassist_stack.launch.py` |
| Unified stack full hardware profile | `ros2 launch holoassist_foxglove holoassist_stack.launch.py profile:=full_hardware` |
| AprilTag workbench (standalone) | `ros2 launch holoassist_foxglove apriltag_workbench.launch.py start_camera:=true` |
| Unified stack + AprilTag tracking | `ros2 launch holoassist_foxglove holoassist_stack.launch.py enable_apriltag_tracking:=true apriltag_start_camera:=false` |
| Observability only | `ros2 launch holoassist_foxglove observability.launch.py` |
| Integrated runtime (existing) | `ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py` |
| Integrated runtime no-hardware-safe | `ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py enable_unity_bringup:=false enable_depth_tracker:=false enable_pointcloud_obstacle:=false enable_ur3_keyboard_teleop:=false enable_ur3_joint_controller:=false` |
| Perception only (RealSense + tracker + RViz) | `ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py start_camera:=true start_tracker:=true start_rviz:=true` |
| Perception only (no RealSense) | `ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py start_camera:=false start_tracker:=true start_rviz:=false` |
| Pointcloud -> MoveIt obstacle | `ros2 launch holoassist_manipulation pointcloud_to_moveit_obstacle.launch.py` |
| Keyboard motion path | `ros2 run ur3_joint_position_controller ur3_joint_position_controller` and `ros2 run ur3_keyboard_teleop keyboard_joint_teleop` |
| Unity TCP endpoint only | `ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py ros_ip:=0.0.0.0 ros_tcp_port:=10000` |
| Unity movement bringup path | `ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py` |
| Foxglove bridge only | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` |

## 5) No-Hardware Expected Behavior

| Signal | Expected | Why |
|---|---|---|
| `/holoassist/diagnostics` | Continuous publish | observability node runs regardless of hardware |
| `holoassist/perception` | `WARN` then `ERROR` | pointcloud/debug stream stale |
| `holoassist/teleop` | `WARN` | target/clicked/twist idle |
| `holoassist/planning` | `WARN` | clicked/planner marker idle |
| `holoassist/safety` | `ERROR` | `/joint_states` stale or absent |
| `holoassist/network` | `WARN` possible | Unity endpoint may be unreachable |
| `/holoassist/state/teleop` | `IDLE` | no active teleop commands |
| `/holoassist/state/planner` | `IDLE` | no active planner marker stream |
| `/holoassist/state/safety` | `ERROR` or `WARN` | safety inputs missing |
| `/holoassist/events` | active transitions | expected diagnostic state change logging |

## 6) Topic Verification Checklist

```bash
# 1) Namespace presence
ros2 topic list | sort | rg "^/(holoassist|holo_assist_depth_tracker|holoassist_manager|ur3_keyboard|unity|tf)"

# 2) Core observability snapshots
ros2 topic echo /holoassist/diagnostics --once
ros2 topic echo /holoassist/events --once
ros2 topic echo /holoassist/state/teleop --once
ros2 topic echo /holoassist/state/planner --once
ros2 topic echo /holoassist/state/safety --once
ros2 topic echo /holoassist/state/runtime --once

# 3) Rate checks
ros2 topic hz /holoassist/diagnostics
ros2 topic hz /holo_assist_depth_tracker/debug_image
ros2 topic hz /holo_assist_depth_tracker/pointcloud
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker

# 4) Perception payload sanity
ros2 topic echo /holo_assist_depth_tracker/bbox --once
ros2 topic echo /holo_assist_depth_tracker/pointcloud --once

# 5) Manager
ros2 topic echo /holoassist_manager/mode --once
ros2 topic echo /holoassist_manager/diagnostics --once

# 6) Object localization adapter topics
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
ros2 topic echo /holoassist/perception/object_marker --once
```

## 7) UI Setup

### 7.1 Foxglove (primary)
- Start runtime with bridge enabled.
- Connect Foxglove Studio to `ws://<host-ip>:8765`.
- Recommended panels:
  - Image: `/holo_assist_depth_tracker/debug_image`
  - 3D: `/tf`, `/holo_assist_depth_tracker/pointcloud`, `/holo_assist_depth_tracker/obstacle_marker`, `/holoassist/perception/object_marker`, `/clicked_goal_marker`
  - Diagnostics: `/holoassist/diagnostics`
  - Log: `/holoassist/events`
  - Plot: `/holoassist/metrics/*`

### 7.2 React dashboard (legacy fallback)

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm ci
npm run dev
```

Uses relay endpoints:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

### 7.3 PyQt dashboard (legacy fallback)

```bash
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash
python3 ~/git/RS2-HoloAssist/john/dashboard/main.py
```

## 8) Troubleshooting

### 8.1 Missing `foxglove_bridge`

```bash
sudo apt install -y ros-humble-foxglove-bridge
ros2 pkg list | rg "^foxglove_bridge$"
```

### 8.2 Missing `realsense2_camera` or RealSense stream issues

```bash
sudo apt install -y ros-humble-realsense2-camera
ros2 pkg list | rg "^realsense2_camera$"
ros2 launch realsense2_camera rs_launch.py --show-args
```

If unstable (USB bandwidth/transport):
- reduce profile in launch: `depth_profile:=640,480,15`

### 8.3 Missing `ros_tcp_endpoint`

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | rg "^ros_tcp_endpoint$"
```

### 8.4 Hardcoded `/home/nic` paths
Already patched in this branch:
- `launch.py`
- `launch.sh`
- `dashboard.sh`
- `Unity/My project/Packages/manifest.json`
- `Unity/My project/Packages/packages-lock.json`

### 8.5 Stale diagnostics when hardware is absent
- Expected in offline bringup.
- Treat as failure only if `/holoassist/diagnostics` is missing entirely.

## 9) Team Quickstart (Copy/Paste)

```bash
# Terminal 1: build + source
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Terminal 1: runtime (camera available, robot absent)
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true

# Terminal 2: verify observability + perception
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash
ros2 topic echo /holoassist/diagnostics --once
ros2 topic hz /holo_assist_depth_tracker/pointcloud
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker
ros2 topic echo /holoassist/perception/object_pose_workspace --once

# Foxglove: connect ws://<host-ip>:8765
```

## 10) Feature-Parity Mapping (Legacy UI -> Foxglove)

| Legacy runtime feature | Foxglove panel/topic | Parity | Adapter needed |
|---|---|---|---|
| Runtime health badges | `/holoassist/diagnostics` | Yes | No |
| Event stream | `/holoassist/events` | Yes | No |
| Joint monitor | `/joint_states`, `/holoassist/metrics/joint_states_hz` | Mostly | Optional normalized summary |
| Camera debug feed | `/holo_assist_depth_tracker/debug_image` | Yes | No |
| Headset feed | `/headset/image_compressed` | Yes (if source present) | No |
| Pointcloud + obstacle | `/holo_assist_depth_tracker/pointcloud`, `/holo_assist_depth_tracker/obstacle_marker` | Yes | No |
| Planner click marker | `/clicked_goal_marker` | Yes | No |
| Unity TCP reachability | `/holoassist/metrics/unity_tcp_latency_ms`, diagnostics network status | Yes | No |
| Object pose topicized for app consumers | `/holoassist/perception/object_pose*` + `/holoassist/perception/object_marker` | New in this branch | No |
| Controller active/inactive latch | N/A | Gap | Add `/holoassist/state/controller` adapter |
| Session mode/durations (`/session/status`) | N/A | Gap | Add `/holoassist/session/*` adapters |
| UI e-stop/resume semantics | N/A | Gap | command/service bridge topics/services |

## 11) Unified Stack Launch Entry Point

New entrypoint:
- `ros2 launch holoassist_foxglove holoassist_stack.launch.py`

Profiles:
- `profile:=no_hardware` (default): observability-first safe startup.
- `profile:=full_hardware`: enables Unity/robot/perception paths by default.

Override flags (all support `auto|true|false`):
- `enable_observability`
- `enable_foxglove_bridge`
- `enable_manager`
- `enable_tf_marker_bridge`
- `enable_object_pose_adapter`
- `enable_apriltag_tracking`
- `enable_depth_tracker`
- `enable_depth_camera`
- `enable_pointcloud_obstacle`
- `enable_unity_endpoint`
- `enable_unity_bringup`
- `enable_click_to_plan`
- `enable_ur3_keyboard_teleop`
- `enable_ur3_joint_controller`
- `enable_robot_demo`

Frame/network args:
- `ros_ip`, `ros_tcp_port`, `command_frame`, `eef_frame`, `object_workspace_frame`

AprilTag args:
- `apriltag_start_camera`
- `apriltag_image_topic`
- `apriltag_camera_info_topic`
- `apriltag_params_file`

## 12) Pointcloud + Object Localization Debug Workflow

### 12.1 Topic contract (standardized)
- `geometry_msgs/PoseStamped`: `/holoassist/perception/object_pose` (camera frame)
- `geometry_msgs/PoseStamped`: `/holoassist/perception/object_pose_workspace` (workspace frame)
- `visualization_msgs/Marker`: `/holoassist/perception/object_marker`

### 12.2 Camera/workspace frame checks

```bash
# Camera frame consistency
ros2 topic echo /holoassist/perception/object_pose --once

# Workspace transform consistency
ros2 run tf2_ros tf2_echo camera_depth_optical_frame base_link
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```

### 12.3 Human/blob safety gating
- Keep publishing:
  - `/holo_assist_depth_tracker/obstacle_marker`
  - `/holo_assist_depth_tracker/bbox`
- Keep `pointcloud_to_moveit_obstacle` active when planning scene gating is required.
- Policy: if blob/human obstacle marker active in workspace, planner must not place next trajectory through that volume.

### 12.4 Validation for your workflow
1. Object enters workspace and appears in pointcloud + object pose topics.
2. Object pose is visible in both RViz and Foxglove 3D.
3. Unity consumes same pose semantics via ROS bridge.
4. Teleop moves object to tray/bin; pose updates remain coherent.
5. Autonomous phase consumes object pose list / latest pose.
6. If human/blob enters scene, obstacle marker activates and planning obstacle remains enforced until scene clears.

## 13) AprilTag / ArUco / Hand-Eye Integration (Research + Recommended Path)

Your requirement (known workbench model + known robot base + known end-effector pose) maps directly to a staged calibration stack:

1. Camera intrinsics (already required for depth projection).
2. Camera extrinsics vs robot/workbench (`easy_handeye2`, eye-on-base).
3. Tag-based workspace anchoring (`apriltag_ros` or `aruco_ros`) for robust frame alignment and periodic recalibration checks.

### 13.1 Recommended practical architecture
- Start with one AprilTag on workbench for quick bringup.
- Move to a multi-tag board/grid (or multiple fixed tags) for robustness to partial occlusion and better pose stability.
- Publish a stable `workspace` frame from tag observations and calibrated camera extrinsics.
- Convert depth/object outputs to `workspace` for bin placement checks.

### 13.2 Package options (Humble)

```bash
sudo apt install -y ros-humble-apriltag ros-humble-apriltag-ros
sudo apt install -y ros-humble-aruco-ros ros-humble-aruco-msgs
apt-cache policy ros-humble-apriltag-ros ros-humble-aruco-ros
```

`easy_handeye2` (source install):

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -iyr --from-paths src --ignore-src
colcon build --symlink-install
```

### 13.3 Practical calibration execution (single tag -> multi-tag)

1. Print one `36h11` AprilTag with measured edge length (for example `0.035 m`) and mount it rigidly on the workbench.
2. Launch depth + AprilTag in one runtime:

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false
```

3. Verify detections and TF:

```bash
ros2 topic echo /detections --once
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
```

4. Run hand-eye calibration (`eye_on_base`) once tag tracking is stable:

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=realsense_eye_on_base \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=tag36h11:0
```

5. Publish saved calibration in normal runtime launches:

```bash
ros2 launch easy_handeye2 publish.launch.py name:=realsense_eye_on_base
```

6. Upgrade to multi-tag by editing:
- `ros2_ws/src/holoassist_foxglove/config/apriltag_workbench_36h11.yaml`
- Fill `tag.ids`, `tag.frames`, and `tag.sizes` so only known bench tags publish TF.

### 13.4 ArUco vs AprilTag (research-backed recommendation)
- `apriltag_ros` is the recommended default for workbench anchoring in this stack.
- `apriltag_ros` exposes per-tag filtering and per-tag frame/size settings (`tag.ids`, `tag.frames`, `tag.sizes`), which directly supports fixed multi-tag bench layouts.
- `aruco_ros` is still useful when board-based tracking is preferred. Its README explicitly lists enhanced precision using boards of markers.
- Keep one tag family per deployment to avoid decoder ambiguity and reduce false positives.

### 13.5 Research references
- `apriltag_ros` ROS 2 node README (topics/config, per-tag IDs/frames/sizes, TF publishing):
  - https://github.com/christianrauch/apriltag_ros
- Unity Robotics ROS-TCP endpoint setup for ROS 2 and `default_server_endpoint` usage:
  - https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md
  - https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2
- `easy_handeye2` ROS 2 hand-eye calibration (eye-on-base and eye-in-hand):
  - https://github.com/marcoesposito1988/easy_handeye2
- `aruco_ros` (single/board marker tracking, marker arrays, fiducial tracking):
  - https://github.com/pal-robotics/aruco_ros

### 13.6 Why this fits your stated workflow
- You can detect and track object ingress/pose in workspace using depth + CV.
- You can keep blob/human safety exclusion zones in planning via marker/pointcloud obstacle outputs.
- You can anchor both Unity and RViz to the same calibrated frames, reducing frame drift/misalignment during teleop/autonomy transitions.

## 14) Implementation Notes in This Branch

Implemented runtime changes:
- New unified launch entrypoint:
  - `ros2_ws/src/holoassist_foxglove/launch/holoassist_stack.launch.py`
- New optional AprilTag launch path:
  - `ros2_ws/src/holoassist_foxglove/launch/apriltag_workbench.launch.py`
  - `ros2_ws/src/holoassist_foxglove/config/apriltag_workbench_36h11.yaml`
- New object localization adapter node:
  - `holoassist_foxglove/obstacle_to_object_pose_adapter.py`
  - maps `/holo_assist_depth_tracker/obstacle_marker` to:
    - `/holoassist/perception/object_pose`
    - `/holoassist/perception/object_pose_workspace`
    - `/holoassist/perception/object_marker`
- Observability/runtime/unity launch wiring updated to include object pose adapter controls.
- Hardcoded `/home/nic` script/Unity path issues removed.

## 15) Workspace Understanding Pipeline (Bench Plane + Tags)

New node:
- `ros2_ws/src/holo_assist_depth_tracker/holo_assist_depth_tracker/workspace_perception_node.py`

New config:
- `ros2_ws/src/holo_assist_depth_tracker/config/workspace_perception_params.yaml`

New launch:
- `ros2_ws/src/holo_assist_depth_tracker/launch/workspace_perception.launch.py`

### 15.1 What it does
- Fits a loose bench plane from `/holo_assist_depth_tracker/pointcloud` with robust RANSAC + SVD refinement.
- Publishes bench plane coefficients and a bench plane marker.
- Publishes `workspace_frame` TF:
  - origin: inlier centroid projected onto bench plane.
  - `+Z`: plane normal pointing from bench toward camera origin.
  - `+X`: from tag0->tag1 projected onto plane when both tags are available; otherwise camera-x projected onto plane.
  - `+Y`: `cross(Z, X)`.
- Falls back to `plane_only` when tags are missing and reports `invalid` when plane fit fails.
- Crops points in workspace ROI and optional pedestal exclusion zone.
- Builds a simple voxel persistence background model and extracts foreground points.
- Clusters foreground and publishes best object estimate:
  - `/holoassist/perception/object_pose` (`geometry_msgs/PoseStamped`, camera frame)
  - `/holoassist/perception/object_marker` (`visualization_msgs/Marker`, camera frame)
  - `/holoassist/perception/object_pose_workspace` (`geometry_msgs/PoseStamped`, workspace frame)

### 15.2 Run commands (today)

Plane-only:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_unity_bringup:=false
```

Plane + AprilTags:
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false \
  enable_unity_bringup:=false
```

### 15.3 Debug/validation topics
- Raw depth cloud (unchanged): `/holo_assist_depth_tracker/pointcloud`
- Cropped ROI cloud: `/holoassist/perception/cropped_pointcloud`
- Foreground cloud: `/holoassist/perception/foreground_pointcloud`
- Plane marker: `/holoassist/perception/bench_plane_marker`
- Workspace axes marker: `/holoassist/perception/workspace_axes_marker`
- Tag markers: `/holoassist/perception/workspace_tag_markers`
- Plane coefficients: `/holoassist/perception/bench_plane_coefficients`
- Workspace mode + diagnostics:
  - `/holoassist/perception/workspace_mode`
  - `/holoassist/perception/workspace_diagnostics`
- Final object outputs:
  - `/holoassist/perception/object_pose`
  - `/holoassist/perception/object_marker`
  - `/holoassist/perception/object_pose_workspace`

Quick checks:
```bash
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
ros2 topic hz /holoassist/perception/cropped_pointcloud
ros2 topic hz /holoassist/perception/foreground_pointcloud
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```
