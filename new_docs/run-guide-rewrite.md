# Updated Run Guide Draft

This is a website-ready replacement outline for `site/docs/run-guide.md` and the launch sections of `site/docs/running.html`.

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble.
- Workspace built:

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

- For real robot runs, UR3e and laptop must be on the robot subnet.
- For Unity/Quest, the headset must be able to reach the laptop on TCP port 10000.

## Calibration

Run calibration after moving the camera or robot base:

```bash
cd /home/john/git/RS2-HoloAssist/main
./scripts/calibrate.sh --robot-ip 192.168.0.194
```

Current calibration stack:

1. `ur_onrobot_control start_robot.launch.py`
2. `holoassist_perception camera.launch.py`
3. `apriltag_ros apriltag_node`
4. `easy_handeye2 calibrate.launch.py`

Current calibration parameters:

| Parameter | Value |
|---|---|
| `calibration_type` | `eye_on_base` |
| `robot_base_frame` | `base_link` |
| `robot_effector_frame` | `tool0` |
| `tracking_base_frame` | `camera_link` |
| `tracking_marker_frame` | `tag36h11:1` |

References:

- Calibration launch command: `scripts/calibrate.py:14`
- Camera launch: `scripts/calibrate.py:120`
- AprilTag settings: `scripts/calibrate.py:129`
- easy_handeye2 settings: `scripts/calibrate.py:141`

Verify:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Main Launcher

Run everything with:

```bash
cd /home/john/git/RS2-HoloAssist/main
./scripts/launch.sh
```

Current default behavior:

- No `--robot-ip`: fake hardware mode.
- Perception on by default.
- MoveIt on by default.
- Dashboard on by default.
- RViz on by default.
- ROS-TCP Endpoint starts on port 10000.

Use opt-out flags for narrower runs:

| Command | Meaning |
|---|---|
| `./scripts/launch.sh` | Fake hardware, perception on, MoveIt on, dashboard on. |
| `./scripts/launch.sh --no-moveit` | Fake hardware plus perception, but switches to teleop controllers. |
| `./scripts/launch.sh --no-perception` | Fake hardware plus MoveIt, but no camera/AprilTag pipeline. |
| `./scripts/launch.sh --no-moveit --no-perception` | Driver/teleop/Unity bridge only. |
| `./scripts/launch.sh --robot-ip 192.168.0.194` | Real robot, perception on, MoveIt on, dashboard on. |
| `./scripts/launch.sh --robot-ip 192.168.0.194 --no-moveit` | Real robot teleop mode. |
| `./scripts/launch.sh --robot-ip 192.168.56.101 --fake-gripper` | URSim/real UR path with fake OnRobot gripper hardware. |

Flags:

| Flag | Current effect |
|---|---|
| `--robot-ip <IP>` | Real robot/URSim mode. Omit for fake hardware. |
| `--ros-ip <IP>` | Bind IP passed to `ros_tcp_endpoint`; default `0.0.0.0`. |
| `--no-rviz` | Disable all RViz windows. |
| `--no-perception` | Disable camera + AprilTag + perception launch. |
| `--no-moveit` | Disable MoveIt pick/place stack and switch to teleop controllers. |
| `--no-dashboard` | Disable dashboard and bridge server. |
| `--dashboard-fullscreen` | Start dashboard fullscreen. |
| `--fake-gripper` | With real/URSim arm, skip physical OnRobot serial gripper hardware. |
| `--verbose` | Print subprocess output instead of only writing logs. |

References:

- `--no-perception` and `--no-moveit`: `scripts/launch.py:327`, `scripts/launch.py:331`.
- Default perception/MoveIt true: `scripts/launch.py:335`.
- Dashboard default true: `scripts/launch.py:348`.
- Fake/real mode: `scripts/launch.py:359`.

## What Starts

Always or near-always:

| Process | Current command |
|---|---|
| Dashboard bridge | `python3 dashboard/bridge_server.py` unless `--no-dashboard`. |
| Dashboard UI | `python3 dashboard/main.py` unless `--no-dashboard`. |
| UR + OnRobot driver | `ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 ...` |
| Trolley scene publisher | `ros2 run holoassist_perception holoassist_trolley_scene_publisher` |
| Calibration poses publisher | `python3 calibration/waypoint_publisher_node.py` |
| ROS-TCP Endpoint | `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=...` |
| IP Beacon | `python3 scripts/beacon.py --ip <wifi_ip>` |

Optional:

| Condition | Process |
|---|---|
| Perception enabled and camera detected | `ros2 launch holoassist_perception perception.launch.py start_camera:=... start_rviz:=false` |
| Webcam/Brio detected | `ros2 run holoassist_perception holoassist_webcam_image_publisher ...` before perception launch |
| MoveIt enabled | `ros2 launch holoassist_movement movement.launch.py robot_ip:=... use_rviz:=...` |

References:

- Driver command: `scripts/launch.py:433`
- Trolley publisher: `scripts/launch.py:446`
- Perception command: `scripts/launch.py:523`
- ROS-TCP endpoint: `scripts/launch.py:550`
- MoveIt command: `scripts/launch.py:581`

## Controller Modes

MoveIt mode:

- Active when `--no-moveit` is not passed.
- Keeps `scaled_joint_trajectory_controller` and gripper trajectory controller active.
- Sends planned arm trajectories to `/scaled_joint_trajectory_controller/joint_trajectory`.

Teleop mode:

- Active when `--no-moveit` is passed.
- Switches to `forward_velocity_controller` and `finger_width_controller`.
- Unity publishes:
  - `/forward_velocity_controller/commands`
  - `/finger_width_controller/commands`

References:

- Controller switch logic: `scripts/launch.py:531`
- Unity teleop topics: `Unity/My project/Assets/Scripts/RobotController.cs:183`
- MoveIt trajectory topic: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:215`

## Pick and Place

With MoveIt enabled:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

Short form is accepted:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: '1', bin_id: '1'}"
```

Monitor:

```bash
ros2 topic echo /pick_place/status
ros2 topic echo /holoassist/movement/state
ros2 topic echo /holoassist/movement/debug
```

Manual motion targets:

```bash
ros2 topic pub --once /holoassist/movement/target_point \
  geometry_msgs/msg/Point "{x: 0.20, y: -0.30, z: 0.15}"

ros2 topic pub --once /holoassist/movement/target_pose \
  geometry_msgs/msg/Pose \
  "{position: {x: 0.20, y: -0.30, z: 0.025}, orientation: {w: 1.0}}"
```

References:

- Service definition: `ros2_ws/src/HoloAssist_Perception/srv/PickCubeToBin.srv:1`
- Service node: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:61`
- Movement topics: `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py:2023`

## Bin Poses

Configured in:

```text
ros2_ws/src/HoloAssist_Movement/config/bin_poses.json
```

Current values:

| Bin | XYZ | RPY degrees |
|---|---|---|
| `bin_1` | `[0.30, 0.00, 0.05]` | `[180.0, 0.0, 0.0]` |
| `bin_2` | `[-0.30, 0.00, 0.05]` | `[180.0, 0.0, 0.0]` |
| `bin_3` | `[0.30, -0.20, 0.05]` | `[180.0, 0.0, 0.0]` |
| `bin_4` | `[0.30, -0.10, 0.05]` | `[180.0, 0.0, 0.0]` |

Reference:

- `ros2_ws/src/HoloAssist_Movement/config/bin_poses.json:1`

## Perception Checks

Expected consumers:

```bash
ros2 topic echo /detections --once
ros2 topic echo /detections_all --once
ros2 topic echo /holoassist/perception/debug_image --once
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
```

Important warning:

The current checkout expects `/holoassist/perception/april_cube_N_pose`, but no installed first-party node currently publishes those pose topics. Before documenting camera-driven autonomous sorting as fully working, either restore/add the cube pose publisher or document this as a known limitation.

References:

- Tracker expects pose topics: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/aprilcube_tracker_node.py:128`
- Pick service expects pose topics: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:47`
- Current installed perception executables: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:32`

## Unity / Quest Checks

Unity connection:

- `ros_tcp_endpoint` listens on port 10000.
- `scripts/launch.py` prints the Wi-Fi IP to use in Unity.
- `ROSAutoConnect.cs` can probe endpoints and call `ROSConnection.Connect`.

Unity topics:

| Direction | Topic | Current owner |
|---|---|---|
| Unity -> ROS | `/forward_velocity_controller/commands` | `RobotController.cs` |
| Unity -> ROS | `/finger_width_controller/commands` | `RobotController.cs` |
| ROS -> Unity | `/joint_states` | robot driver / controllers |
| ROS -> Unity | `/holoassist/unity/cube_N_pose` | expected Unity cube pose feed |
| Unity -> ROS | `/headset/image_compressed` | `HeadsetStreamPublisher.cs` |
| Unity -> ROS | `/session/status`, `/session/events` | `SessionLogger.cs` |

Reference:

- `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:84`
- `Unity/My project/Assets/Scripts/ROSAutoConnect.cs:20`
- `Unity/My project/Assets/Scripts/RobotController.cs:183`

