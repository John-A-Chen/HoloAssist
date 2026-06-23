# Current HoloAssist Architecture

This document describes the current `main` checkout. It should replace the older website architecture text that still references `holo_assist_depth_tracker`, `holo_assist_depth_tracker_sim`, and `moveit_robot_control`.

## Runtime Shape

The system is now organised around two first-party ROS packages plus upstream robot, gripper, calibration, ROS-Unity, and Unity dependencies.

```text
Unity / Quest 3
  |  ROS-TCP Connector package
  |  port 10000
  v
ros_tcp_endpoint submodule
  |
  +-> /forward_velocity_controller/commands
  +-> /finger_width_controller/commands
  +-> /joint_states
  +-> /holoassist/unity/cube_N_pose
  |
scripts/launch.py
  |
  +-> ur_onrobot_control start_robot.launch.py
  |     +-> UR ROS driver from apt / upstream dependencies
  |     +-> onrobot_driver submodule
  |     +-> ur_onrobot_description + ur_onrobot_moveit_config submodule packages
  |
  +-> holoassist_perception perception.launch.py
  |     +-> realsense2_camera or webcam publisher
  |     +-> apriltag_ros
  |     +-> aprilcube_tracker_node
  |
  +-> holoassist_movement movement.launch.py
        +-> ur_onrobot_moveit_config/ur_onrobot_moveit.launch.py
        +-> workspace_scene_manager
        +-> coordinate_listener
        +-> pick_place_sequencer
        +-> pick_place_service_node
        +-> selected_cube_to_moveit_target_node
```

Source references:

- First-party package names and dependencies: `ros2_ws/src/HoloAssist_Perception/package.xml:3`, `ros2_ws/src/HoloAssist_Movement/package.xml:4`.
- Installed perception executables: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:32`.
- Installed movement executables: `ros2_ws/src/HoloAssist_Movement/CMakeLists.txt:28`.
- Top-level launcher defaults: `scripts/launch.py:327`, `scripts/launch.py:335`, `scripts/launch.py:340`, `scripts/launch.py:348`.
- Top-level driver command: `scripts/launch.py:433`.
- Top-level perception command: `scripts/launch.py:523`.
- Top-level MoveIt command: `scripts/launch.py:581`.

## First-Party ROS Packages

### `holoassist_perception`

Path: `ros2_ws/src/HoloAssist_Perception`

Role:

- Owns camera launch wrappers.
- Owns webcam fallback publisher.
- Starts `apriltag_ros`.
- Provides the RGB debug overlay node.
- Provides the pick-cube-to-bin service node.
- Provides the selected-cube-to-MoveIt adapter.
- Provides the trolley marker publisher.
- Defines `holoassist_perception/srv/PickCubeToBin`.
- Defines `holoassist_perception/msg/CubePerceptionStatus`.

Current executables installed by CMake:

- `aprilcube_tracker_node`
- `holoassist_webcam_image_publisher`
- `holoassist_trolley_scene_publisher`
- `pick_place_service_node`
- `selected_cube_to_moveit_target_node`

References:

- Package declaration: `ros2_ws/src/HoloAssist_Perception/package.xml:3`.
- Interface generation: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:11`.
- Executable installs: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:32`.
- Main perception launch file: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:1`.

Working integration:

`perception.launch.py` starts `apriltag_ros` and `aprilcube_tracker_node` for the AprilTag cube tracking flow. The working perception interface provides `/holoassist/perception/april_cube_N_pose` for RViz, Unity, and autonomous pick-place consumers.

References:

- Launch starts tracker only: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:61`.
- Tracker subscribes to cube pose topics: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/aprilcube_tracker_node.py:128`.
- Pick service subscribes to cube pose topics: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:47`.
- Perception executable installs: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:32`.

### `holoassist_movement`

Path: `ros2_ws/src/HoloAssist_Movement`

Role:

- Owns MoveIt topic execution.
- Owns trolley/workspace planning scene management.
- Owns the pick/place sequencer.
- Defines `holoassist_movement/msg/TargetRPY`.
- Depends on `holoassist_perception` for `PickCubeToBin`.
- Wraps the upstream `ur_onrobot_moveit_config` launch.

Current executables installed by CMake:

- `holoassist_movement`
- `coordinate_listener`
- `workspace_scene_manager`
- `pick_place_sequencer`

References:

- Package declaration: `ros2_ws/src/HoloAssist_Movement/package.xml:4`.
- Dependency on `holoassist_perception`: `ros2_ws/src/HoloAssist_Movement/package.xml:18`.
- Interface generation: `ros2_ws/src/HoloAssist_Movement/CMakeLists.txt:8`.
- Executable installs: `ros2_ws/src/HoloAssist_Movement/CMakeLists.txt:28`.
- Main launch file: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:18`.

## Robot Stack

The robot stack is a composition of upstream and local integration packages:

- `ur_onrobot_control`: launches the UR3e plus OnRobot RG2 control stack.
- `ur_onrobot_description`: combined UR + OnRobot robot description.
- `ur_onrobot_moveit_config`: MoveIt SRDF/config for the combined arm and gripper.
- `onrobot_description`: gripper URDF/xacro.
- `onrobot_driver`: OnRobot gripper hardware interface.

The first three packages come from the `ros2_ws/src/UR_OnRobot_ROS2` submodule. The last two are separate submodules.

Important local patch:

`UR_OnRobot_ROS2` is dirty because this checkout carries local uncommitted patches for `use_fake_gripper_hardware`. That patch lets the UR arm connect through RTDE while only the OnRobot gripper serial port is skipped.

References:

- Local-patch explanation: `docs/ur_onrobot_ros2_local_patches.md:3`.
- Reason for the patch: `docs/ur_onrobot_ros2_local_patches.md:6`.
- Current top-level launcher passes the flag: `scripts/launch.py:442`.
- Movement launch forwards the flag to xacro/MoveIt: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:51`, `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:145`, `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:180`.

## Top-Level Launcher

Current user entry point:

```bash
./scripts/launch.sh [flags]
```

`scripts/launch.sh` sources ROS Humble and `ros2_ws/install/setup.bash`, then runs `scripts/launch.py`.

References:

- Shell wrapper source/install: `scripts/launch.sh:7`.
- Python handoff: `scripts/launch.sh:10`.

Current defaults:

- Fake hardware if `--robot-ip` is omitted.
- Perception is enabled by default.
- MoveIt is enabled by default.
- Dashboard is enabled by default.
- RViz is enabled unless `--no-rviz` is passed.

References:

- Fake/real robot decision: `scripts/launch.py:359`.
- `--no-perception` opt-out and default true: `scripts/launch.py:327`, `scripts/launch.py:335`.
- `--no-moveit` opt-out and default true: `scripts/launch.py:331`, `scripts/launch.py:335`.
- Dashboard opt-out and default true: `scripts/launch.py:340`, `scripts/launch.py:348`.

## Perception Flow

The intended perception data flow is:

```text
RealSense or webcam
  -> /camera/camera/color/image_raw
  -> apriltag_ros
  -> /detections_all or /detections
  -> cube pose fusion publisher
  -> /holoassist/perception/april_cube_N_pose
  -> pick_place_service_node, aprilcube_tracker_node, Unity relay/consumer
```

Current implementation status:

- `perception.launch.py` starts `apriltag_ros` using `apriltag_cubes.yaml`.
- `perception.launch.py` starts `aprilcube_tracker_node`.
- `aprilcube_tracker_node` consumes cube pose topics and produces `/holoassist/perception/debug_image`.
- Cube poses are published on `/holoassist/perception/april_cube_N_pose` for downstream consumers.

References:

- AprilTag node launch: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:49`.
- AprilTag config default: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:36`.
- Tracker launch: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:61`.
- Tracker debug image publisher: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/aprilcube_tracker_node.py:144`.
- Cube config values: `ros2_ws/src/HoloAssist_Perception/config/cubes.yaml:1`.
- Tag detector config values: `ros2_ws/src/HoloAssist_Perception/config/apriltag_cubes.yaml:1`.

## MoveIt / Pick-Place Flow

Current MoveIt flow:

```text
/holoassist/pick_cube_to_bin
  -> pick_place_service_node
  -> /pick_place/mode and /pick_place/command
  -> pick_place_sequencer
  -> /holoassist/movement/target_pose or /holoassist/movement/target_point
  -> coordinate_listener
  -> MoveIt services
  -> /scaled_joint_trajectory_controller/joint_trajectory
  -> UR3e + RG2
```

References:

- Pick service type and name: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:61`.
- Pick service command publishers: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:58`.
- Sequencer default topics: `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/pick_place_sequencer.py:29`.
- Coordinate listener input/output topics: `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py:2023`.
- Movement launch sequence: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:282`.

## Unity / Quest Flow

Unity uses the Robotics ROS-TCP Connector from `ThirdParty/ROS-TCP-Connector` and connects to `ros_tcp_endpoint` on port 10000.

Important Unity scripts:

- `ROSAutoConnect.cs`: connects to the ROS TCP endpoint.
- `RobotController.cs`: publishes teleoperation velocity and gripper commands.
- `JointStateSubscriber.cs`, `RobotHUD.cs`, `RobotDataPanel.cs`: consume `/joint_states`.
- `CubePoseSubscriber.cs`: subscribes to `/holoassist/unity/cube_N_pose`.
- `HeadsetStreamPublisher.cs`: publishes `/headset/image_compressed`.
- `SessionLogger.cs`: publishes `/session/status` and `/session/events`.

References:

- Unity ROS package source paths: `Unity/My project/Packages/manifest.json:13`.
- Endpoint connect logic: `Unity/My project/Assets/Scripts/ROSAutoConnect.cs:20`, `Unity/My project/Assets/Scripts/ROSAutoConnect.cs:36`.
- Teleop command topics: `Unity/My project/Assets/Scripts/RobotController.cs:183`.
- Cube subscriber prefix: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:10`.
- Cube subscriber topic construction: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:84`.
- Headset image topic: `Unity/My project/Assets/Scripts/HeadsetStreamPublisher.cs:24`.
- Session topics: `Unity/My project/Assets/Scripts/SessionLogger.cs:54`.
