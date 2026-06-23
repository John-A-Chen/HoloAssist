# Website Page Mapping

This maps current repo facts to the existing website pages in `/home/john/git/RS2-HoloAssist/site/docs`.

## `index.html`

Current role:

- Landing page for documentation links.

Recommended update:

- Add a short "Current package names" note so readers understand the website has been updated from the old package split.
- Link to pages using current names: `holoassist_perception`, `holoassist_movement`, `UR_OnRobot_ROS2`, `ROS-TCP-Endpoint`, Unity project.

## `running.html` and `run-guide.md`

Current source of truth:

- `scripts/launch.sh`
- `scripts/launch.py`
- `scripts/calibrate.sh`
- `scripts/calibrate.py`

Recommended update:

- Make `./scripts/launch.sh` the primary command, not `./launch.sh`.
- Document that perception, MoveIt, dashboard, and RViz are enabled by default.
- Replace `--perception`, `--moveit`, and `--dashboard` as enabling flags with opt-out flags:
  - `--no-perception`
  - `--no-moveit`
  - `--no-dashboard`
  - `--no-rviz`
- Keep `--robot-ip`, `--ros-ip`, `--fake-gripper`, `--dashboard-fullscreen`, and `--verbose`.
- Replace old package/service names:
  - Pick service type is `holoassist_perception/srv/PickCubeToBin`.
  - Movement topics are `/holoassist/movement/...`.
  - Perception command is `ros2 launch holoassist_perception perception.launch.py`.
  - MoveIt command is `ros2 launch holoassist_movement movement.launch.py`.

Important current behavior:

- If no camera is detected, current `launch.py` skips perception. It does not start the old sim fallback stack.
- If `--no-moveit` is passed, the launcher switches to `forward_velocity_controller` and `finger_width_controller` for Unity teleop.
- If MoveIt is enabled, it keeps the trajectory controllers active.

References:

- Defaults and opt-out flags: `scripts/launch.py:327`, `scripts/launch.py:335`, `scripts/launch.py:340`, `scripts/launch.py:348`.
- Camera missing behavior: `scripts/launch.py:464`.
- Teleop controller switch only when MoveIt is off: `scripts/launch.py:531`.
- MoveIt launch command: `scripts/launch.py:581`.

## `submodules.md`

Current source of truth:

- `.gitmodules`
- `git ls-files -s` gitlink entries
- package manifests under `ros2_ws/src`

Recommended update:

- Replace "Packages That Need Submodule Conversion" with "Current Submodules".
- Remove direct `Universal_Robots_ROS2_Driver` submodule instructions.
- Mark `easy_handeye2`, `ROS-TCP-Endpoint`, `onrobot_description`, `onrobot_driver`, and `UR_OnRobot_ROS2` as already present submodules.
- Include Unity submodules: `ThirdParty/ROS-TCP-Connector`, `ThirdParty/URDF-Importer`, `ThirdParty/Q3toROS`.
- Include `UR_OnRobot_ROS2` dirty local-patch warning.

References:

- `.gitmodules:1`
- `.gitmodules:4`
- `.gitmodules:8`
- `.gitmodules:11`
- `.gitmodules:15`
- `.gitmodules:19`
- `.gitmodules:23`
- `.gitmodules:26`

## `dependencies.md` and `dependencies.html`

Current source of truth:

- `package.xml` files
- `.gitmodules`
- Unity `manifest.json`

Recommended update:

- First-party packages should list only:
  - `holoassist_perception` 0.1.0
  - `holoassist_movement` 0.1.0
- Upstream ROS submodule packages should list:
  - `ros_tcp_endpoint`
  - `easy_handeye2`
  - `easy_handeye2_msgs`
  - `onrobot_description`
  - `onrobot_driver`
  - `ur_onrobot_control`
  - `ur_onrobot_description`
  - `ur_onrobot_moveit_config`
- Do not list old packages as current:
  - `holo_assist_depth_tracker`
  - `holo_assist_depth_tracker_sim`
  - `holo_assist_depth_tracker_sim_interfaces`
  - `moveit_robot_control`
  - `moveit_robot_control_msgs`
  - `holoassist_unity_bridge`

## `perception.html` and `perception.md`

Current source of truth:

- `holoassist_perception`
- `perception.launch.py`
- `camera.launch.py`
- `apriltag_cubes.yaml`
- `cubes.yaml`
- `aprilcube_tracker_node.py`
- `pick_place_service_node.py`

Recommended update:

- Replace package name with `holoassist_perception`.
- Replace `camera_only.launch.py` with `camera.launch.py`.
- Replace `depth_tracker_visualization.rviz` with `holoassist_full.rviz`.
- Replace `/holo_assist_depth_tracker/debug_image` with `/holoassist/perception/debug_image`.
- Remove or mark `cube_pose_relay.py` as absent unless it is reintroduced.
- Add a "current implementation gap" callout that no installed node currently publishes `/holoassist/perception/april_cube_N_pose`.

Important nuance:

The website can still describe the intended cube-pose pipeline, but it should not claim the current `aprilcube_tracker_node` publishes fused cube poses. It subscribes to them for overlay.

## `rviz-configs.md`

Current source of truth:

- `ros2_ws/src/HoloAssist_Perception/rviz/holoassist_full.rviz`
- `ros2_ws/src/HoloAssist_Perception/rviz/holoassist_sim.rviz`
- `ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz`
- `ros2_ws/src/HoloAssist_Movement/rviz/view_robot.rviz`

Recommended update:

- Replace `ros2_ws/src/holo_assist_depth_tracker/config/depth_tracker_visualization.rviz`.
- Replace `ros2_ws/src/ur_onrobot/ur_onrobot_description/rviz/view_robot.rviz`.
- Mention that current top-level launcher opens the perception RViz early if perception is enabled.
- Mention that MoveIt RViz opens only when MoveIt is standalone without perception.

References:

- Top-level RViz behavior: `scripts/launch.py:409`, `scripts/launch.py:573`.
- Movement RViz default: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:27`.
- Perception RViz default: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:32`.

## `robot.html`

Current source of truth:

- `ur_onrobot_control`
- `holoassist_movement`
- `RobotController.cs`
- top-level launcher controller switch logic

Recommended update:

- Replace `moveit_robot_control` with `holoassist_movement`.
- Replace "launches `full_holoassist_hardware.launch.py`" with "launches `holoassist_movement movement.launch.py` after the UR + OnRobot driver is already running."
- Keep the distinction between teleop controllers and MoveIt trajectory controllers:
  - Unity teleop uses `/forward_velocity_controller/commands` and `/finger_width_controller/commands`.
  - MoveIt uses `/scaled_joint_trajectory_controller/joint_trajectory` and `/finger_width_trajectory_controller/joint_trajectory`.

References:

- Unity teleop topics: `Unity/My project/Assets/Scripts/RobotController.cs:183`.
- MoveIt trajectory topic in launch: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:215`.
- Controller switch logic: `scripts/launch.py:531`.

## `autonomous.html` and `moveit-pick-place.md`

Current source of truth:

- `holoassist_movement`
- `holoassist_perception/srv/PickCubeToBin`
- `movement.launch.py`
- `pick_place_sequencer.py`
- `holoassist_movement.py`
- `pick_place_service_node.py`

Recommended update:

- Replace package name `moveit_robot_control` with `holoassist_movement`.
- Replace service type `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin` with `holoassist_perception/srv/PickCubeToBin`.
- Replace `/moveit_robot_control/target_pose`, `/moveit_robot_control/target_point`, `/moveit_robot_control/state`, and `/moveit_robot_control/debug` with:
  - `/holoassist/movement/target_pose`
  - `/holoassist/movement/target_point`
  - `/holoassist/movement/state`
  - `/holoassist/movement/debug`
- Replace `full_holoassist_hardware.launch.py` with `movement.launch.py`.

References:

- Service type import: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:8`.
- Service name: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:61`.
- Movement topics: `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py:2023`.
- Launch file: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:18`.

## `teleoperation.html`

Current source of truth:

- `Unity/My project/Assets/Scripts/RobotController.cs`
- `scripts/launch.py`
- `ur_onrobot_control`

Recommended update:

- Keep teleop as Unity-controlled velocity commands.
- Mention that teleop mode requires running with `--no-moveit`, otherwise trajectory controllers stay active for MoveIt.
- Document topics:
  - `/forward_velocity_controller/commands`
  - `/finger_width_controller/commands`
  - `/joint_states`
- Keep dashboard/E-stop references if matching dashboard code.

References:

- Teleop topics: `Unity/My project/Assets/Scripts/RobotController.cs:183`.
- Controller switch to teleop only when MoveIt is off: `scripts/launch.py:531`.

## `visualisation.html`

Current source of truth:

- Unity project under `Unity/My project`
- Unity ROS packages from `ThirdParty`
- ROS TCP endpoint from `ros2_ws/src/ROS-TCP-Endpoint`
- `CubePoseSubscriber.cs`
- `ROSAutoConnect.cs`

Recommended update:

- Keep the page focused on Unity/Quest and ROS-TCP.
- Update dependency paths to match `Unity/My project/Packages/manifest.json`.
- Keep `/holoassist/unity/cube_N_pose` as the Unity-facing cube topic because `CubePoseSubscriber` uses it.
- Add a warning that the ROS-side relay/publisher to `/holoassist/unity/cube_N_pose` is not present in the current first-party CMake install.

References:

- Unity package file dependencies: `Unity/My project/Packages/manifest.json:13`.
- Cube topic prefix: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:10`.
- Cube topic subscriptions: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:84`.
- Auto-connect behavior: `Unity/My project/Assets/Scripts/ROSAutoConnect.cs:20`.

## `api.html`

Recommended update:

- Replace stale movement API entries:
  - `/moveit_robot_control/target_pose` -> `/holoassist/movement/target_pose`
  - `/moveit_robot_control/target_point` -> `/holoassist/movement/target_point`
  - `/moveit_robot_control/status` -> `/holoassist/movement/status`
  - add `/holoassist/movement/state`, `/holoassist/movement/debug`, `/holoassist/movement/complete`, `/holoassist/movement/stop`
- Replace stale service type:
  - `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin` -> `holoassist_perception/srv/PickCubeToBin`
- Keep Unity teleop topics:
  - `/forward_velocity_controller/commands`
  - `/finger_width_controller/commands`
  - `/joint_states`
- Mark `/holoassist/perception/april_cube_N_pose` as expected by consumers, but note the current publisher gap.

## `calibration.html`

Current source of truth:

- `scripts/calibrate.sh`
- `scripts/calibrate.py`
- `easy_handeye2`

Recommended update:

- Use `./scripts/calibrate.sh`, not `./calibrate.sh`.
- Current calibration code uses `tag36h11:1`, not ID 0.
- Current calibration code starts `ros2 launch holoassist_perception camera.launch.py`.
- Current calibration code saves as `holoassist_calibration`.

References:

- Calibration command usage in script docstring: `scripts/calibrate.py:14`.
- Camera launch: `scripts/calibrate.py:120`.
- AprilTag detector parameters: `scripts/calibrate.py:129`.
- easy_handeye2 parameters: `scripts/calibrate.py:141`.

