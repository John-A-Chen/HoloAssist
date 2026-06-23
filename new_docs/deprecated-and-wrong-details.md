# Deprecated and Wrong Details Audit

This file lists website details that are deprecated or wrong against the current `main` checkout. Each item includes the current replacement and source references.

## Highest Priority Corrections

### 1. Old first-party ROS package names

Wrong references:

- `site/docs/submodules.md:16` lists `holo_assist_depth_tracker`.
- `site/docs/submodules.md:17` lists `holo_assist_depth_tracker_sim`.
- `site/docs/submodules.md:18` lists `holo_assist_depth_tracker_sim_interfaces`.
- `site/docs/submodules.md:19` lists `moveit_robot_control`.
- `site/docs/submodules.md:20` lists `moveit_robot_control_msgs`.
- `site/docs/submodules.md:21` lists `holoassist_unity_bridge`.
- `site/docs/dependencies.md:90` through `site/docs/dependencies.md:95` repeats the same old package catalog.

Current replacement:

- `holoassist_perception`
- `holoassist_movement`

Current source references:

- `ros2_ws/src/HoloAssist_Perception/package.xml:3`
- `ros2_ws/src/HoloAssist_Movement/package.xml:4`
- `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:11`
- `ros2_ws/src/HoloAssist_Movement/CMakeLists.txt:8`

### 2. Old launch flag model

Wrong references:

- `site/docs/run-guide.md:56` says `./launch.sh` is "Fake hardware (no robot)".
- `site/docs/run-guide.md:68` says `--perception` enables perception.
- `site/docs/run-guide.md:77` says `--moveit` enables MoveIt.
- `site/docs/run-guide.md:95` lists `--perception` and `--moveit` as enable flags.

Current replacement:

- Use `./scripts/launch.sh`.
- Perception and MoveIt are enabled by default.
- Use `--no-perception` and `--no-moveit` to disable them.
- Dashboard is enabled by default; use `--no-dashboard`.

Current source references:

- `scripts/launch.sh:7` and `scripts/launch.sh:10`
- `scripts/launch.py:327`
- `scripts/launch.py:331`
- `scripts/launch.py:335`
- `scripts/launch.py:340`
- `scripts/launch.py:348`

### 3. Old simulated perception fallback

Wrong reference:

- `site/docs/run-guide.md:93` says no camera starts `holo_assist_depth_tracker_sim sim_april_cube_perception.launch.py`.

Current replacement:

- Current `scripts/launch.py` logs no camera and skips perception; it does not start that old sim stack.

Current source reference:

- `scripts/launch.py:464`

### 4. Old MoveIt package and launch file

Wrong references:

- `site/docs/moveit-pick-place.md:3` says package is `moveit_robot_control`.
- `site/docs/autonomous.html:56` says package is `moveit_robot_control`.
- `site/docs/moveit-pick-place.md:54` says key parameters are in `full_holoassist_hardware.launch.py`.
- `site/docs/autonomous.html:154` says parameters are set in `full_holoassist_hardware.launch.py`.
- `site/docs/robot.html:119` says `--moveit` launches `full_holoassist_hardware.launch.py`.

Current replacement:

- Package is `holoassist_movement`.
- Launch file is `ros2 launch holoassist_movement movement.launch.py`.
- Top-level `launch.py` starts the UR + OnRobot driver first, then starts `holoassist_movement movement.launch.py`.

Current source references:

- `ros2_ws/src/HoloAssist_Movement/package.xml:4`
- `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:18`
- `scripts/launch.py:581`

### 5. Old pick service type

Wrong references:

- `site/docs/run-guide.md:165` uses `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`.
- `site/docs/moveit-pick-place.md:31` uses `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`.
- `site/docs/autonomous.html:139` and `site/docs/autonomous.html:144` use `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`.

Current replacement:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

Current source references:

- Service definition: `ros2_ws/src/HoloAssist_Perception/srv/PickCubeToBin.srv:1`
- Service import: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:8`
- Service creation: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/pick_place_service_node.py:61`

### 6. Old movement topic namespace

Wrong references:

- `site/docs/run-guide.md:180` uses `/moveit_robot_control/state`.
- `site/docs/run-guide.md:217` uses `/moveit_robot_control/target_point`.
- `site/docs/moveit-pick-place.md:100` uses `/moveit_robot_control/state`.
- `site/docs/moveit-pick-place.md:102` uses `/moveit_robot_control/debug`.
- `site/docs/moveit-pick-place.md:108` uses `/moveit_robot_control/target_point`.
- `site/docs/moveit-pick-place.md:112` uses `/moveit_robot_control/target_pose`.
- `site/docs/api.html:162`, `site/docs/api.html:167`, and `site/docs/api.html:172` use `/moveit_robot_control/...`.
- `site/docs/autonomous.html:179` and `site/docs/autonomous.html:180` use `/moveit_robot_control/...`.

Current replacement:

- `/holoassist/movement/target`
- `/holoassist/movement/target_point`
- `/holoassist/movement/target_pose`
- `/holoassist/movement/target_joint_state`
- `/holoassist/movement/status`
- `/holoassist/movement/state`
- `/holoassist/movement/debug`
- `/holoassist/movement/complete`
- `/holoassist/movement/stop`

Current source reference:

- `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py:2023`
- Publisher/subscriber setup: `ros2_ws/src/HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py:2234`

### 7. Old perception package, debug topic, and launch files

Wrong references:

- `site/docs/perception.md:3` says package is `holo_assist_depth_tracker`.
- `site/docs/perception.md:66` says debug topic is `/holo_assist_depth_tracker/debug_image`.
- `site/docs/perception.md:70` says RViz config is `depth_tracker_visualization.rviz`.
- `site/docs/perception.md:86` lists `camera_only.launch.py`.
- `site/docs/perception.md:87` lists `visualize_depth_tracker.launch.py`.
- `site/docs/perception.md:88` lists `holoassist_4tag_board_4cube.launch.py`.
- `site/docs/perception.md:89` lists `sim_holoassist_trolley.launch.py`.
- `site/docs/perception.html:59` and `site/docs/perception.html:202` say package is `holo_assist_depth_tracker`.
- `site/docs/perception.html:209` through `site/docs/perception.html:215` use old paths under `main/ros2_ws/src/holo_assist_depth_tracker`.

Current replacement:

- Package: `holoassist_perception`
- Main launch: `perception.launch.py`
- Camera launch: `camera.launch.py`
- RViz: `rviz/holoassist_full.rviz`
- Debug image: `/holoassist/perception/debug_image`

Current source references:

- Package declaration: `ros2_ws/src/HoloAssist_Perception/package.xml:3`
- Launch file: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:1`
- Camera launch include: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:42`
- RViz default: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:32`
- Debug image publisher: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/aprilcube_tracker_node.py:144`

### 8. Perception cube-pose publisher claim is not supported by current installed executables

Wrong or unsafe references:

- `site/docs/perception.md:16` says `CubePoseNode (holoassist_cube_pose_node)` publishes cube poses.
- `site/docs/perception.html:88` says `holoassist_cube_pose_node` is the pipeline stage.
- `site/docs/perception.html:210` says `holoassist_cube_pose_node` exists under the old package path.

Current reality:

- Current `holoassist_perception` CMake does not install a `holoassist_cube_pose_node`.
- Current `perception.launch.py` does not launch one.
- Current `aprilcube_tracker_node` subscribes to cube pose topics and draws the overlay; it is not the pose fusion publisher.

Current source references:

- Installed perception executables: `ros2_ws/src/HoloAssist_Perception/CMakeLists.txt:32`
- Launch only starts `aprilcube_tracker_node`: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:61`
- Tracker subscribes to pose topics: `ros2_ws/src/HoloAssist_Perception/holoassist_perception_nodes/aprilcube_tracker_node.py:128`

Suggested website wording:

> Current integration expects `/holoassist/perception/april_cube_N_pose`, but the current checkout does not install a first-party cube pose publisher. This should be fixed or documented as a known integration gap before claiming full autonomous pick-place from camera detections.

### 9. Unity cube relay details are stale or incomplete

Wrong references:

- `site/docs/run-guide.md:143` through `site/docs/run-guide.md:155` describes a manual `topic_tools relay`.
- `site/docs/perception.html:94` and `site/docs/perception.html:211` reference `cube_pose_relay.py`.
- `site/docs/visualisation.html:469` tells users to confirm `cube_pose_relay.py` is running.

Current reality:

- Unity `CubePoseSubscriber.cs` subscribes to `/holoassist/unity/cube_N_pose`.
- I did not find a current repo executable or script named `cube_pose_relay.py`.
- If the website keeps `/holoassist/unity/cube_N_pose`, it should also document the actual current bridge needed to populate it.

Current source references:

- Unity prefix: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:10`
- Unity topic construction: `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs:84`

### 10. Calibration tag ID mismatch

Wrong references:

- `site/docs/perception.html:156` says calibration tag is ID 0.
- `site/docs/perception.html:166` says attach calibration tag ID 0.

Current replacement:

- Current calibration script uses `tag36h11:1`.

Current source references:

- Printed calibration parameter: `scripts/calibrate.py:100`
- easy_handeye2 launch argument: `scripts/calibrate.py:148`

### 11. Calibration command path mismatch

Wrong references:

- `site/docs/run-guide.md:17` uses `./calibrate.sh`.
- `site/docs/perception.html:168` uses `./calibrate.sh`.

Current replacement:

- Use `./scripts/calibrate.sh`.

Current source references:

- Existing wrapper: `scripts/calibrate.sh:1`
- It sources workspace setup: `scripts/calibrate.sh:5`
- It runs `scripts/calibrate.py`: `scripts/calibrate.sh:8`

### 12. Submodule conversion plan is obsolete

Wrong references:

- `site/docs/submodules.md:40` says `easy_handeye2` just needs `.gitmodules`.
- `site/docs/submodules.md:58` says `Universal_Robots_ROS2_Driver` needs to become a submodule.
- `site/docs/submodules.md:83` says `ROS-TCP-Endpoint` needs a fork first.
- `site/docs/submodules.md:121` says `onrobot_description` and `onrobot_driver` are source files committed directly into the repo.
- `site/docs/submodules.md:146` says `ur_onrobot` is source files committed directly into this repo.
- `site/docs/submodules.md:204` shows target `.gitmodules` that no longer matches this checkout.

Current replacement:

- All current submodules are already listed in `.gitmodules`.
- `UR_OnRobot_ROS2` is already a submodule, but dirty due local fake-gripper patch.
- `Universal_Robots_ROS2_Driver` is not present as a direct submodule in `.gitmodules`.

Current source references:

- `.gitmodules:1`
- `.gitmodules:4`
- `.gitmodules:8`
- `.gitmodules:11`
- `.gitmodules:15`
- `.gitmodules:19`
- `.gitmodules:23`
- `.gitmodules:26`
- Local patch doc: `docs/ur_onrobot_ros2_local_patches.md:3`

### 13. RViz paths are stale

Wrong references:

- `site/docs/rviz-configs.md:8` uses `ros2_ws/src/holo_assist_depth_tracker/config/depth_tracker_visualization.rviz`.
- `site/docs/rviz-configs.md:21` uses `/holo_assist_depth_tracker/debug_image`.
- `site/docs/rviz-configs.md:38` uses `ros2_ws/src/ur_onrobot/ur_onrobot_description/rviz/view_robot.rviz`.
- `site/docs/rviz-configs.md:69` uses the old perception RViz path.

Current replacement:

- Perception full RViz: `ros2_ws/src/HoloAssist_Perception/rviz/holoassist_full.rviz`
- Perception sim RViz: `ros2_ws/src/HoloAssist_Perception/rviz/holoassist_sim.rviz`
- Movement hardware RViz: `ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz`
- Movement robot view: `ros2_ws/src/HoloAssist_Movement/rviz/view_robot.rviz`
- Debug image: `/holoassist/perception/debug_image`

Current source references:

- Perception RViz default: `ros2_ws/src/HoloAssist_Perception/launch/perception.launch.py:32`
- Movement RViz default: `ros2_ws/src/HoloAssist_Movement/launch/movement.launch.py:27`
- Top-level perception RViz path: `scripts/launch.py:412`
- Top-level MoveIt RViz path: `scripts/launch.py:576`

## Lower Priority Internal Docs Drift

These are not website pages but should be fixed later because they disagree with current files:

- `ros2_ws/src/HoloAssist_Perception/README.md` still mentions `camera_only.launch.py`, `visualize_depth_tracker.launch.py`, and `holoassist_4tag_board_4cube.launch.py`, but current launch files are `camera.launch.py` and `perception.launch.py`.
- `ros2_ws/src/HoloAssist_Movement/README.md` mentions launch files such as `full_holoassist_moveit_sim.launch.py`, `full_holoassist_hardware.launch.py`, and `pick_place.launch.py`; current `ros2_ws/src/HoloAssist_Movement/launch` contains only `coordinate_listener.launch.py` and `movement.launch.py`.

