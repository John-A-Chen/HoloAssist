# May 6 — Workspace Calibration Work

## What was done

### 1. UR3e kinematic calibration extracted

The UR driver was emitting a calibration-mismatch warning on every startup because the robot description was using the generic UR3e factory defaults instead of the physical robot's measured DH parameters.

**Extraction command used:**
```bash
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.0.192 \
    target_filename:=ros2_ws/src/ur_onrobot/ur_onrobot_description/config/ur3e_calibration.yaml
```

The extracted file (hash `calib_13258964235769181718`) lives at:
```
ros2_ws/src/ur_onrobot/ur_onrobot_description/config/ur3e_calibration.yaml
```

This file is robot-serial-number specific. It is now the default for all hardware launches — do not overwrite it without re-extracting from the same physical robot.

**Files changed:**
- `ur_onrobot/ur_onrobot_description/config/ur3e_calibration.yaml` — created (extracted calibration)
- `ur_onrobot/ur_onrobot_description/CMakeLists.txt` — added `config` to install targets
- `ur_onrobot/ur_onrobot_control/launch/start_robot.launch.py` — added `kinematics_config` argument, passed to xacro as `kinematics_parameters_file`
- `moveit_robot_control/launch/full_holoassist_hardware.launch.py` — added `kinematics_config` argument, threaded through to robot stack

To override the calibration file at launch time:
```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
    robot_ip:=192.168.0.192 \
    kinematics_config:=/path/to/other_calibration.yaml
```

---

### 2. Board calibration ("bed levelling") workflow

A robot-FK-based interactive calibration routine for aligning `workspace_frame` to the physical board. The robot hovers above each tag corner, the user slides the board until the tag is under the TCP, and the node solves the best-fit rigid transform from 4 (or fewer) FK measurements.

**New files:**
- `holo_assist_depth_tracker/nodes/board_calibration_node.py` — calibration node (~950 lines)
- `holo_assist_depth_tracker/config/board_calibration_params.yaml` — all tunable parameters
- `holo_assist_depth_tracker/launch/board_calibration.launch.py` — one-command launch

**Modified files:**
- `holo_assist_depth_tracker/setup.py` — added `holoassist_board_calibration` entry point
- `moveit_robot_control/moveit_robot_control_node/workspace_frame_tf.py` — marked LEGACY (see below)

**Calibration output** (written after each run):
```
~/.holoassist/calibration/calibration_latest.yaml          ← always use this one
~/.holoassist/calibration/calibration_YYYY-MM-DD_HH-MM-SS.yaml
```

The YAML is directly loadable by `workspace_frame_tf` and contains the full `base_link → workspace_frame` transform plus residual metadata.

---

### 3. workspace_frame geometry fixes

Updated robot Z offset and Y derivation in both workspace config files.

**Files changed:**
- `holo_assist_depth_tracker/config/workspace.yaml` — `robot_z_m: -0.010`, corrected Y comment
- `holo_assist_depth_tracker/config/workspace_perception_params.yaml` — `robot_pose_z_m: -0.010`, `robot_pose_follow_workspace: false`, `robot_pose_from_tag_pair: false`, corrected comments

---

## How to run

### Robot kinematic calibration (one-time, robot must be on)
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.0.192 \
    target_filename:=$(ros2 pkg prefix ur_onrobot_description --share)/config/ur3e_calibration.yaml
# then rebuild:
colcon build --packages-select ur_onrobot_description --symlink-install
```

### Board calibration (run before each session if board moved)

**Terminal 1 — robot:**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py \
    ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.192 launch_rviz:=false
# then run External Control on the teach pendant
```

**Terminal 2 — MoveIt:**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
    ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=false \
    robot_ip:=192.168.0.192 launch_rviz:=false launch_servo:=false
```

**Terminal 3 — board calibration:**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch holo_assist_depth_tracker board_calibration.launch.py \
    verify_with_camera:=false
```

Follow the interactive prompts. The node stays alive after calibration to hold the `base_link → workspace_frame` static TF. Keep Terminal 3 open.

**To apply a saved calibration after a reboot (without re-running):**
```bash
ros2 run moveit_robot_control workspace_frame_tf \
    --ros-args --params-file ~/.holoassist/calibration/calibration_latest.yaml
```

---

## Architecture notes

### TF ownership — workspace_frame

Only one node can publish the `workspace_frame` child TF at a time:

| Node | TF published | When to use |
|------|-------------|-------------|
| `workspace_board_node` | `camera_color_optical_frame → workspace_frame` | Normal runtime (dynamic, from AprilTags) |
| `workspace_frame_tf` | `base_link → workspace_frame` | **LEGACY** — sim launches, or loading a saved calibration YAML |
| `board_calibration_node` | `base_link → workspace_frame` | During and after board calibration |

Never run `workspace_board_node` and `board_calibration_node` simultaneously.

### workspace_frame_tf is now legacy

`moveit_robot_control/moveit_robot_control_node/workspace_frame_tf.py` is still used by:
- `full_holoassist_moveit_sim.launch.py`
- `full_holoassist_gazebo_sim.launch.py`
- Loading saved calibration YAMLs on restart

On hardware, `board_calibration_node` replaces it. The file has a LEGACY header.

### Reachability note

The UR3e's 500 mm working radius means the front two board tags (tags 0 and 1) may be outside reach depending on board placement. The calibration node skips unreachable tags automatically and works with however many it can measure (minimum 2). Adjusting `approx_ws_y_m` in `board_calibration_params.yaml` changes where the hover targets are computed — make it less negative to bring targets closer to the robot.
