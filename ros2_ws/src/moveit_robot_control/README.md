# moveit_robot_control

MoveIt execution package for the HoloAssist UR3e + OnRobot RG2 workflow.

This package now supports both real and fake hardware through the same top-level launcher flow, while keeping trajectory/planning behavior aligned across modes.

## Current Role in the Stack

`moveit_robot_control` is responsible for:

- planning and execution interface (`coordinate_listener`)
- workspace/trolley planning scene publishing (`workspace_scene_manager`)
- workspace frame publishing (`workspace_frame_tf`)
- pick/place state machine (`pick_place_sequencer`)
- integration with selected-cube target adapter and pick service bridge

It is launched by default through:

```bash
cd /home/john/git/RS2-HoloAssist/main
./launch.sh --moveit [--robot-ip <real_ip>] [--perception]
```

## Launch Files Used Now

### `full_holoassist_hardware.launch.py`

Used by top-level `launch.py` for both modes:

- real mode: `--robot-ip <ip>` provided
- fake mode: no `--robot-ip`, launcher passes relaxed safety gates

This launch starts:

- MoveIt stack (`ur_onrobot_moveit_config`)
- workspace frame/static calibration frame publisher
- workspace scene manager (trolley mesh)
- coordinate listener
- pick/place sequencer
- `/holoassist/pick_cube_to_bin` service bridge
- selected cube to MoveIt target adapter
- RViz (unless disabled)

### `full_holoassist_moveit_sim.launch.py`

Alternative all-in-one fake sim launch that includes fake `ros2_control` bringup and sim perception pipeline in one launch. Useful for package-level debugging.

## Real vs Fake Hardware Behavior

### Real hardware mode

- trajectory topic defaults to:

```text
/scaled_joint_trajectory_controller/joint_trajectory
```

- controller/status checks are enabled
- launcher performs controller switch after driver readiness

### Fake hardware mode (via top-level launcher)

- launcher still uses `full_holoassist_hardware.launch.py`
- launcher sets `require_robot_status:=false`
- launcher sets `require_controller_check:=false`

This preserves command flow while bypassing UR-driver-only status requirements.

### Fake hardware mode (full sim launch)

`full_holoassist_moveit_sim.launch.py` uses:

```text
/joint_trajectory_controller/joint_trajectory
```

and spawns fake-hardware-compatible controllers directly.

## Inputs and Outputs

### Coordinate listener topics

Inputs:

- `/moveit_robot_control/target_point` (`geometry_msgs/Point`)
- `/moveit_robot_control/target_pose` (`geometry_msgs/Pose`)
- `/moveit_robot_control/target` (`moveit_robot_control_msgs/TargetRPY`)
- `/moveit_robot_control/target_joint_state` (`sensor_msgs/JointState`)

Outputs:

- `/moveit_robot_control/status`
- `/moveit_robot_control/state`
- `/moveit_robot_control/debug`
- `/moveit_robot_control/complete`

### Pick/place command surface

Service:

- `/holoassist/pick_cube_to_bin`

Command topics used internally:

- `/pick_place/mode`
- `/pick_place/status`

### Perception handoff

By default, pick/place service bridge reads cube poses from:

- `/holoassist/perception/april_cube_1_pose` ... `_4_pose`

## Key Parameters You Will Actually Touch

In `full_holoassist_hardware.launch.py`:

- `velocity_scale` (default `0.05`)
- `orientation_mode` (default `auto`)
- `pose_goal_planning_time` (default `5.0`)
- `use_rviz` (default `true`)
- `start_pick_place` (default `true`)
- `use_calibrated_workspace` (default `true`)
- `calibration_yaml` (default `~/.holoassist/calibration/calibration_latest.yaml`)

In `coordinate_listener.launch.py`:

- `require_robot_status`
- `require_controller_check`
- `avoid_flange_forearm_clamp`
- `trajectory_topic`

## Current Config Files

Primary runtime configs:

- `config/full_holoassist_hw.yaml`
- `config/full_holoassist_sim.yaml`

These define:

- workspace frame placement
- trolley mesh/collision pose
- target offsets from cube center to approach pose

## Recommended Commands

### Full stack with real robot

```bash
cd /home/john/git/RS2-HoloAssist/main
./launch.sh --robot-ip 192.168.0.194 --perception --moveit --dashboard
```

### Full stack fake mode

```bash
cd /home/john/git/RS2-HoloAssist/main
./launch.sh --perception --moveit --dashboard
```

### Package-only launch (advanced)

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_robot_control full_holoassist_hardware.launch.py robot_ip:=192.168.0.194
```

## Build and Verify

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select moveit_robot_control --symlink-install
source install/setup.bash
```

Runtime checks:

```bash
ros2 node list | rg moveit
ros2 topic list | rg moveit_robot_control
ros2 service list | rg pick_cube_to_bin
```

## Troubleshooting

### MoveIt running but no robot motion

- real mode: ensure External Control is running on teach pendant
- verify active controllers:

```bash
ros2 control list_controllers
```

### Pick service blocks waiting for cubes

If `--moveit` is used without `--perception`, service is up but waits for cube poses. Start perception or publish cube pose topics.

### Wrong planning frame behavior

Check TF chain:

```bash
ros2 run tf2_ros tf2_echo base_link workspace_frame
```

If calibrated workspace file is missing/invalid, run with `use_calibrated_workspace:=false` for debugging.
