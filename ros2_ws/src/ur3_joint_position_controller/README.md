# ur3_joint_position_controller

Keyboard-command bridge that converts selected-joint jog intent into UR3 joint trajectory commands.

## Topics

Subscribed:
- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/scaled_joint_trajectory_controller/state` (`control_msgs/msg/JointTrajectoryControllerState`)
- `/io_and_status_controller/robot_program_running` (`std_msgs/msg/Bool`)
- `/io_and_status_controller/robot_mode` (`ur_dashboard_msgs/msg/RobotMode`)
- `/io_and_status_controller/safety_mode` (`ur_dashboard_msgs/msg/SafetyMode`)

Published:
- `/scaled_joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/ur3_keyboard/robot_command_text` (`std_msgs/msg/String`)

## Behavior

- Uses current `/joint_states` as motion baseline.
- Validates robot/controller readiness before publishing.
- While key command is active, jogs selected joint continuously.
- On key release timeout, publishes hold-position command.
- Publishes human-readable command status on `/ur3_keyboard/robot_command_text`.

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_joint_position_controller --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

or

```bash
ros2 launch ur3_joint_position_controller ur3_joint_position_controller.launch.py
```

## Minimum Bringup

1. UR driver path running (real robot, URSim, or fake hardware).
2. `scaled_joint_trajectory_controller` active.
3. `ur3_joint_position_controller` running.
4. `ur3_keyboard_teleop` running.

## Fake Hardware Example

Terminal 1:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=0.0.0.0 \
  use_fake_hardware:=true fake_sensor_commands:=true launch_rviz:=false
```

Terminal 2:
```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate forward_velocity_controller
```

Terminal 3:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

Terminal 4:
```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

## Foxglove Visibility

These keyboard-motion signals are consumed by observability stack and visible in Foxglove:
- `/ur3_keyboard/command_text`
- `/ur3_keyboard/robot_command_text`
- `/holoassist/events`
- `/holoassist/diagnostics`

## Useful Parameters

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller --ros-args \
  -p jog_speed_rad_per_sec:=0.25 \
  -p control_rate_hz:=40.0 \
  -p hold_timeout_sec:=0.10
```
