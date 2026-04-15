# ur3_joint_position_controller

This package subscribes to the keyboard teleop topics and publishes UR3 trajectory commands to the scaled joint trajectory controller.

## Subscribed topics

- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/scaled_joint_trajectory_controller/state` (`control_msgs/msg/JointTrajectoryControllerState`)
- `/io_and_status_controller/robot_program_running` (`std_msgs/msg/Bool`)
- `/io_and_status_controller/robot_mode` (`ur_dashboard_msgs/msg/RobotMode`)
- `/io_and_status_controller/safety_mode` (`ur_dashboard_msgs/msg/SafetyMode`)

## Published topics

- `/scaled_joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/ur3_keyboard/robot_command_text` (`std_msgs/msg/String`)

## Behavior

- Uses the current `/joint_states` values as the base robot pose
- Checks that `scaled_joint_trajectory_controller` is active through `controller_manager`
- Checks that the robot program is running, robot mode is `RUNNING`, and safety mode is safe before publishing
- While `a` or `d` is being held, it continuously jogs the selected joint
- When the key is released and the direction messages stop, it publishes a hold-position command so the robot stops
- Sends one-point trajectories with all six joint positions in the controller's expected order:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint

## Build

```bash
cd /home/ollie/ros2_ws
colcon build --packages-select ur3_joint_position_controller
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

## What To Run First

Before this package can move the real robot, these parts need to be running:

1. Start the UR robot driver and controllers

```bash
cd /home/ollie/ros2_ws
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ROBOT_IP>
```

2. Make sure the External Control program is running on the robot teach pendant

- The robot must be powered on
- The robot must be in remote/external control mode
- The External Control URCap program must be started on the pendant

3. Optional but recommended: start MoveIt and RViz

```bash
cd /home/ollie/ros2_ws
source install/setup.bash
ros2 launch /home/ollie/ros2_ws/ur_moveit.launch.py ur_type:=ur3e
```

This gives you:

- `move_group`
- `rviz2`
- MoveIt planning scene and visualization

4. Start the keyboard-to-trajectory bridge node

```bash
cd /home/ollie/ros2_ws
source install/setup.bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

5. Start the keyboard teleop node

```bash
cd /home/ollie/ros2_ws
source install/setup.bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

## Minimum Setup

If you only want the keyboard package to move the robot, the minimum required pieces are:

1. UR driver running
2. External Control program running on the robot
3. `ur3_joint_position_controller` running
4. `ur3_keyboard_teleop` running

MoveIt and RViz are not strictly required for the keyboard package, but they are useful for checking robot state and planning.

## Important

- This package assumes `/scaled_joint_trajectory_controller` is active
- Default jog speed is `0.4` radians per second
- Default control rate is `30.0` Hz
- Default hold timeout is `0.12` seconds
- Default trajectory duration is `0.5` seconds
- If the robot is not ready, the node will publish a clear status message instead of silently sending commands
- You can change the jog behavior with:

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller --ros-args -p jog_speed_rad_per_sec:=0.25 -p control_rate_hz:=40.0 -p hold_timeout_sec:=0.10
```
