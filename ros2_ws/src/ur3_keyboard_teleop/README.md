# ur3_keyboard_teleop

This package reads keyboard input from the terminal and publishes ROS 2 topics for a UR3 joint teleop flow.

## Keys

- `1` to `6`: select the active UR3 joint
- `a`: move the selected joint anticlockwise
- `d`: move the selected joint clockwise
- `s`: stop the selected joint
- `q`: quit the teleop node

Holding `a` or `d` will keep publishing motion commands while the terminal repeats the key. When the key repeat stops, the node automatically publishes stop.

## Published topics

- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
  - Joint number from `1` to `6`
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
  - `-1` = anticlockwise
  - `0` = stop
  - `1` = clockwise
- `/ur3_keyboard/command_text` (`std_msgs/msg/String`)
  - Human-readable status text

## Build

```bash
cd /home/ollie/ros2_ws
colcon build --packages-select ur3_keyboard_teleop
source install/setup.bash
```

## Run

```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

or

```bash
ros2 launch ur3_keyboard_teleop keyboard_joint_teleop.launch.py
```

## Useful parameters

- `publish_rate_hz`
  - How fast the node republishes the active motion command while a key is held
  - Default: `30.0`
- `hold_timeout_sec`
  - How long the node waits without seeing another repeated `a` or `d` before it publishes stop
  - Default: `0.08`

Example:

```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop --ros-args -p publish_rate_hz:=30.0 -p hold_timeout_sec:=0.15
```

## Example subscriber logic

Another package can subscribe to:

- `/ur3_keyboard/selected_joint` to know which joint is active
- `/ur3_keyboard/joint_direction` to know which way to move it

Then it can map:

- joint `1` to `shoulder_pan_joint`
- joint `2` to `shoulder_lift_joint`
- joint `3` to `elbow_joint`
- joint `4` to `wrist_1_joint`
- joint `5` to `wrist_2_joint`
- joint `6` to `wrist_3_joint`
