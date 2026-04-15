# ur3_keyboard_teleop

Terminal keyboard teleop for selecting UR3 joints and publishing jog direction commands.

## Keys

- `1` to `6`: select active UR3 joint
- `a`: move selected joint anticlockwise
- `d`: move selected joint clockwise
- `s`: stop selected joint
- `q`: quit node

Holding `a`/`d` keeps publishing until key repeat stops.

## Published Topics

- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
- `/ur3_keyboard/command_text` (`std_msgs/msg/String`)

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_keyboard_teleop --symlink-install
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

## Parameters

- `publish_rate_hz` (default `30.0`)
- `hold_timeout_sec` (default `0.08`)

Example:

```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop --ros-args \
  -p publish_rate_hz:=30.0 \
  -p hold_timeout_sec:=0.15
```

## Downstream Usage

Typical consumer:
- `ur3_joint_position_controller` subscribes and converts commands into trajectory points.

Foxglove runtime visibility:
- `/ur3_keyboard/command_text` is surfaced via observability event stream and teleop diagnostics.
