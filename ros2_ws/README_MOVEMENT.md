# HoloAssist Movement Bringup (Ollie Branch)

This workspace now includes the movement pieces imported from `john` and shaped for Unity integration (`nic` workflow).

## Packages

- `holoassist_manipulation`
  - `clicked_point_to_moveit` (RViz `/clicked_point` -> MoveIt plan+execute)
- `holoassist_servo_tools`
  - `pose_to_twist_servo` (`PoseStamped` target -> `TwistStamped` servo command)
- `holoassist_unity_bridge`
  - `tcp_endpoint.launch.py` (ROS-TCP endpoint for Unity)
  - `unity_movement_bringup.launch.py` (combined launch)

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  holoassist_manipulation holoassist_servo_tools holoassist_unity_bridge
source install/setup.bash
```

## Launch (Unity + movement)

```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

## Useful launch args

- `enable_click_to_plan:=true|false`
- `ros_ip:=0.0.0.0`
- `ros_tcp_port:=10000`
- `command_frame:=base_link`
- `eef_frame:=tool0`
- `target_topic:=/servo_target_pose`
- `twist_topic:=/servo_node/delta_twist_cmds`

## Notes

- `clicked_point_to_moveit` assumes clicked points are already in `planning_frame`.
- `pose_to_twist_servo` currently controls linear velocity only; angular tracking is left for follow-up work.
- You still need your UR driver + MoveIt + Servo stack running for real motion execution.
