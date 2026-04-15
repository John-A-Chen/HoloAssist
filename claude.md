# HoloAssist Architecture Notes (Foxglove Era)

This repository now treats **Foxglove** as the primary runtime observability layer.

## Source-of-Truth Runtime Stack

- ROS 2 runtime transport: `foxglove_bridge`
- Runtime observability adapters: `ros2_ws/src/holoassist_foxglove`
- System mode/heartbeat diagnostics: `ros2_ws/src/holoassist_manager`
- Motion + teleop controller packages:
  - `ros2_ws/src/holoassist_movement`
  - `ros2_ws/src/ur3_keyboard_teleop`
  - `ros2_ws/src/ur3_joint_position_controller`

## Main Runtime Launches

- `ros2 launch holoassist_foxglove observability.launch.py`
- `ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py`
- `ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py`

`unity_movement_bringup.launch.py` now includes Foxglove observability integration by default.

## Dashboard Naming Direction

Legacy UI path:

- `holoassist-dashboard` (retained for compatibility/testing)

Forward naming and architecture:

- `holoassist_foxglove` (ROS package)
- Foxglove Studio layouts/panels for operations, debugging, and demos

When adding new runtime status/telemetry, publish ROS topics first so they are immediately visible in Foxglove.

## Observability Topic Contracts

Primary aggregated topics:

- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`
- `/holoassist/metrics/*`

Manager topics:

- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Developer Rule of Thumb

If a runtime signal is only visible in terminal logs today, add or adapt it into a ROS topic (prefer standard message types) so Foxglove can visualize it live and in recorded playback.
