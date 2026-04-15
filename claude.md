# HoloAssist Architecture Notes (Foxglove Era)

Last updated: 2026-04-15
Branch: `john`

## What Was Completed

This branch has been consolidated into a Foxglove-first runtime flow.

Completed integration work:
- Added `holoassist_foxglove` package for runtime observability aggregation and Foxglove-oriented topic contracts.
- Integrated `holoassist_manager` into the runtime flow for mode + heartbeat diagnostics.
- Integrated motion/teleop packages from other branches:
  - `holoassist_movement`
  - `ur3_keyboard_teleop`
  - `ur3_joint_position_controller`
- Wired `foxglove_bridge` into launch paths:
  - `holoassist_foxglove/launch/observability.launch.py`
  - `holoassist_foxglove/launch/holoassist_foxglove_runtime.launch.py`
  - `holoassist_unity_bridge/launch/unity_movement_bringup.launch.py`
- Added/updated documentation for Foxglove runtime, discovery, and perception validation.
- Synced Unity source-of-truth content from `origin/nic` into this branch:
  - `Unity/My project/Assets`
  - `Unity/My project/Packages`
  - `Unity/My project/ProjectSettings`
- Added `.gitattributes` for Unity mesh/artifact handling (`*.dae`, `*.apk` with LFS).
- Tightened `.gitignore` to exclude generated Unity and local dependency repo noise.

Recent push history on `origin/john`:
- `98e132d` feat: integrate manager and UR3 teleop packages from seb/ollie branches
- `e364f9e` feat: add foxglove-first observability package and runtime launch flow
- `43cde86` chore: ignore Unity generated build and local config artifacts
- `6980535` chore: sync Unity/XR workspace from nic and clean local artifact ignores

## Source-of-Truth Runtime Stack

Core packages for runtime bringup and observability:
- `ros2_ws/src/holoassist_foxglove`
- `ros2_ws/src/holoassist_manager`
- `ros2_ws/src/holoassist_unity_bridge`
- `ros2_ws/src/holo_assist_depth_tracker`
- `ros2_ws/src/holoassist_manipulation`
- `ros2_ws/src/holoassist_servo_tools`
- `ros2_ws/src/holoassist_movement`
- `ros2_ws/src/ur3_keyboard_teleop`
- `ros2_ws/src/ur3_joint_position_controller`

## Main Launch Entry Points

Build:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Foxglove observability only:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

Integrated runtime baseline:
```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

Unity bringup with observability enabled:
```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

Official Foxglove bridge only:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Test Commands

Perception only (no camera launch):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

Perception + RealSense camera:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Keyboard motion path:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

XR/Unity transport endpoint:
```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## Key Observability Topics

Aggregated runtime topics:
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

Manager topics:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

Perception topics:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

## No-Hardware Expectations

If no robot/camera/headset hardware is online:
- Runtime still starts and publishes Foxglove-facing observability topics.
- `/holoassist/diagnostics` will show WARN/ERROR for stale streams (expected).
- `/holoassist/state/teleop` and `/holoassist/state/planner` will usually show `IDLE`.
- `/holoassist/state/safety` may show `ERROR` when `/joint_states` is absent.
- Foxglove remains useful for bringup validation (topic graph, diagnostics transitions, event stream).

## Dashboard Naming Direction

Legacy retained path:
- `holoassist-dashboard` (compatibility/fallback)

Forward path:
- `holoassist_foxglove` package + Foxglove Studio layouts

Rule:
- New runtime status should be published to ROS topics first, then visualized in Foxglove.
- Avoid terminal-only operational state when the same signal can be topicized.
