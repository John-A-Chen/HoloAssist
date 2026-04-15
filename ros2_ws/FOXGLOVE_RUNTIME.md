# HoloAssist Foxglove Runtime Guide

Last updated: 2026-04-15
Primary branch: `john`

## Runtime Model

Foxglove is the default runtime observability layer.

Transport:
- `foxglove_bridge` (WebSocket)

Runtime aggregation:
- `holoassist_foxglove/runtime_observability_node`

Manager supervision:
- `holoassist_manager`

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Paths

### Observability-only

```bash
ros2 launch holoassist_foxglove observability.launch.py
```

### Integrated runtime baseline (recommended)

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

### Integrated runtime with optional stacks

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_pointcloud_obstacle:=true \
  enable_ur3_keyboard_teleop:=true \
  enable_ur3_joint_controller:=true \
  enable_robot_demo:=false
```

### Unity bringup path (still valid)

```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

### Bridge only

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Online Tests

### Perception

RealSense + tracker + RViz:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Tracker only (no hardware camera):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

### Motion (keyboard teleop stack)

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

### XR/Unity transport

```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## Simulation Bringup Paths (Merged)

### Fast software-only observability smoke test

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py \
  enable_unity_bringup:=false \
  enable_depth_tracker:=false \
  enable_pointcloud_obstacle:=false \
  enable_ur3_keyboard_teleop:=false \
  enable_ur3_joint_controller:=false
```

### UR fake hardware + keyboard motion

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

Terminal 5:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

### URSim + real UR driver path (optional)

If URSim External Control is running and reachable:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=<URSIM_IP> launch_rviz:=false
```

Then ensure:
- External Control program is started in URSim.
- Controller/output selection matches your motion path.

## What to Expect With No Hardware Plugged In

Expected behavior in offline mode:
- Launch succeeds and observability topics are published.
- Foxglove connection works.
- `/holoassist/diagnostics` reports stale streams (WARN/ERROR) for missing sensors/robot.
- `/holoassist/state/teleop` and `/holoassist/state/planner` generally stay `IDLE`.
- `/holoassist/state/safety` may become `ERROR` if `/joint_states` is missing.
- `/holoassist/events` still logs diagnostics state transitions.

This is normal and useful for validating runtime wiring before hardware availability.

## Core Topics for Foxglove Panels

### Aggregate observability
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`

### Metrics
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

### Perception and scene
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`
- `/clicked_goal_marker`
- `/tf`

### Manager
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Steam Deck / Browser Connection

1. Run runtime on Ubuntu host.
2. Ensure port `8765` reachable from Steam Deck network.
3. Open Foxglove Studio in browser.
4. Connect WebSocket:

```text
ws://<host-ip>:8765
```

Panel mapping baseline:
- `ros2_ws/src/holoassist_foxglove/config/foxglove_layout_spec.yaml`

## Legacy Dashboard Position

`holoassist-dashboard` remains available as fallback.

Forward direction:
- ROS topics + Foxglove panels are the source of truth.
- Legacy dashboards should consume topic-derived summaries, not define new runtime contracts.
