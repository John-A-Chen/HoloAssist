# HoloAssist

XR-assisted human-robot collaboration stack for ROS 2 Humble on Ubuntu 22.04.

## Current Status (2026-04-15)

This branch is now Foxglove-first for runtime observability.

Delivered in this branch:
- Unified observability package: `ros2_ws/src/holoassist_foxglove`
- Integrated manager diagnostics/mode supervision: `ros2_ws/src/holoassist_manager`
- Integrated keyboard motion path:
  - `ros2_ws/src/ur3_keyboard_teleop`
  - `ros2_ws/src/ur3_joint_position_controller`
  - `ros2_ws/src/holoassist_movement`
- Integrated Foxglove Bridge into runtime launch flows
- Updated perception + Foxglove docs and runbooks
- Synced Unity source-of-truth project folders from `origin/nic`

## Repository Layout

```text
john/
├── ros2_ws/
│   ├── src/
│   │   ├── holoassist_foxglove/
│   │   ├── holoassist_manager/
│   │   ├── holoassist_unity_bridge/
│   │   ├── holo_assist_depth_tracker/
│   │   ├── holoassist_manipulation/
│   │   ├── holoassist_servo_tools/
│   │   ├── holoassist_movement/
│   │   ├── ur3_keyboard_teleop/
│   │   └── ur3_joint_position_controller/
│   └── *.md runbooks
├── holoassist-dashboard/           # legacy dashboard path (compat/fallback)
├── dashboard/                      # nic branch desktop dashboard snapshot
├── Unity/My project/               # Unity source-of-truth (Assets/Packages/ProjectSettings)
└── claude.md
```

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Runtime Launches

Unified stack entrypoint (default profile = no-hardware-safe):
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py
```

Unified stack full hardware profile:
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py profile:=full_hardware
```

Unified stack with AprilTag tracking (for workbench calibration):
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  enable_apriltag_tracking:=true apriltag_start_camera:=false
```

Recommended integrated runtime:
```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

Observability-only:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

Unity bridge bringup (includes Foxglove observability by default):
```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

Official Foxglove Bridge only:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Test Commands

Perception pipeline:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Perception without camera (software-only smoke test):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

Keyboard motion path:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

Unity ROS TCP endpoint:
```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## What You Should See Without Hardware

Expected when no robot/camera/headset is connected:
- Runtime and Foxglove Bridge launch successfully.
- `/holoassist/diagnostics` is still published.
- Diagnostics show WARN/ERROR for stale missing streams (expected in offline mode).
- `/holoassist/events` logs transitions and missing stream changes.
- `/holoassist/state/teleop` and `/holoassist/state/planner` generally `IDLE`.
- `/holoassist/state/safety` may be `ERROR` when `/joint_states` is unavailable.

This is normal and useful for validating the observability layer before hardware is online.

## Foxglove Connection (Steam Deck / Browser)

1. Run runtime on Ubuntu host.
2. Confirm TCP 8765 is reachable from client network.
3. Open Foxglove Studio in browser on Steam Deck.
4. Connect to:

```text
ws://<host-ip>:8765
```

Layout guidance:
- `ros2_ws/src/holoassist_foxglove/config/foxglove_layout_spec.yaml`
- `ros2_ws/FOXGLOVE_RUNTIME.md`
- `ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md`

## Workspace Pipeline (Bench Plane + Tags)

Bench-understanding adapter node:
- `holo_assist_depth_tracker/workspace_perception_node.py`
- Fits bench plane from depth pointcloud
- Publishes `workspace_frame` TF
- Uses two AprilTag corner frames (when available) to stabilize in-plane orientation
- Publishes cropped + foreground clouds and final object pose/marker

### Plane-only bringup (today)

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_unity_bringup:=false
```

### Plane + AprilTag stabilization (2 corner tags)

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false \
  enable_unity_bringup:=false
```

### Validation topic checklist

```bash
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash

ros2 topic list | rg "^/holoassist/perception/(bench_plane|cropped|foreground|object|workspace)"
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
ros2 topic echo /holoassist/perception/bench_plane_coefficients --once
ros2 topic hz /holoassist/perception/cropped_pointcloud
ros2 topic hz /holoassist/perception/foreground_pointcloud
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```

## Key Runtime Topics

Core observability:
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`
- `/holoassist/metrics/*`

Perception:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

Manager:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Dashboard Migration Direction

Legacy UI remains for compatibility:
- `holoassist-dashboard`

Forward path:
- Foxglove Studio + `holoassist_foxglove` topic contracts

If a runtime status exists only in terminal output, publish it to a ROS topic so Foxglove can visualize and record it.
