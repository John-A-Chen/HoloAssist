# HoloAssist Foxglove Runtime

This workspace now supports a Foxglove-first runtime observability path via:

- `holoassist_foxglove` (new)
- `holoassist_manager` (integrated from `seb`)
- `ur3_keyboard_teleop`, `ur3_joint_position_controller`, `holoassist_movement` (integrated from `ollie`)

## Architecture

`foxglove_bridge` is the live transport layer for Foxglove Studio.

`holoassist_foxglove/runtime_observability_node` aggregates runtime state and publishes:

- `/holoassist/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- `/holoassist/events` (`std_msgs/String`)
- `/holoassist/state/teleop` (`std_msgs/String`)
- `/holoassist/state/planner` (`std_msgs/String`)
- `/holoassist/state/safety` (`std_msgs/String`)
- `/holoassist/state/runtime` (`std_msgs/String`)
- `/holoassist/metrics/*` (latency/frequency/network metrics)

`holoassist_manager` consumes heartbeat topics and publishes:

- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Paths

### 1) Observability only

```bash
ros2 launch holoassist_foxglove observability.launch.py
```

### 2) Integrated runtime (recommended baseline)

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

Example with additional stacks:

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_pointcloud_obstacle:=true \
  enable_ur3_keyboard_teleop:=true \
  enable_ur3_joint_controller:=true
```

### 3) Existing unity bringup with Foxglove enabled

```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

This now includes `holoassist_foxglove/observability.launch.py` by default.

## Steam Deck / Remote Client

1. Start runtime on Ubuntu host.
2. Ensure host firewall allows Foxglove bridge port (default `8765`).
3. From Steam Deck browser:
   - Open Foxglove Studio.
   - Connect via WebSocket: `ws://<host-ip>:8765`
4. Load panels according to `src/holoassist_foxglove/config/foxglove_layout_spec.yaml`.

## Layout Spec

Use:

- `ros2_ws/src/holoassist_foxglove/config/foxglove_layout_spec.yaml`

This defines panel/topic mapping for:

- camera feeds
- 3D scene (TF/markers/point cloud)
- teleop/planner state
- safety/diagnostics
- metrics/latency/network
- event stream

## Migration Note

`holoassist-dashboard` remains available as a legacy UI path.

Forward path:

- runtime observability name: `holoassist_foxglove`
- launch path: `holoassist_foxglove/*.launch.py`
- dashboards and operator workflows should target Foxglove panels/topics first
