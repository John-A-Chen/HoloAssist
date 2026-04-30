# holoassist_foxglove

Foxglove-first runtime observability package for HoloAssist.

## What It Provides

- Runtime aggregation node: `runtime_observability_node`
- Object pose adapter node: `obstacle_to_object_pose_adapter`
- Unified diagnostics: `/holoassist/diagnostics`
- Event stream: `/holoassist/events`
- Runtime state summaries: `/holoassist/state/*`
- Metrics: `/holoassist/metrics/*`
- Heartbeat publishers compatible with `holoassist_manager`
- Launch integration with `foxglove_bridge`

## Key Metrics Topics

- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/bbox_hz`
- `/holoassist/metrics/pointcloud_points`
- `/holoassist/metrics/debug_image_age_s`
- `/holoassist/metrics/pointcloud_age_s`
- `/holoassist/metrics/perception_pipeline_ok` (`1.0` = fresh, `0.0` = stale)
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

## Launch Files

- `launch/observability.launch.py`
  - starts runtime observability node
  - optional `holoassist_manager`
  - optional `foxglove_bridge`

- `launch/holoassist_foxglove_runtime.launch.py`
  - integrated runtime convenience launch
  - optional perception/motion stacks
  - optional Unity bringup + observability

- `launch/apriltag_workbench.launch.py`
  - optional AprilTag tracker + optional RealSense launch for workbench calibration

## Quick Start

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch holoassist_foxglove observability.launch.py
```

Integrated runtime:

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

## Useful Launch Arguments

From `observability.launch.py`:
- `enable_foxglove_bridge` (default `true`)
- `enable_manager` (default `true`)
- `enable_tf_marker_bridge` (default `false`)
- `diagnostics_rate_hz` (default `1.0`)
- `stale_timeout_s` (default `2.5`)
- `unity_tcp_host` / `unity_tcp_port`
- `foxglove_bridge_host` / `foxglove_bridge_port`

## Expected Offline Behavior

Without robot/camera/headset online:
- package still publishes diagnostics/events/state topics
- diagnostics will report stale streams
- this is expected and useful for pipeline bringup validation

## Recommended Foxglove Layout Spec

- `config/foxglove_layout_spec.yaml`
