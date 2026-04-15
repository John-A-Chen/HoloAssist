# holoassist_foxglove

`holoassist_foxglove` is the Foxglove-first runtime observability package for the HoloAssist stack.

It provides:

- Runtime aggregation node (`runtime_observability_node`)
- Unified diagnostics/topic freshness reporting on `/holoassist/diagnostics`
- Event stream on `/holoassist/events`
- Runtime state summaries on `/holoassist/state/*`
- Network and transport latency metrics on `/holoassist/metrics/*`
- Heartbeat publishers compatible with `holoassist_manager`
- Launch files that integrate `foxglove_bridge` in the runtime flow

## Launch Files

- `observability.launch.py`: observability node + optional `holoassist_manager` + optional `foxglove_bridge`
- `holoassist_foxglove_runtime.launch.py`: full runtime convenience launch (unity bridge, optional perception/teleop stack, observability)

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch holoassist_foxglove observability.launch.py
```

Or for the integrated runtime:

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```
