# holoassist_manager

Central runtime supervisor for HoloAssist mode + heartbeat diagnostics.

## Operating Modes

| Mode | Description |
|---|---|
| `MANUAL` | Human-in-the-loop teleoperation |
| `HYBRID` | Combined manual + autonomous behavior |

## Supervised Heartbeats

Expected heartbeat topics:
- `/xr_interface/heartbeat`
- `/perception/heartbeat`
- `/planning/heartbeat`
- `/control/heartbeat`
- `/rviz/heartbeat`
- `/unity_bridge/heartbeat`
- `/robot_bringup/heartbeat`

`holoassist_foxglove/runtime_observability_node` publishes compatible heartbeats.

## ROS Interface

Published:
- `~/mode` (`std_msgs/String`, latched)
- `~/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

Services:
- `~/set_manual` (`std_srvs/Trigger`)
- `~/set_hybrid` (`std_srvs/Trigger`)
- `~/get_mode` (`std_srvs/Trigger`)
- `~/system_status` (`std_srvs/Trigger`)

Common absolute names when node name is `holoassist_manager`:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`
- `/holoassist_manager/set_manual`
- `/holoassist_manager/set_hybrid`
- `/holoassist_manager/get_mode`
- `/holoassist_manager/system_status`

Parameters:
- `initial_mode` (default `MANUAL`)
- `status_publish_rate` (default `1.0` Hz)
- `heartbeat_timeout_sec` (default `5.0` sec)

## Build and Run

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holoassist_manager --symlink-install
source install/setup.bash

ros2 launch holoassist_manager manager.launch.py
```

## Foxglove Integration

With observability launch enabled:
- manager diagnostics appear in Foxglove on `/holoassist_manager/diagnostics`
- manager mode appears on `/holoassist_manager/mode`
