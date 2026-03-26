# holoassist_manager

Central supervisor node for the HoloAssist framework. It manages system-wide operating modes and monitors the health of all subsystem nodes via heartbeat signals.

## Operating Modes

| Mode     | Description |
|----------|-------------|
| `MANUAL` | Full human-in-the-loop teleoperation |
| `HYBRID` | Combined manual + autonomous behaviour |

## Supervised Subsystems

The manager monitors heartbeats from the following subsystems (configurable timeout):

| Subsystem       | Heartbeat Topic               |
|-----------------|-------------------------------|
| xr_interface    | `/xr_interface/heartbeat`     |
| perception      | `/perception/heartbeat`       |
| planning        | `/planning/heartbeat`         |
| control         | `/control/heartbeat`          |
| rviz            | `/rviz/heartbeat`             |
| unity_bridge    | `/unity_bridge/heartbeat`     |
| robot_bringup   | `/robot_bringup/heartbeat`    |

Each subsystem is expected to publish a `std_msgs/String` on its heartbeat topic. If no message is received within the configured timeout the subsystem is flagged as **down**.

## ROS Interface

### Published Topics

| Topic              | Type                                  | Description                    |
|--------------------|---------------------------------------|--------------------------------|
| `~/mode`           | `std_msgs/String`                     | Current operating mode (latched) |
| `~/diagnostics`    | `diagnostic_msgs/DiagnosticArray`     | Per-subsystem health status    |

### Services

| Service            | Type                | Description                        |
|--------------------|---------------------|------------------------------------|
| `~/set_manual`     | `std_srvs/Trigger`  | Switch to MANUAL mode              |
| `~/set_hybrid`     | `std_srvs/Trigger`  | Switch to HYBRID mode              |
| `~/get_mode`       | `std_srvs/Trigger`  | Query the current mode             |
| `~/system_status`  | `std_srvs/Trigger`  | Aggregated alive/down report       |

### Parameters

| Parameter               | Type     | Default    | Description                                      |
|-------------------------|----------|------------|--------------------------------------------------|
| `initial_mode`          | `string` | `"MANUAL"` | Starting mode (`MANUAL` or `HYBRID`)             |
| `status_publish_rate`   | `double` | `1.0`      | Diagnostics publish rate in Hz                   |
| `heartbeat_timeout_sec` | `double` | `5.0`      | Seconds before a subsystem is considered down    |

## Building

```bash
cd ros2_ws
colcon build --packages-select holoassist_manager
```

## Running

```bash
source ros2_ws/install/setup.bash

# Run the node directly
ros2 run holoassist_manager manager_node

# Or use the launch file (loads config/manager_params.yaml)
ros2 launch holoassist_manager manager.launch.py
```

## Example Usage

```bash
# Switch to hybrid mode
ros2 service call /holoassist_manager/set_hybrid std_srvs/srv/Trigger

# Query current mode
ros2 service call /holoassist_manager/get_mode std_srvs/srv/Trigger

# Check system health
ros2 service call /holoassist_manager/system_status std_srvs/srv/Trigger

# Monitor mode topic
ros2 topic echo /holoassist_manager/mode
```
