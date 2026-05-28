# HoloAssist Run Guide

## Prerequisites

- Ubuntu 22.04, ROS 2 Humble installed
- `ros2_ws` built: `cd ros2_ws && colcon build --symlink-install`
- UR3e at `192.168.0.194` (default), laptop on same subnet
- Meta Quest 3 on same Wi-Fi subnet as laptop

---

## 1. Calibration

Run **once** after mounting the camera, or any time the camera or robot base moves.

```bash
./scripts/calibrate.sh --robot-ip 192.168.0.194
```

**What it starts:**
1. `ur_onrobot_control start_robot.launch.py` — UR3e driver (real hardware)
2. `holoassist_perception camera_only.launch.py` — RealSense color + depth streams
3. `apriltag_ros apriltag_node` — detects tag family `36h11`, size `0.032 m`, publishes TF
4. `easy_handeye2 calibrate.launch.py` — calibration GUI (`eye_on_base`, tracking `camera_link`)

**Calibration params baked in:**

| Param | Value |
|---|---|
| `calibration_type` | `eye_on_base` |
| `robot_base_frame` | `base_link` |
| `robot_effector_frame` | `tool0` |
| `tracking_base_frame` | `camera_link` |
| `tracking_marker_frame` | `tag36h11:1` |

**Steps:**
1. Attach AprilTag 36h11 ID **1** to the robot gripper (~5 cm from base)
2. On teach pendant: load and run **External Control** (host IP `192.168.0.100`)
3. Move robot through varied poses; click **Take Sample** in the easy_handeye2 GUI each time
4. Collect **15–25 samples** with varied positions and wrist orientations
5. Click **Save** — calibration writes to `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`

**Verify:**
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## 2. Main Launcher (`scripts/launch.sh`)

The launcher sources ROS Humble + `ros2_ws/install/setup.bash`, then runs `launch.py`.

### Modes

#### Fake hardware (no robot)
```bash
./scripts/launch.sh
```

#### Real robot, teleop only
```bash
./scripts/launch.sh --robot-ip 192.168.0.194
```
- Switches to `forward_velocity_controller` + `finger_width_controller` for velocity teleop
- On teach pendant: load and run **External Control**

#### Real robot + perception
```bash
./scripts/launch.sh --robot-ip 192.168.0.194 --perception
```
- Camera auto-detected: RealSense → Logitech Brio → generic webcam → sim fallback
- Reads `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib` and publishes static TF
- Starts `visualize_depth_tracker.launch.py` (RealSense) or `holoassist_4tag_board_4cube.launch.py` (webcam)
- Cube poses published on `/holoassist/perception/april_cube_{1..4}_pose`

#### Real robot + perception + MoveIt autonomous sorting
```bash
./scripts/launch.sh --robot-ip 192.168.0.194 --perception --moveit
```
- Keeps `scaled_joint_trajectory_controller` active (MoveIt's default)
- Starts `full_holoassist_hardware.launch.py` with staggered timer:
  - `t=0s` — MoveIt `move_group`
  - `t=5s` — workspace scene manager (trolley mesh)
  - `t=8s` — coordinate listener + pick/place sequencer
  - `t=10s` — pick/place service + selected-cube adapter
  - `t=12s` — RViz

#### Fake hardware + perception (sim fallback)
```bash
./scripts/launch.sh --perception
```
- No camera found → starts `holoassist_perception sim_april_cube_perception.launch.py`

### Flags Reference

| Flag | Effect |
|---|---|
| `--robot-ip <IP>` | Real robot mode; omit for fake hardware |
| `--perception` | Start camera + AprilTag + cube pose pipeline |
| `--moveit` | Start MoveIt pick/place stack |
| `--fake-gripper` | Real UR arm + fake OnRobot gripper |
| `--no-rviz` | Suppress all RViz windows |
| `--dashboard` | Open E-stop dashboard alongside launcher |
| `--dashboard-fullscreen` | Open dashboard fullscreen |
| `--verbose` | Print all subprocess logs to terminal |

### What `launch.py` starts (always)

| Process | Command |
|---|---|
| UR + OnRobot Driver | `ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2` |
| Trolley Scene Publisher | `holoassist_perception holoassist_trolley_scene_publisher` |
| ROS-TCP Endpoint | `ros_tcp_endpoint default_server_endpoint` on port 10000 |
| IP Beacon | `beacon.py` — UDP multicast so Quest auto-discovers the ROS endpoint |

**Logs:** `/tmp/holoassist_*.log` (one file per process)

---

## 3. Dashboard (`scripts/dashboard.sh`)

PyQt5 E-stop and monitoring dashboard. Designed for 1280×800 (Steam Deck).

```bash
./scripts/dashboard.sh
./scripts/dashboard.sh --fullscreen
```

In normal mode, also starts `dashboard/bridge_server.py` on port **9090** (WebSocket) so a Steam Deck or other client can connect remotely:

```bash
# On Steam Deck / remote machine:
./scripts/dashboard.sh --bridge ws://192.168.0.xxx:9090
```

**Dashboard tabs:** Status · Headset · Graph · Stats · Latency · Session

**E-stop:** Burst-publishes `[0,0,0,0,0,0]` × 10 to `/forward_velocity_controller/commands`, then switches controllers off. Hold **RESUME** 5 s to re-enable.

---

## 4. Unity Cube Relay (topic_tools)

After `scripts/launch.sh --perception`, perception publishes to `/holoassist/perception/april_cube_N_pose`. Unity reads `/holoassist/unity/cube_N_pose`. Run this relay to bridge them:

```bash
for n in 1 2 3 4; do
  ros2 run topic_tools relay \
    /holoassist/perception/april_cube_${n}_pose \
    /holoassist/unity/cube_${n}_pose &
done
```

> **Note:** This relay is only needed if you are not using `--moveit` mode (which starts the full `holoassist_perception` stack that already includes the adapter). For standalone perception + Unity visualisation without MoveIt, run the relay above.

---

## 5. MoveIt Pick-and-Place Service

Once `--moveit` is running, trigger a pick with:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

**Bin positions** (`config/bin_poses.json`, all in `base_link` frame):

| Bin | XYZ (m) |
|---|---|
| `bin_1` | `[0.30, 0.00, 0.05]` |
| `bin_2` | `[-0.30, 0.00, 0.05]` |
| `bin_3` | `[0.30, -0.20, 0.05]` |
| `bin_4` | `[0.30, -0.10, 0.05]` |

**Monitor state:**
```bash
ros2 topic echo /holoassist/movement/state
# QUEUED → PLANNING → PLANNED → EXECUTING → COMPLETE
ros2 topic echo /pick_place/status
```

---

## 6. Expected Startup Checklist

After `./scripts/launch.sh --robot-ip 192.168.0.194 --perception`:

- [ ] UR driver active: `ros2 control list_controllers` shows `joint_state_broadcaster [active]`
- [ ] Controller switched: `forward_velocity_controller [active]`
- [ ] TCP endpoint live: port 10000 accepting connections
- [ ] Calibration TF: `ros2 run tf2_ros tf2_echo base_link camera_link` returns a transform
- [ ] Cube poses publishing: `ros2 topic hz /holoassist/perception/april_cube_1_pose` → ~20 Hz when visible
- [ ] Unity: set ROS IP to the `WiFi IP` printed at launch startup, port 10000, then hit Play

---

## 7. Standalone Testing

```bash
# Check AprilTag detections
ros2 topic echo /detections_all --once

# Check cube 1 pose
ros2 topic echo /holoassist/perception/april_cube_1_pose --once

# Check full TF chain
ros2 run tf2_ros tf2_echo base_link tag36h11:10

# Simulate a joint velocity command (no headset)
ros2 topic pub --rate 50 /forward_velocity_controller/commands \
  std_msgs/Float64MultiArray '{data: [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]}'

# Manually send robot to XYZ (MoveIt must be running)
ros2 topic pub --once /holoassist/movement/target_point geometry_msgs/msg/Point \
  "{x: 0.20, y: -0.30, z: 0.15}"
```
