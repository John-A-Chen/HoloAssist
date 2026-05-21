# HoloAssist - Working Perception + Cube Pipeline

Last verified: 2026-05-08

Everything below has been tested end-to-end: real UR3e robot, RealSense D435I camera, AprilTag cubes detected and appearing in Unity on Quest 3.

---

## Network Setup

All devices on the UR3e's dedicated router (192.168.0.x subnet):

| Device | IP | Connection |
|---|---|---|
| UR3e robot | 192.168.0.192 | Ethernet to router |
| Laptop Ethernet | 192.168.0.100 | Ethernet to router (robot comms) |
| Laptop WiFi | 192.168.0.102 | WiFi on robot's router (Quest comms) |
| Quest 3 | DHCP | WiFi on robot's router |

- Teach pendant External Control host IP: `192.168.0.100`
- Unity ROS Settings IP: laptop WiFi IP (e.g. `192.168.0.102`)

---

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04)
- Unity 6.3 LTS
- Both workspaces built:

```bash
# Nic's workspace
cd ~/git/RS2-HoloAssist/nic/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Main workspace (perception packages)
cd ~/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

- Camera calibration completed (see Calibration section below)

---

## Quick Start

### Teleoperation only (no perception)

```bash
cd ~/git/RS2-HoloAssist/nic

# Fake hardware (no robot needed)
./launch.sh

# Real robot
./launch.sh --robot-ip 192.168.0.192
```

### With perception (AprilTag cubes in Unity)

```bash
cd ~/git/RS2-HoloAssist/nic

# Real robot + camera + cube detection
./launch.sh --robot-ip 192.168.0.192 --perception

# Fake hardware + simulated perception
./launch.sh --perception
```

Then:
1. On teach pendant: load and run **External Control** (host IP: 192.168.0.100)
2. Hit **Play** in Unity

### Dashboard (separate terminal)

```bash
cd ~/git/RS2-HoloAssist/nic
./dashboard.sh --fullscreen
```

---

## What `launch.sh --perception` Starts

The `--perception` flag adds these on top of the base launch:

| Process | What it does |
|---|---|
| Camera Calibration TF | Publishes `base_link -> camera_link` from saved calibration file |
| RealSense Camera | `camera_only.launch.py` — starts depth + colour streams |
| AprilTag Detector | `apriltag_ros` — detects 36h11 tags, publishes TF per tag |
| Cube Pose Node | `holoassist_cube_pose_node` — merges 6 face tags per cube into one centre pose, publishes PoseStamped |
| Cube Pose Relay | `cube_pose_relay.py` — relays poses to `/holoassist/unity/cube_{1-4}_pose` for Unity |

### Cube tag IDs

Each physical cube has 6 AprilTag faces (36h11 family, 32mm):

| Cube | Tag IDs | Unity topic |
|---|---|---|
| Cube 1 | 10, 11, 12, 13, 14, 15 | `/holoassist/unity/cube_1_pose` |
| Cube 2 | 16, 17, 18, 19, 20, 21 | `/holoassist/unity/cube_2_pose` |
| Cube 3 | 22, 23, 24, 25, 26, 27 | `/holoassist/unity/cube_3_pose` |
| Cube 4 | 28, 29, 30, 31, 32, 33 | `/holoassist/unity/cube_4_pose` |

Face order per cube (6 tags in sequence): +X, -X, +Y, -Y, +Z, -Z.

### TF chain (hardware mode)

```
world -> base_link -> camera_link -> camera_color_optical_frame -> tag36h11:XX
                  (calibration)   (RealSense driver)           (apriltag_ros)
```

The cube pose node looks up each tag directly in `base_link` frame via this chain. No workspace board tags (0-3) needed.

---

## Camera Calibration

The calibration determines where the camera is relative to the robot base (`base_link -> camera_link`). You need to recalibrate if the camera or robot moves.

### Running calibration

```bash
cd ~/git/RS2-HoloAssist/nic
./calibrate.sh --robot-ip 192.168.0.192
```

This starts the robot driver, camera, AprilTag detector, and easy_handeye2 GUI in one command.

### Calibration steps

1. Stick an AprilTag (36h11, ID 0, 32mm) on the robot gripper, approximately 5cm from the gripper base
2. Run `./calibrate.sh --robot-ip 192.168.0.192`
3. On teach pendant: load and run **External Control**
4. Use the teach pendant to move the robot to different poses (freedrive or jog)
5. At each pose, click **Take Sample** in the easy_handeye2 GUI
6. Collect **15-25 samples** with varied positions and orientations (tilt the wrist, move across the workspace)
7. Click **Save**

Calibration saves to: `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`

### Critical calibration parameters

These are baked into `calibrate.sh` / `calibrate.py`:

| Parameter | Value | Why |
|---|---|---|
| calibration_type | eye_on_base | Camera is fixed in the world, not on the robot |
| tracking_base_frame | **camera_link** | Must be `camera_link`, NOT `camera_color_optical_frame` |
| tracking_marker_frame | tag36h11:0 | The calibration tag on the gripper |
| robot_base_frame | base_link | Robot base |
| robot_effector_frame | tool0 | Robot end-effector |

The `tracking_base_frame:=camera_link` is critical. The RealSense driver publishes `camera_link -> camera_color_optical_frame`, so the calibration must target `camera_link` to avoid a TF parent conflict.

### Verifying calibration

After calibration, check in RViz that the camera model appears at the correct physical location relative to the robot:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

The translation should roughly match the physical camera position relative to the robot base.

### Solver

The easy_handeye2 backend uses the **Park** solver (not Tsai-Lenz, which gives bad results). This was changed in:
`main/ros2_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py` (line 62, `algorithm = 'Park'`)

---

## Unity Setup

### Required scripts on GameObjects

| Script | Attach to | Inspector fields |
|---|---|---|
| ROSAutoConnect | Any GameObject | rosPort=10000 |
| JointStateSubscriber | `ur3e_rg2` root | (auto-discovers joints) |
| RobotController | Any GameObject | Input Actions, robotBase, collisionGuard |
| CubePoseSubscriber | Any GameObject | robotBase = `ur3e_rg2` transform |

### ROSAutoConnect

Auto-discovers the ROS endpoint. Scans in order:
1. localhost (127.0.0.1) — for Unity Editor on same machine
2. 192.168.0.101-110 (skips .100 = robot Ethernet)
3. All local machine IPs

Falls back to the IP configured in Robotics > ROS Settings if nothing found.

### CubePoseSubscriber

- Subscribes to `/holoassist/unity/cube_{1-4}_pose`
- Spawns coloured cubes (red, green, blue, orange) at detected positions
- Positions relative to `robotBase` transform
- 3-second timeout hides cubes when perception stops
- Set `robotBase` to the `ur3e_rg2` GameObject in Inspector

---

## Debugging

### Check if tags are being detected

```bash
ros2 topic echo /detections --once
```

### Check if cube poses are being published

```bash
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/unity/cube_1_pose --once
```

### Check TF chain

```bash
# Calibration working?
ros2 run tf2_ros tf2_echo base_link camera_link

# Full chain to a tag?
ros2 run tf2_ros tf2_echo base_link tag36h11:10

# View full TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf in current directory
```

### Common issues

| Problem | Cause | Fix |
|---|---|---|
| No detections | Camera not seeing tags | Check camera angle, lighting, tag size |
| Detections but no cube poses | cube_pose_node not running | Check `ros2 node list \| grep cube` |
| Cube poses but not in Unity | Relay not running or Unity not connected | Check relay node + ROSAutoConnect |
| TF trees disconnected | Calibration TF not published | Re-run calibration, check calib file exists |
| Camera in wrong position in RViz | Bad calibration | Recalibrate with more samples |
| Port 50002 already in use | Previous driver still running | `fuser -k 50002/tcp` then relaunch |
| Port 10000 already in use | Previous endpoint still running | launch.py auto-kills this |

---

## File Locations

| File | Purpose |
|---|---|
| `nic/launch.py` | Main launcher (robot + teleop + perception) |
| `nic/launch.sh` | Shell wrapper (sources ROS before launch.py) |
| `nic/calibrate.py` | Calibration launcher (robot + camera + apriltag + easy_handeye2) |
| `nic/calibrate.sh` | Shell wrapper for calibrate.py |
| `nic/ros2_ws/cube_pose_relay.py` | Relays cube poses to Unity-friendly topics |
| `nic/Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` | Unity subscriber for cube poses |
| `nic/Unity/My project/Assets/Scripts/ROSAutoConnect.cs` | Auto-discovers ROS endpoint |
| `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib` | Saved calibration result |
| `main/ros2_ws/src/holo_assist_depth_tracker/` | Perception package (cube_pose_node, camera launch, etc.) |
| `main/ros2_ws/src/easy_handeye2/` | Calibration package (Park solver patched) |
