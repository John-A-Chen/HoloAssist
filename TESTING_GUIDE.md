# HoloAssist Testing Guide — Sebastian

**Date written:** 2026-05-26  
**Tester:** Sebastian (Visualisation subsystem)  
**Hardware you need:** Brio camera, 3 AprilTag cubes, Quest 3 headset  
**Repo:** https://github.com/John-A-Chen/HoloAssist  

---

## What Works (as of today)

| Feature | Status |
|---|---|
| UR3e driver — real robot | Working |
| UR3e driver — URSim Docker | Working — needs Polyscope External Control manually triggered |
| OnRobot RG2 gripper — real | Working |
| OnRobot RG2 gripper — URSim fake | Working with `--fake-gripper` |
| Brio camera AprilTag detection | Working — auto-detected by launch script |
| Cube pose estimation (3 cubes) | Working |
| Cube persistence in RViz | Working — freeze at last seen position when tags leave view |
| Bin verification (camera-side) | Working — checks cube XY against bin_poses.json at 20 Hz |
| Trolley scene in RViz | Working — correct 180° orientation |
| RViz visualisation | Working |
| MoveIt pick/place stack | Working |
| E-stop dashboard | Working |
| Quest 3 ↔ ROS bridge (port 10000) | Bridge runs — **tonight's test** |
| UDP IP beacon for Quest auto-discovery | Running automatically |

---

## Cube AprilTag IDs

Physical size: **40 mm** cubes, **32 mm** printed tags, family **36h11**.

| Cube | Tag IDs |
|---|---|
| Cube 1 | 10, 11, 12, 13, 14, 15 |
| Cube 2 | 16, 17, 18, 19, 20, 21 |
| Cube 3 | 22, 23, 24, 25, 26, 27 |

At least one face must be visible for a cube to be detected. Once detected, the position **freezes** in RViz even if tags go out of view.

---

## Prerequisites

```bash
cd ~/git/RS2-HoloAssist/main

# Source and build (first time or after changes)
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install && cd ..
```

If the workspace is already built, just sourcing is enough — `./launch.sh` does it automatically.

---

## Option A — Real Robot

**Robot IP:** `192.168.0.194` (check the robot's touchscreen if different)

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --robot-ip 192.168.0.194 --perception --moveit --dashboard
```

At startup you'll see:

```
=================================================================
  HoloAssist ROS 2 Launcher
=================================================================
  Mode:        REAL ROBOT
  Robot IP:    192.168.0.194
  WiFi IP:     10.84.45.XX  <-- set this in Unity ROS Settings
  Gripper:     real
  Perception:  on (Brio 4K)
  MoveIt:      on
  Dashboard:   on  (bridge on :9090)
```

**Write down the WiFi IP** — you need it in the Quest app.

RViz opens after ~15–20 s showing the trolley, arm, and cubes (once detected).

---

## Option B — URSim (no physical robot)

URSim runs a Polyscope simulator at `192.168.56.101` over a VirtualBox host-only network.

### Terminal 2 — start URSim (leave it running)

```bash
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  -p 50001-50002:50001-50002 \
  --name ursim \
  universalrobots/ursim_e-series
```

Open Polyscope once it starts:
- **Browser (noVNC):** http://localhost:6080
- **VNC client:** `localhost:5900`

In Polyscope: navigate to **Run Program → External Control**. **Do not click Play yet.**

### Terminal 1 — launch

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --robot-ip 192.168.56.101 --fake-gripper --perception --moveit --dashboard
```

> **`--fake-gripper` is required with URSim.** URSim doesn't expose port 54321 (OnRobot
> serial relay). Without it, the controller hangs waiting for a Modbus connection.

Once you see `✓  UR + OnRobot Driver`, click **Play** in Polyscope. The driver log confirms:

```
Robot connected to reverse interface. Ready to receive control commands.
```

---

## Option C — Fully Fake (no robot at all)

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --perception --dashboard
```

Useful for testing perception and the Quest connection without any robot.

---

## Quest 3 Connection

1. Quest and laptop must be on the **same WiFi network**.
2. Note the `WiFi IP` printed at launcher startup.
3. In the Unity app: **ROS Settings → Host IP** → enter that address.
4. Port: **10000** (ROS-TCP-Endpoint).
5. UDP beacon broadcasts automatically — the app may auto-discover the host.

Expected: Quest renders the robot arm, trolley, and cube positions as AR overlays in the physical space.

---

## Bin Verification

After a cube is sorted, the perception node checks whether it landed in the right bin. Works at 20 Hz on any cube that has been detected at least once — even if tags are hidden after the sort.

### Check it live

```bash
ros2 topic echo /holoassist/perception/april_cube_1_bin_check
```

### What the message looks like

```
cube_id=1 sorted=true  bin=bin_1 distance_xy_m=0.034 margin_m=0.080
cube_id=2 sorted=false bin=none  distance_xy_m=0.241 margin_m=0.080
```

- `sorted=true` — cube XY is within **8 cm** of a known bin centre (in robot frame)
- Z ignored — robust to small calibration height errors
- Bins come from `ros2_ws/src/moveit_robot_control/config/bin_poses.json`
- Requires hand-eye calibration to be loaded; skips silently with a warning if not

**If you're getting false negatives** (cube is in the bin but `sorted=false`), increase the margin:

```yaml
# ros2_ws/src/holo_assist_depth_tracker/config/cubes.yaml
bin_xy_margin_m: 0.10   # 10 cm
```

---

## Useful Flags

| Flag | Effect |
|---|---|
| `--robot-ip <IP>` | Connect to real UR3e or URSim at this IP |
| `--fake-gripper` | Skip Modbus/serial gripper comms — required for URSim |
| `--perception` | Start Brio camera + AprilTag cube detection |
| `--moveit` | Start MoveIt pick/place stack |
| `--dashboard` | Open e-stop dashboard (Qt window, bridge on :9090) |
| `--no-rviz` | Skip RViz |
| `--verbose` | Print full node logs to terminal |

---

## Monitoring Logs

```bash
# UR driver — RTDE connection status
tail -f /tmp/holoassist_ur_onrobot_driver.log | grep -E "connected|error|failed|RTDE"

# AprilTag detection + bin checks
tail -f /tmp/holoassist_cube_pose.log

# MoveIt planning + execution
tail -f /tmp/holoassist_moveit.log
```

---

## Stopping Everything

`Ctrl+C` in the launcher terminal. It will SIGTERM → wait 2 s → SIGKILL → pkill sweep for zombies.

If processes remain after that:

```bash
pkill -9 -f ur_ros2_control_node
pkill -9 -f move_group
pkill -9 -f robot_state_publisher
```

---

## Known Issues

**URSim WiFi disconnect**
RTDE (ports 30001/30004) drops if WiFi goes away. Fix:
1. Click **Stop Program** in Polyscope
2. `Ctrl+C` the launcher, wait for cleanup
3. Re-run the launch command
4. Click **Play** again in Polyscope

**"Receive program failed: 192.168.56.1:50002 connection refused"**
Polyscope clicked Play before the driver was ready. Restart the launcher, wait for `✓  UR + OnRobot Driver`, then Play.

**Cubes not visible in RViz**
They only appear after first detection. Point the Brio at a cube face until it appears — then it persists even if occluded.

**Arm not visible in RViz**
Robot description takes ~15 s to publish. If nothing after 30 s, check the driver log.

**High CPU after Ctrl+C**
Zombie `ur_ros2_control_node` from a previous session. Run `pkill -9 -f ur_ros2_control_node`.

---

## System Architecture

```
Brio Camera
    │  /camera/camera/color/image_raw
    ▼
AprilTag Detector
    │  /detections_all
    ▼
Cube Pose Node ──────────────────────────────────────────────────────┐
    │  /holoassist/perception/april_cube_N_pose  (PoseStamped)       │
    │  /holoassist/perception/april_cube_N_bin_check (sorted=true?)  │
    │  (cubes freeze in RViz on tag loss)                            │
    ▼                                                                │
MoveIt Pick-and-Place                                               │
    │  scaled_joint_trajectory_controller                            │
    ▼                                                                │
UR3e Driver ◄────────────────────────────────────────────────────────┘
    │  /joint_states, TF
    ▼
RViz  +  ROS-TCP Endpoint :10000
                │
         Quest 3 (Unity)
          AR overlay + teleop controls
```

---

*Questions → John*
