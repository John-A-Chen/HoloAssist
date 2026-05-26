# Running the System

## Prerequisites

```bash
# Source ROS 2 Humble and workspace
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/main/ros2_ws/install/setup.bash
```

Build (first time or after source changes):

```bash
cd ~/git/RS2-HoloAssist/main/ros2_ws
colcon build --symlink-install
```

---

## Launch Flags

| Flag | Effect |
|---|---|
| `--robot-ip <IP>` | Connect to real UR3e or URSim at this IP |
| `--fake-gripper` | Skip Modbus/serial gripper comms — required for URSim |
| `--perception` | Start Brio/RealSense camera + AprilTag cube detection |
| `--moveit` | Start MoveIt pick-and-place stack |
| `--dashboard` | Open e-stop dashboard (Qt window, bridge on :9090) |
| `--no-rviz` | Skip RViz |
| `--verbose` | Print full node logs to terminal |

---

## Option A — Real Robot

**Default IP:** `192.168.0.194` (check the robot's touchscreen if different)

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --robot-ip 192.168.0.194 --perception --moveit --dashboard
```

At startup the launcher prints:

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

**Note the WiFi IP — enter it in the Quest Unity app under ROS Settings.**

RViz opens after ~15–20 s showing the trolley, arm, and cubes.

---

## Option B — URSim (Docker Simulator)

Use this when the physical robot is not available. URSim runs a Polyscope simulator at `192.168.56.101` over a VirtualBox host-only network.

### Terminal 2 — start URSim

```bash
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  -p 50001-50002:50001-50002 \
  --name ursim \
  universalrobots/ursim_e-series
```

Wait for `Polyscope started`, then open Polyscope:

- **Browser (noVNC):** http://localhost:6080
- **VNC client:** `localhost:5900`

In Polyscope: navigate to **Run Program → External Control**. **Do not click Play yet.**

### Terminal 1 — main launch

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --robot-ip 192.168.56.101 --fake-gripper --perception --moveit --dashboard
```

!!! warning "--fake-gripper is required with URSim"
    URSim does not expose port 54321 (OnRobot serial relay). Without `--fake-gripper`,
    the controller manager blocks waiting for the gripper Modbus connection.

Once the driver shows `✓  UR + OnRobot Driver`, click **Play** in Polyscope. The log will confirm:

```
Robot connected to reverse interface. Ready to receive control commands.
```

---

## Option C — Fully Fake Hardware

No robot IP = fully simulated arm. Perception still runs.

```bash
cd ~/git/RS2-HoloAssist/main
./launch.sh --perception --dashboard
```

---

## Quest 3 Connection

1. Connect the Quest and laptop to the **same WiFi network**.
2. Note the `WiFi IP` printed by the launcher at startup.
3. In the Unity app: open **ROS Settings** → set host IP to that address.
4. The launcher broadcasts a UDP beacon for auto-discovery.
5. ROS-TCP-Endpoint listens on **port 10000**.

---

## Stopping

`Ctrl+C` in the launcher terminal. The launcher:

1. SIGTERM → 2 s wait → SIGKILL all child processes
2. Sweeps for zombie nodes with `pkill -9 -f` (ur_ros2_control_node, move_group, etc.)

If anything remains:

```bash
pkill -9 -f ur_ros2_control_node
pkill -9 -f move_group
```

---

## Monitoring Logs

All logs go to `/tmp/holoassist_*.log`.

```bash
# UR driver connection
tail -f /tmp/holoassist_ur_onrobot_driver.log | grep -E "connected|error|failed|RTDE"

# AprilTag cube detection
tail -f /tmp/holoassist_cube_pose.log

# MoveIt
tail -f /tmp/holoassist_moveit.log
```

---

## Known Issues

**URSim WiFi disconnect**
If the laptop's WiFi drops, RTDE (ports 30001/30004) is lost. Fix:

1. Click **Stop Program** in Polyscope
2. `Ctrl+C` the launcher and wait for cleanup
3. Re-run the launch command
4. Click **Play** in Polyscope again

**"Receive program failed: 192.168.56.1:50002 connection refused"**
The driver failed before port 50002 was ready. Polyscope connected back too early. Restart the launcher first, then Play.

**High CPU after Ctrl+C**
Zombie ur_ros2_control_node processes from a previous session. Run:
```bash
pkill -9 -f ur_ros2_control_node
```

**Arm not visible in RViz**
Robot description takes ~15 s to publish. If nothing appears after 30 s:
```bash
tail -f /tmp/holoassist_ur_onrobot_driver.log
```

**Cubes not visible in RViz**
Cubes only appear after first detection. Point the Brio at a cube face until it appears, then it persists.
