# HoloAssist

**XR-Based Human–Robot Collaboration Framework**

HoloAssist integrates a Meta Quest 3 headset with a UR3e collaborative robot for real-time teleoperation, autonomous pick-and-place, and spatial AR visualisation.

---

## System Overview

```
Brio/RealSense Camera
    ↓ AprilTag detection (30 Hz)
Perception Node  ──→  Cube Poses + Bin Verification
    ↓
MoveIt Pick-and-Place ──→ UR3e + OnRobot RG2 Gripper
    ↓
ROS-TCP Endpoint :10000
    ↓
Quest 3 (Unity) — AR overlay, teleop controls
```

## Subsystems

| Subsystem | Lead | Status |
|---|---|---|
| Perception (AprilTag cubes) | John | Working |
| Autonomous Pick-and-Place | Oliver | Working |
| Teleoperation | Nic | Working |
| Visualisation (Quest 3 / Unity) | Sebastian | In testing |

## Quick Start

```bash
cd ~/git/RS2-HoloAssist/main

# URSim (simulated robot)
./launch.sh --robot-ip 192.168.56.101 --fake-gripper --perception --moveit --dashboard

# Real robot
./launch.sh --robot-ip 192.168.0.194 --perception --moveit --dashboard

# Fully fake (no robot)
./launch.sh --perception --dashboard
```

See [Running the System](running.md) for the full guide.
