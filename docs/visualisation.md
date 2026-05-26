# Visualisation

**Lead:** Sebastian  
**Hardware:** Meta Quest 3  
**Engine:** Unity (ROS-TCP-Connector)

---

## Architecture

```
ROS 2 (host machine)
  └─ ROS-TCP-Endpoint :10000  ←── Unity app (Quest 3)
  └─ UDP Beacon (auto-discovery)
```

The Quest connects over WiFi to the ROS-TCP-Endpoint. The Unity app subscribes to ROS topics and renders AR overlays in the physical space.

---

## Connecting the Quest

1. **Same network:** Laptop and Quest must be on the same WiFi.
2. **Note the WiFi IP** — printed at launcher startup:
   ```
   WiFi IP:     10.84.45.XX  <-- set this in Unity ROS Settings
   ```
3. In the Quest Unity app: **ROS Settings → Host IP** → enter the WiFi IP.
4. Port: **10000** (ROS-TCP-Endpoint default).
5. The launcher broadcasts a UDP beacon — the app may auto-discover the host.

---

## Topics Consumed by Unity

| Topic | Type | Description |
|---|---|---|
| `/holoassist/perception/april_cube_N_pose` | `PoseStamped` | Live cube positions |
| `/holoassist/perception/april_cube_N_bin_check` | `String` | Bin verification result |
| `/joint_states` | `JointState` | Robot arm joint angles (for AR overlay) |
| `/tcp_pose_broadcaster/pose` | `PoseStamped` | End-effector TCP pose |

---

## RViz

RViz runs automatically with the launcher (unless `--no-rviz`). Key visualisations:

- **Trolley mesh** — `UR3eTrolley(1).dae` rendered at correct orientation
- **Robot arm** — live joint state from `robot_state_publisher`
- **Cube markers** — coloured cubes at detected positions (freeze on tag loss)
- **TF tree** — camera → workspace → robot chain after calibration

### Cube marker colours

| Cube | Colour |
|---|---|
| Cube 1 | Red (RGBA 1.0, 0.0, 0.0) |
| Cube 2 | Green (RGBA 0.0, 1.0, 0.0) |
| Cube 3 | Blue (RGBA 0.0, 0.3, 1.0) |
| Cube 4 | Orange (RGBA 1.0, 0.7, 0.0) |

Frozen cubes render at 50% alpha. Active cubes render at full alpha.

---

## E-stop Dashboard

The dashboard (`--dashboard` flag) opens a Qt window with:

- Per-process status (green = running, red = failed)
- E-stop button (sends `/robot/stop` command to the UR driver)
- Bridge server on port 9090 (rosbridge WebSocket)

```bash
./launch.sh --dashboard          # dashboard window
./launch.sh --dashboard-fullscreen  # fullscreen mode
```

---

## Troubleshooting

**Quest can't connect**
- Confirm laptop and Quest are on the same WiFi subnet
- Check `./launch.sh` printed WiFi IP matches what you entered
- Try `ros2 topic list` — if ROS-TCP-Endpoint is running you'll see topics

**Arm not visible in AR**
- `joint_states` requires the UR driver to be connected (real robot or URSim)
- With fully fake hardware (`./launch.sh --perception` only), joint states still publish at zero position

**Cube positions drifting**
- Ensure hand-eye calibration is loaded (see [Calibration](calibration.md))
- Without calibration, cubes publish in camera frame, not robot frame — positions will not align with AR overlay
