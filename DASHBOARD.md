# HoloAssist Dashboard

E-stop, debug console, and session monitoring for the HoloAssist robotic sorting system. Runs natively on a Steam Deck OLED as a dedicated operator panel, connected to the laptop over WebSocket.

## Quick Reference (at uni, no internet)

You've already done the one-time setup. Here's what to do each session:

### 1. Laptop — start the robot stack + bridge

Open your normal terminals for the robot:
```bash
./launch.sh --robot-ip 192.168.0.194    # or ./launch.sh for fake hardware
```

Then open **one more terminal** for the bridge:
```bash
./bridge.sh
```

Check the laptop's WiFi IP (you'll need it for the Deck):
```bash
hostname -I
# Look for the 192.168.0.x address — that's the robot network
```

### 2. Steam Deck — connect to robot WiFi and launch

On the Deck (Desktop Mode → Konsole):

1. Connect to the robot's WiFi router (same one the laptop is on). **No internet needed.**
2. Run:
```bash
cd ~/holoassist
python3 dashboard/main.py --bridge ws://LAPTOP_IP:9090 --fullscreen
```

Replace `LAPTOP_IP` with the laptop's `192.168.0.x` address from step 1.

Example:
```bash
python3 dashboard/main.py --bridge ws://192.168.0.102:9090 --fullscreen
```

**That's it.** The dashboard should appear fullscreen on the Deck with live robot data.

### Controls on the Deck

- **Left/Right arrow keys** (D-pad): cycle tabs
- **F11**: toggle fullscreen
- **Touchscreen**: tap e-stop, resume, mode buttons
- **E-stop**: tap the red button. Hold yellow resume button for 3 seconds to resume.

---

## One-Time Setup (needs internet — do at home)

This is already done. Only redo if SteamOS updates wipe packages.

### Steam Deck dependencies

In Desktop Mode, open Konsole:

```bash
# Unlock filesystem
sudo steamos-readonly disable

# Initialize pacman keyring (required on fresh SteamOS)
sudo pacman-key --init
sudo pacman-key --populate archlinux
sudo pacman-key --populate holo

# Install Python + PyQt5 + websockets
sudo pacman -Sy python-pip python-pyqt5 python-websockets

# Lock filesystem
sudo steamos-readonly enable
```

If you get PGP signature errors, bypass with:
```bash
sudo pacman -Sy --noconfirm python-pip python-pyqt5 python-websockets --config <(cat /etc/pacman.conf | sed 's/SigLevel.*/SigLevel = Never/')
```

Verify:
```bash
python3 -c "from PyQt5.QtWidgets import QApplication; print('PyQt5 OK')"
python3 -c "import websockets; print('websockets OK')"
```

### Copy dashboard files to Deck

Both devices on the same home WiFi. From the **laptop**:

```bash
# Find the Deck's IP — run this on the Deck:
#   ip addr show | grep "inet "
# Look for the 192.168.x.x address (not 127.0.0.1)

# Enable SSH on the Deck (run on Deck):
#   passwd                              # set a password
#   sudo systemctl enable --now sshd

# From laptop, copy files:
ssh deck@DECK_IP "mkdir -p ~/holoassist"
scp -r /home/nic/git/RS2-HoloAssist/nic/dashboard/ deck@DECK_IP:~/holoassist/dashboard/
scp /home/nic/git/RS2-HoloAssist/nic/deck_dashboard.sh deck@DECK_IP:~/holoassist/
```

Replace `DECK_IP` with the Deck's actual IP and `deck` with the Deck's username (`whoami` on the Deck).

### Updating files later

If you change the dashboard code, re-scp from the laptop:
```bash
scp -r /home/nic/git/RS2-HoloAssist/nic/dashboard/ deck@DECK_IP:~/holoassist/dashboard/
```

---

## Architecture

```
Steam Deck (Arch Linux)              Laptop (Ubuntu 22.04)              Robot
────────────────────────              ────────────────────               ─────
dashboard/main.py                    dashboard/bridge_server.py
  PyQt5 GUI (1280x800)                WebSocket server (:9090)
  net_interface.py      ──WebSocket──▶  ros_interface.py    ──ROS 2 DDS──▶ UR3e + RG2
  (no ROS needed)          (WiFi)        (rclpy node)                      Gripper
```

The laptop runs `bridge_server.py` which wraps the ROS 2 node and pushes data over WebSocket at 30 Hz. The Deck runs the same `main.py` GUI with `--bridge`, using `net_interface.py` instead of rclpy. No ROS needed on the Deck.

### What goes over the WebSocket

**Laptop → Deck (30 Hz push):**
- JSON: joint states, session info, events, gripper, collision, graphs, mode, pick status
- Binary: headset JPEG stream from Quest 3

**Deck → Laptop (on demand):**
- `{"cmd": "estop"}` / `{"cmd": "resume"}`
- `{"cmd": "switch_teleop"}` / `{"cmd": "switch_moveit"}`
- `{"cmd": "pick_cube", "cube_id": N, "bin_id": N}`

### Network

```
Robot's Router (192.168.0.x, no internet):
  ├─ UR3e robot:      192.168.0.194
  ├─ Laptop Ethernet: 192.168.0.100  ←→ UR3e (direct cable)
  ├─ Laptop WiFi:     192.168.0.1xx  ←→ bridge_server.py (:9090)
  ├─ Quest 3:         192.168.0.1xx  ←→ Unity (ros_tcp_endpoint)
  └─ Steam Deck WiFi: 192.168.0.1xx  ←→ dashboard (WebSocket client)
```

All devices on the same router. No internet required.

## Screens

| Tab | Description |
|---|---|
| **STATUS** | Joint positions (degrees), velocities, gripper bar, event log |
| **HEADSET** | Live JPEG stream from Quest 3 XR camera |
| **CAMERA** | Depth camera feed (placeholder for RealSense) |
| **STATS** | Session timer, mode breakdown %, joint velocity graph (30s), topic health (60s) |
| **LATENCY** | Live latency numbers, message age graph, command interval graph |
| **SESSION** | Control mode, durations, e-stop count, connection status, topic rates |
| **CUBE** | MoveIt pick/place buttons (only visible in MOVEIT mode) |

## Status Bar (always visible, top)

- **ROS**: connected / offline
- **CTRL**: velocity controller active / inactive
- **JNT**: joint state Hz
- **COL**: collision guard clear / slow / blocked
- **GRIP**: gripper open / closed / %
- **STATE**: running / e-stopped / resuming / disconnected

## E-Stop

Always visible on the right side.

1. Tap red **EMERGENCY STOP** button
2. Burst-publishes zeros to stop the robot, then deactivates controllers
3. Hold yellow **HOLD TO RESUME** for 3 seconds to reactivate

## Mode Switching

Two buttons at the bottom:

- **TELEOP**: activates velocity + gripper controllers (for XR teleoperation)
- **MOVEIT**: activates trajectory controllers (for autonomous sorting), shows CUBE tab

## Files

```
dashboard/
  main.py              — PyQt5 GUI (all screens, e-stop, status bar)
  ros_interface.py     — ROS 2 node (rclpy), laptop only
  net_interface.py     — WebSocket client, replaces ros_interface on the Deck
  bridge_server.py     — WebSocket server, wraps ros_interface for the Deck
bridge.sh              — Laptop: sources ROS + runs bridge_server.py
deck_dashboard.sh      — Deck: runs main.py --bridge
dashboard.sh           — Laptop-only mode (no bridge, original)
```

## Troubleshooting (read before going to uni)

**Dashboard won't connect (shows DISCONNECTED):**
- Is `./bridge.sh` running on the laptop? Check for `[bridge] listening on ws://0.0.0.0:9090`
- Is the IP right? Run `hostname -I` on the laptop, use the `192.168.0.x` address
- Can you ping the laptop from the Deck? `ping LAPTOP_IP`
- Is the Deck on the robot WiFi? Check with `ip addr show | grep "inet "`

**Dashboard connects but shows "ROS: OFFLINE":**
- Bridge is working fine — ROS just isn't running yet. Start `./launch.sh` first.

**Headset stream not showing on HEADSET tab:**
- Unity must be in Play mode with `HeadsetStreamPublisher` active
- `ros_tcp_endpoint` must be running on the laptop

**E-stop doesn't work from Deck:**
- Check laptop terminal for `[bridge] client connected` — if missing, WebSocket isn't connected
- The bridge needs ROS running to actually stop the robot

**Need to re-install packages after SteamOS update:**
- SteamOS updates can wipe pacman packages. Redo the install:
```bash
sudo steamos-readonly disable
sudo pacman -Sy python-pip python-pyqt5 python-websockets --config <(cat /etc/pacman.conf | sed 's/SigLevel.*/SigLevel = Never/')
sudo steamos-readonly enable
```

**Laptop-only mode (no Deck):**
```bash
./dashboard.sh --fullscreen
```
