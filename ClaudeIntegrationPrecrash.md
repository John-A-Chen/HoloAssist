# Integration Session — 2026-05-04

Recovery notes in case of laptop crash during Unity rebuild.

## What Was Fixed

### launch.py bugs (from integration merge)
1. **`NIC_DIR` undefined** — 3 occurrences (lines 185, 227, 281) replaced with `SCRIPT_DIR`
2. **Controller switch never executed** — `switch_cmd` was defined but `run_once()` was never called. Added the call.
3. **Controller switch used STRICT mode** — changed `strictness: 2` → `strictness: 1` (BEST_EFFORT) because `spawner-4` dies from a race condition and `scaled_joint_trajectory_controller` never loads. BEST_EFFORT skips missing controllers and still activates `forward_velocity_controller` + `finger_width_controller`.
4. **Perception sim launched duplicate robot stack** — `full_holoassist_moveit_sim.launch.py` ignores `start_robot:=false` (that arg doesn't exist). Changed to `holo_assist_depth_tracker_sim sim_april_cube_perception.launch.py use_rviz:=false publish_scene_state_publisher:=false` which only launches perception nodes.
5. **Port 10000 not freed on crash** — added `fuser -k 10000/tcp` before starting `ros_tcp_endpoint`.
6. **Non-critical process death killed everything** — the main loop called `cleanup()` when ANY process exited. Now only `UR + OnRobot Driver` and `ROS-TCP Endpoint` are critical; others just print a warning and get removed from the process list.

### Missing dependencies
- **`tf_transformations`** — install via: `sudo apt install -y ros-humble-tf-transformations`
- **`rosbridge_server`** — install via: `sudo apt install -y ros-humble-rosbridge-server`
- **`transforms3d`** — already installed via pip (`pip install transforms3d`)

## Current State (working)

### Launch command
```bash
./launch.sh
```
Fake hardware by default. Starts: UR driver, controller switch (teleop), TCP endpoint, beacon, rosbridge, MoveIt, perception sim, cube relay, workspace scene manager, coordinate listener, pick-place sequencer, pick-place service, auto-sort orchestrator.

### Controllers confirmed active
```
forward_velocity_controller: activate successful
finger_width_controller: activate successful
```

### Known spawner-4 race condition
`spawner-4` (active controllers: `joint_state_broadcaster`, `io_and_status_controller`, etc.) fails due to triple-load race on `joint_state_broadcaster`. This is a ROS 2 Humble bug with the UR driver's parallel spawners. Non-critical for teleop — all needed controllers are in `spawner-5` (inactive list) and get activated by the switch command.

### Network (fake hardware, desktop testing)
```
Laptop WiFi:  172.19.115.20  (university network)
Docker:       172.17.0.1
VirtualBox:   192.168.56.1
```
Not on robot's `192.168.0.x` subnet — that's fine for fake hardware.

**Unity ROS Settings:** `127.0.0.1` port `10000` (localhost, since ros_tcp_endpoint listens on `0.0.0.0`)

`ROSAutoConnect.cs` scans `192.168.0.101-109`, finds nothing on university WiFi, falls back to ROS Settings IP.

### TF warnings (non-blocking)
MoveIt warns about disconnected TF trees (`workspace_frame`, `camera_link`, etc. not connected to `world`). This is because `workspace_frame_tf` node (from Oliver's `full_holoassist_moveit_sim.launch.py`) isn't running in the current perception-only launch. Doesn't block teleop.

## If Laptop Crashes

1. Kill leftover processes:
   ```bash
   fuser -k 10000/tcp 2>/dev/null
   pkill -f ros2_control_node
   pkill -f ros_tcp_endpoint
   ```

2. Verify dependencies:
   ```bash
   python3 -c "import tf_transformations; print('OK')"
   ros2 pkg list | grep rosbridge
   ```

3. Launch:
   ```bash
   cd ~/git/RS2-HoloAssist/nic
   ./launch.sh
   ```

4. Unity ROS Settings: `127.0.0.1:10000`

5. If you only need teleop (no perception/MoveIt):
   ```bash
   ./launch.sh --no-perception --no-moveit
   ```

## Files Modified
- `launch.py` — all 6 fixes above
- No ROS packages modified, no rebuild needed
