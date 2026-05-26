# Robot Control

**Hardware:** Universal Robots UR3e + OnRobot RG2 gripper  
**Package:** `ur_onrobot_control`, `moveit_robot_control`

---

## Hardware Overview

| Component | Details |
|---|---|
| Arm | UR3e (6-DOF, 3 kg payload, 500 mm reach) |
| Gripper | OnRobot RG2 (0–110 mm finger width, 40 N force) |
| Driver | `ur_robot_driver` (ROS 2 control) |
| Gripper comms | Modbus RS-485 via `/tmp/ttyUR` (socat → port 54321) |
| Default IP | `192.168.0.194` |
| URSim IP | `192.168.56.101` |

---

## Launch Modes

### Real robot

```bash
./launch.sh --robot-ip 192.168.0.194 --perception --moveit --dashboard
```

### URSim (simulated)

```bash
# Terminal 2
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  -p 50001-50002:50001-50002 \
  --name ursim universalrobots/ursim_e-series

# Terminal 1
./launch.sh --robot-ip 192.168.56.101 --fake-gripper --perception --moveit --dashboard
```

After `✓  UR + OnRobot Driver` appears, click **Play → External Control** in Polyscope (browser: http://localhost:6080).

!!! note "Why --fake-gripper with URSim?"
    URSim does not emulate port 54321 (the OnRobot RS-485 serial relay). Without
    `--fake-gripper`, the `tool_communication_node` (socat) hangs, blocking
    `controller_manager` indefinitely.

---

## External Control (URSim / Real)

The UR driver uses the URScript External Control program to take over joint control from Polyscope. The flow is:

1. `ur_ros2_control_node` connects to robot on ports 30001 (primary) and 30004 (RTDE)
2. Driver uploads the External Control URScript
3. Polyscope connects back to host on port **50002** to download the script
4. Robot enters External Control mode — ROS controls joints

**If "Receive program failed: 192.168.56.1:50002"** — the driver hasn't opened port 50002 yet (or crashed). Restart the launcher, wait for `✓  UR + OnRobot Driver`, then click Play.

---

## MoveIt Pick-and-Place

Enabled with `--moveit`. Launches `full_holoassist_hardware.launch.py` which starts:

- MoveIt move_group with UR3e + RG2 SRDF
- Trajectory execution against `scaled_joint_trajectory_controller`
- Planning scene with floor collision object

### Cube sorting flow

1. Perception node detects cube poses → publishes to `/holoassist/perception/april_cube_N_pose`
2. `pick_place_sequencer` reads cube poses and target bin assignments
3. Plans Cartesian path (primary) → OMPL fallback
4. Executes: move above cube → close gripper → lift → move to bin → open gripper
5. Perception node checks bin: `/holoassist/perception/april_cube_N_bin_check`

### Controller layout

| Controller | Mode | Purpose |
|---|---|---|
| `scaled_joint_trajectory_controller` | Active (MoveIt mode) | MoveIt trajectory execution |
| `joint_trajectory_controller` | Inactive | Alternative trajectory controller |
| `forward_velocity_controller` | Active (teleop mode) | Teleop velocity control |
| `finger_width_trajectory_controller` | Active | Gripper position trajectory |
| `finger_width_controller` | Inactive | Direct gripper width control |

The launcher switches controllers depending on `--moveit`:

- With `--moveit`: keeps `scaled_joint_trajectory_controller` active for MoveIt
- Without `--moveit`: switches to `forward_velocity_controller` + `finger_width_controller` for teleop

---

## URDF / XACRO

**File:** `ur_onrobot_description/urdf/ur_onrobot.urdf.xacro`

Key xacro args:

| Arg | Default | Description |
|---|---|---|
| `ur_type` | `ur3e` | Robot model |
| `onrobot_type` | `rg2` | Gripper model |
| `use_fake_hardware` | `false` | Fake arm (no real robot needed) |
| `use_fake_gripper_hardware` | `false` | Fake gripper (skips Modbus) |
| `base_yaw_rad` | `0.0` | Mounting yaw offset (trolley = 0.0) |

!!! note "base_yaw_rad"
    The trolley mounts the arm facing 180° from UR default. This is handled by
    the trolley scene publisher (180° yaw in its static transform), not the URDF.
    The URDF `base_yaw_rad` is kept at `0.0`.

---

## Safety

- MoveIt collision check on every trajectory
- Floor collision object in planning scene prevents table crashes
- UR flange-to-forearm clamp check (`avoid_flange_forearm_clamp=true`) rejects self-collision trajectories
- Robot status checks before execution (safety mode, program running)
- E-stop available via the dashboard window (red button)
