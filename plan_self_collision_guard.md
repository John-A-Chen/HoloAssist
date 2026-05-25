# Self-Collision Guard — ROS 2 Velocity Filter Node

**Date:** 2026-05-26  
**Status:** Planned  
**Goal:** Slow down / stop the robot when it's at risk of hitting itself, without relying on Unity

## Problem

The Unity-based `MeshCollisionGuard.cs` breaks arm control when enabled — only the gripper works. Need a solution outside Unity that's independent of the XR app.

## Approach

A ROS 2 Python node that sits between Unity and the velocity controller as a transparent filter. It intercepts velocity commands, checks self-collision risk using forward kinematics, and scales velocities down when links get too close to each other.

```
Unity (RobotController.cs)              collision_guard node              Robot
────────────────────────                ────────────────────              ─────
publishes to                subscribes to /cmd_vel_raw
/cmd_vel_raw        ──→     reads /joint_states, does FK
                            checks link-pair distances
                            scales velocity if too close
                            publishes to /forward_velocity_controller/commands  ──→  UR3e
```

## Implementation Steps

### 1. Create `ros2_ws/src/collision_guard/` package

New ROS 2 Python package with a single node.

### 2. `collision_guard_node.py`

- Subscribe to `/joint_states` (500 Hz from UR driver)
- Subscribe to `/cmd_vel_raw` (50 Hz from Unity via ros_tcp_endpoint)
- Compute FK using UR3e DH parameters (port from `UR3eKinematics.cs`)
- Get 3D positions of all link origins (base, shoulder, elbow, wrist_1, wrist_2, wrist_3, tool0)
- Check distances between non-adjacent link pairs that can actually collide:
  - Wrist/gripper area (wrist_1, wrist_2, wrist_3, tool0) vs base/shoulder area (base, shoulder, upper_arm)
  - Skip adjacent links (they're connected, can't collide)
- Compute velocity scale factor:
  - Distance > `soft_threshold` (e.g. 0.15m): scale = 1.0 (no slowdown)
  - Distance between `hard_threshold` and `soft_threshold`: linear interpolation (gradual slowdown)
  - Distance < `hard_threshold` (e.g. 0.05m): scale = 0.0 (full stop)
- Multiply all 6 joint velocities by the scale factor
- Publish scaled velocities to `/forward_velocity_controller/commands`
- Publish current scale factor + closest pair distance to a diagnostics topic for the dashboard

### 3. One-line change in Unity `RobotController.cs`

Change the velocity publish topic from `/forward_velocity_controller/commands` to `/cmd_vel_raw`.

### 4. Add to launch sequence

Add the collision guard node to `launch.py` / `launch.sh` so it starts automatically. Should start after the UR driver and before Unity connects.

### 5. Tuning parameters (ROS params)

- `soft_threshold`: distance (m) where slowdown begins (start with 0.15)
- `hard_threshold`: distance (m) where full stop (start with 0.05)
- `enabled`: bool to bypass the filter entirely (passthrough mode)
- These should be adjustable at runtime via `ros2 param set`

## UR3e DH Parameters (from UR3eKinematics.cs)

```
a  = [0, -0.24365, -0.21325, 0, 0, 0]
d  = [0.15185, 0, 0, 0.11235, 0.08535, 0.0819]
α  = [π/2, 0, 0, π/2, -π/2, 0]
```

## Link Pairs to Check

UR3e has 7 link positions (base through tool0). Non-adjacent pairs that can realistically collide:

| Proximal (near base) | Distal (near gripper) |
|---|---|
| base_link | wrist_1, wrist_2, wrist_3, tool0 |
| shoulder_link | wrist_1, wrist_2, wrist_3, tool0 |
| upper_arm_link | wrist_2, wrist_3, tool0 |

Skip pairs that are physically impossible to collide (e.g., base vs elbow — too far apart on a UR3e).

## Rollback

If it doesn't work or tuning is bad:
1. Kill the collision guard node
2. Change Unity topic back to `/forward_velocity_controller/commands`
3. Everything works exactly as before

## Dependencies

- Only standard ROS 2 Python (rclpy, std_msgs, sensor_msgs)
- No MoveIt, no extra packages
- numpy for FK matrix math (already available)

## Dashboard Integration (optional, later)

- Publish scale factor + closest distance to `/collision_guard/status`
- Dashboard could show a collision proximity indicator
- Bridge server would need to subscribe to the new topic
