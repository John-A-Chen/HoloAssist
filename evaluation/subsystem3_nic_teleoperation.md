# Subsystem 3 — Teleoperation and Interaction

**Lead:** Nicholas Sabatta (24796955)

## Scope

XR-based teleoperation of the UR3e arm and OnRobot gripper via Meta Quest 3. Includes control modes, operator dashboard, session logging, and gripper integration.

---

## Pass (P) — Stable digital twin on Quest 3

- JointStateSubscriber.cs subscribes to `/joint_states` and drives Unity joint transforms
- Digital twin verified on Quest 3 — joints match real robot direction and axes
- Purely kinematic (no physics simulation) — stable under all conditions

**Status:** DONE

---

## Credit (C) — RMRC + Direct Joint + e-stop dashboard

- RobotController.cs with RMRC (Jacobian-based Cartesian velocity, translate/rotate sub-modes) and Direct Joint mode
- Both modes publish to `/forward_velocity_controller/commands` at 50 Hz
- UR3eKinematics.cs provides forward kinematics, geometric Jacobian, and DLS pseudoinverse
- Dashboard (main.py + ros_interface.py) with e-stop: burst zero-velocity publish, controller deactivation, 5-second hold resume, cooldown

**Status:** DONE

---

## Distinction (D) — Three modes + headset streaming dashboard

- Three control modes cycled via Menu button: RMRC → Direct Joint → Hand Guide
- RMRC has two sub-modes (Translate/Rotate) toggled via X button
- HeadsetStreamPublisher.cs renders VR camera view → JPEG → `/headset/image_compressed` at 15 FPS
- Dashboard tabs: STATUS, HEADSET (live XR view), STATS (session bar + joint velocity graph), LATENCY, SESSION
- Launch scripts (launch.sh, dashboard.sh) for one-command startup

**Status:** DONE

---

## High Distinction (HD) — Hand Guide mode + session logging + gripper

- Hand Guide mode: hold right grip to engage — controller 3D position mapped to end-effector via proportional control + Jacobian IK, gain 2, speed cap 0.15 m/s
- SessionLogger.cs: tracks mode switches, per-mode durations, session time — publishes JSON to `/session/status` (2 Hz) and saves on quit
- Dashboard saves session log to `~/holoassist_sessions/` on shutdown (events, e-stop count, Unity session info)
- OnRobot gripper open/close triggered from XR controller during teleoperation, enabling full pick-and-place workflow

**Status:** Hand Guide + session logging DONE. Gripper integration TODO.
