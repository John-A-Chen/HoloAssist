# Subsystem 3 — Teleoperation and Interaction

**Lead:** Nicholas Sabatta (24796955)

## Scope

XR-based teleoperation of the UR3e arm and OnRobot gripper via Meta Quest 3. Includes control modes, operator dashboard, session logging, and gripper integration.

---

## Pass (P)

XR headset displays a stable digital twin of the UR3e. Robot joint states update reliably in the Unity scene during motion, with the digital twin accurately mirroring the real robot's pose.

**Evidence:**
- JointStateSubscriber.cs subscribes to `/joint_states` and drives Unity joint transforms
- Digital twin verified on Quest 3 — joints match real robot direction and axes
- Purely kinematic (no physics simulation) — stable under all conditions

---

## Credit (C)

A basic teleoperation mode is operational through XR input (RMRC and Direct Joint), with task completion achievable in repeated trials. A software emergency stop dashboard is functional on a secondary device (Steam Deck), capable of halting robot motion and deactivating the velocity controller with a safe resume sequence.

**Evidence:**
- RobotController.cs with RMRC mode (Jacobian-based Cartesian velocity, translate/rotate sub-modes) and Direct Joint mode (individual joint jogging)
- Both modes publish to `/forward_velocity_controller/commands` at 50Hz
- UR3eKinematics.cs provides forward kinematics, geometric Jacobian, and DLS pseudoinverse
- Dashboard (main.py + ros_interface.py) with e-stop: burst zero-velocity publish, controller deactivation, 5-second hold resume, cooldown

---

## Distinction (D)

The operator can switch between multiple teleoperation modes. The operator dashboard streams the headset camera view via ROS and displays real-time system telemetry including topic publish rates, joint states, latency metrics, and controller status across a tabbed interface controllable from the Steam Deck.

**Evidence:**
- Three control modes cycled via Menu button: RMRC → Direct Joint → Hand Guide
- RMRC has two sub-modes (Translate/Rotate) toggled via X button
- HeadsetStreamPublisher.cs renders XR camera view → JPEG → `/headset/image_compressed` at 15 FPS
- Dashboard tabs: STATUS (joints + event log), HEADSET (live XR view), STATS (session bar + joint velocity graph + topic health), LATENCY (message age + command interval graphs), SESSION (full text overview of system state)
- Launch scripts (launch.sh, dashboard.sh) for one-command startup

---

## High Distinction (HD)

Teleoperation includes a hand-guided tracking mode using Jacobian IK, allowing the operator to intuitively guide the end-effector by moving the controller in 3D space. Teleoperation sessions are logged with quantitative performance metrics. The OnRobot gripper is controllable from the XR interface, enabling the operator to perform complete pick-and-place sorting tasks.

**Evidence:**
- Hand Guide mode: hold right grip to engage — controller 3D position tracked and mapped to end-effector via proportional control + Jacobian IK. Mellow gain (2) + speed cap (0.15 m/s) + joint bias weights favouring wrist over base/shoulder. Axis mapping verified on Quest 3.
- SessionLogger.cs: tracks mode switches, per-mode durations, session time. Publishes JSON to `/session/status` (2Hz) + `/session/events`. Saves session log JSON to persistent storage on quit.
- Dashboard saves session log to `~/holoassist_sessions/` on shutdown (events, e-stop count, Unity session info).
- OnRobot gripper open/close commands triggered from XR controller during teleoperation, enabling full pick-and-place workflow for the sorting task.
