# HoloAssist — Project Plan

## Concept

HoloAssist is an XR-assisted robotic sorting system. A UR3e collaborative arm, controlled via a Meta Quest 3 headset, sorts physical objects into bins within a defined workspace. A depth camera (Intel RealSense) detects objects as they enter the workspace, identifies them (by colour or computer vision), and tracks their position until the robot manipulates them.

The system has two operating modes:

- **Teleoperation mode** — a human operator wearing the Quest 3 headset controls the robot arm and gripper to pick up objects and place them in the correct bin. The operator sees the workspace in mixed reality with XR overlays showing object identity, bin assignments, and robot state.
- **Autonomous mode** — the robot autonomously sorts objects using planned trajectories (MoveIt 2), without human input. Object-bin assignments come from a predefined mapping or (later) a trained model.

### The XR Twist

Real-world objects are mapped to different virtual objects in the XR scene. For example, a physical tomato on the workspace appears as a bomb in the Quest 3 headset that the operator must "defuse" by sorting it into the correct bin. This creates an engaging, gamified operator experience while the underlying robotic task remains a practical sorting pipeline.

### Stretch Goal: Human vs CPU

Teleop session data (object poses, grasp strategies, sort times) can later be used to train a model that learns the relationship between real objects and their target bins. This enables a "race mode" where a teleoperator competes against an overlay of the autonomous agent performing the same task — primarily for demonstration purposes.

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Robot arm | Universal Robots UR3e | 6-DOF collaborative manipulator |
| Gripper | OnRobot (RG2 or RG6) | Parallel gripper for pick-and-place |
| XR headset | Meta Quest 3 | Operator interface (mixed reality passthrough) |
| Depth camera | Intel RealSense | Object detection, identification, and tracking |
| Laptop | Dell XPS 15 (Ubuntu 22.04) | ROS 2, Unity editor, network bridge |
| Secondary display | Steam Deck OLED | Supervisor dashboard (e-stop, telemetry) |
| Workspace markers | QR codes (TBD) | Camera-to-robot coordinate frame calibration |

---

## System Architecture

```
                         +-----------------------+
                         |   Intel RealSense     |
                         |   Depth Camera        |
                         +-----------+-----------+
                                     |
                            object detection,
                          colour/CV identification,
                              pose tracking
                                     |
                                     v
+-------------------+      +---------+-----------+      +-------------------+
|   Meta Quest 3    |      |      ROS 2          |      |   OnRobot Gripper |
|   (XR Interface)  +<---->+   (Humble)          +----->+   (RG2/RG6)       |
|                   |      |                     |      +-------------------+
| - MR passthrough  |  TCP | - UR driver         |
| - XR object       |bridge| - Perception node   |      +-------------------+
|   overlays        |      | - MoveIt 2 (auto)   +----->+   UR3e Arm        |
| - Teleop controls |      | - ros_tcp_endpoint  |      |   (6-DOF)         |
| - Gripper trigger |      | - Gripper driver    |      +-------------------+
+-------------------+      +---------+-----------+
                                     |
                                     v
                         +-----------+-----------+
                         |   Steam Deck          |
                         |   Dashboard           |
                         |   (e-stop, telemetry) |
                         +-----------------------+
```

### Data Flow

1. **Object enters workspace** — the depth camera detects the object, identifies it (colour/CV), and publishes its pose to ROS.
2. **Pose broadcast** — the object's pose is transformed into the robot's coordinate frame (using QR code calibration) and sent to both Unity (via ros_tcp_endpoint) and RViz.
3. **XR rendering** — Unity receives the object pose and renders the corresponding virtual object (e.g., tomato -> bomb) at the correct location in the operator's mixed reality view.
4. **Sorting** — depending on the active mode:
   - *Teleop*: the operator uses XR controllers to drive the robot arm to the object, grip it, and place it in the correct bin.
   - *Autonomous*: MoveIt 2 plans a collision-free trajectory to the object, the robot picks it up and places it in the assigned bin.
5. **Bin verification** — the depth camera confirms whether the object has been placed in the correct bin.

---

## Subsystems

### Subsystem 1 — Perception (John)

Object detection, identification, and spatial tracking using the Intel RealSense depth camera.

**Responsibilities:**
- Configure and calibrate the RealSense (intrinsics, extrinsics, camera-to-robot transform via QR codes)
- Detect objects entering the workspace (depth segmentation, background subtraction)
- Identify objects by colour, shape, or computer vision (classification)
- Publish object poses to ROS (type, position, orientation) in the robot's coordinate frame
- Track objects through the workspace until they are manipulated
- Optionally verify correct bin placement after sorting

**Key ROS interfaces:**
- Publishes: object poses, object types/classes, point clouds
- Coordinate frame: camera frame -> robot base frame (via static or QR-based transform)

---

### Subsystem 2 — Autonomous Sorting (Oliver)

Autonomous pick-and-place using MoveIt 2 trajectory planning — the robot sorts objects without human input.

**Responsibilities:**
- Configure MoveIt 2 for the UR3e (collision geometry, planning groups, end-effector)
- Plan collision-free trajectories from current pose to detected object pose
- Execute pick-and-place sequences: approach -> grip -> lift -> move to bin -> release
- Integrate gripper open/close commands into the motion pipeline
- Handle object-to-bin assignment logic (predefined mapping, or later a trained model)
- Mode switching: operator can toggle between teleop and autonomous mode

**Key ROS interfaces:**
- Subscribes: object poses (from perception), gripper state
- Publishes: planned trajectories, gripper commands
- Uses: MoveIt 2 planning scene, `forward_velocity_controller` or `scaled_joint_trajectory_controller`

---

### Subsystem 3 — Teleoperation and Interaction (Nic)

XR-based teleoperation — the human operator controls the robot arm and gripper through the Quest 3 headset to sort objects.

**Responsibilities:**
- Existing teleoperation modes (all carry over from current implementation):
  - RMRC (Resolved Motion Rate Control) — Jacobian-based Cartesian velocity control with translate/rotate sub-modes
  - Direct Joint — individual joint jogging
  - Hand Guide — controller position tracking with Jacobian IK
- **New: gripper control** — trigger or button to open/close the OnRobot gripper via ROS
- E-stop dashboard on Steam Deck (existing — fully functional)
- Session logging and telemetry (existing — mode tracking, durations, events)
- Headset scene streaming to dashboard (existing)
- Safety features: joint speed limits, velocity scaling, e-stop

**Key ROS interfaces:**
- Publishes: velocity commands (`/forward_velocity_controller/commands`), gripper commands, session events
- Subscribes: joint states, object poses (for awareness), gripper state

**What's already built:**
- RobotController.cs (3 modes, tested on Quest 3 with real robot)
- UR3eKinematics.cs (FK, Jacobian, DLS inverse)
- JointStateSubscriber.cs (digital twin)
- RobotBasePlacer.cs (MR robot placement)
- RobotHUD.cs (floating mode/status display)
- SpatialMarkers.cs (end-effector visualisation)
- HeadsetStreamPublisher.cs (XR camera to dashboard)
- SessionLogger.cs (metrics and logging)
- Dashboard (main.py + ros_interface.py — e-stop, 6 tabs, session logs)
- launch.sh / dashboard.sh (one-command launchers)

**What's new:**
- Gripper control integration (open/close from XR controller)

---

### Subsystem 4 — XR Scene and Visualisation (Sebastian)

Unity scene rendering, XR environment, and the virtual object overlay system.

**Responsibilities:**
- Design and build the XR scene environment in Unity
- Implement the object mapping system: real object type -> virtual XR object (e.g., tomato -> bomb)
- Render virtual objects at the correct pose in the MR passthrough view (aligned with real workspace)
- Visualise bins/targets in XR (labels, highlights, placement guides)
- Coordinate frame alignment between Unity scene and real workspace (QR code anchoring)
- UI/UX for the operator: bin assignments, sorting progress, task status overlays
- Visual feedback on sort correctness (success/failure indicators)

**Key ROS interfaces:**
- Subscribes: object poses and types (from perception), sort confirmations
- Scene rendering is local to Unity (no ROS publish needed for visuals)

---

## Workspace Layout (Physical)

```
+--------------------------------------------------+
|                                                  |
|   [Camera]  (Intel RealSense, mounted TBD)       |
|                                                  |
|   +------------------------------------------+   |
|   |            Workspace Area                |   |
|   |                                          |   |
|   |    [QR]                          [QR]    |   |
|   |                                          |   |
|   |         (objects placed here)             |   |
|   |                                          |   |
|   |    [QR]                          [QR]    |   |
|   +------------------------------------------+   |
|                                                  |
|          [UR3e + OnRobot Gripper]                 |
|                  |                                |
|   +------+  +------+  +------+                   |
|   |Bin A |  |Bin B |  |Bin C |                   |
|   +------+  +------+  +------+                   |
|                                                  |
+--------------------------------------------------+
```

Bins may be horizontally arranged on a table or vertically stacked on a shelf — TBD based on workspace constraints and robot reach.

---

## Integration Points

| From | To | What | How |
|---|---|---|---|
| Perception | All | Object type + pose | ROS topic (custom msg or PoseStamped + metadata) |
| Perception | Autonomous | "Object ready to sort" trigger | ROS topic or service |
| Perception | Visualisation | Object class for XR mapping | ROS topic (forwarded via ros_tcp_endpoint) |
| Teleop | Robot | Joint velocity commands | `/forward_velocity_controller/commands` |
| Teleop | Gripper | Open/close commands | ROS topic or service (TBD with OnRobot driver) |
| Autonomous | Robot | Planned trajectories | MoveIt 2 execution |
| Autonomous | Gripper | Open/close commands | Same gripper interface |
| Visualisation | Operator | Virtual objects, bin overlays | Unity scene (local rendering) |
| Perception | Teleop/Auto | Bin verification | ROS topic (sort success/failure) |

---

## Milestones

### Phase 1 — Foundation (largely complete)
- [x] UR3e digital twin in Unity (joint mirroring)
- [x] XR teleoperation (RMRC, Direct Joint, Hand Guide)
- [x] E-stop dashboard on Steam Deck
- [x] Session logging and telemetry
- [x] Headset streaming to dashboard
- [ ] OnRobot gripper driver in ROS
- [ ] Gripper control from XR controller

### Phase 2 — Perception Pipeline
- [ ] RealSense configured and publishing in ROS
- [ ] Camera-to-robot calibration (QR codes)
- [ ] Object detection and classification (colour/CV)
- [ ] Object pose published to ROS in robot frame

### Phase 3 — Sorting Task
- [ ] Object poses received in Unity and rendered as XR overlays
- [ ] Teleop sorting: operator picks and places objects into bins
- [ ] Autonomous sorting: MoveIt plans and executes pick-and-place
- [ ] Bin verification via depth camera

### Phase 4 — Integration and Polish
- [ ] Mode switching between teleop and autonomous
- [ ] XR object mapping (real -> virtual, e.g., tomato -> bomb)
- [ ] Gamification overlay (teleop vs CPU race — stretch goal)
- [ ] WiFi resilience (auto e-stop on dropout — stretch goal)
- [ ] Final integrated demonstration

---

## Notes

- The existing teleoperation stack (RMRC, Hand Guide, Direct Joint, dashboard) is fully functional and carries over as-is. Nic's primary new work is gripper integration.
- MoveIt Servo is not used for teleoperation (replaced by RMRC). MoveIt 2 will be used by Oliver's autonomous sorting subsystem for trajectory planning.
- The ML training pipeline (learning object-bin relationships from teleop data) is out of scope for the current plan — it is a future extension.
- Camera mounting position (overhead, side, wrist) is TBD and will be decided based on workspace testing.
- QR code placement and calibration strategy will be finalised during Phase 2.
