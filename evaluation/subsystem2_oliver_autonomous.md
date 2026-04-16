# Subsystem 2 — Autonomous Sorting

**Lead:** Oliver Lau (24770051)

## Scope

Autonomous pick-and-place sorting using MoveIt 2 trajectory planning. The robot sorts detected objects into the correct bins without human input.

---

## Pass (P)

MoveIt 2 is configured for the UR3e and can plan and execute collision-free trajectories in simulation (fake hardware). The OnRobot gripper driver is installed and responds to open/close commands via ROS.

**Criteria:**
- MoveIt 2 planning group configured for UR3e (arm + end-effector)
- Collision geometry loaded (robot self-collision + workspace table)
- Trajectory planning and execution demonstrated in RViz with fake hardware
- OnRobot gripper ROS driver functional — open/close commands work

---

## Credit (C)

The robot can execute a planned trajectory to a given target pose on the real hardware. Gripper open/close is integrated into the motion sequence.

**Criteria:**
- MoveIt 2 plans executed on the real UR3e (not just simulation)
- A target pose (e.g., hardcoded or from a service call) results in the arm moving to that pose
- Gripper opens/closes as part of a scripted sequence (approach → grip → lift)
- Collision boundaries (at minimum, the workspace table) prevent unsafe trajectories

---

## Distinction (D)

A complete autonomous pick-and-place sequence is functional: the robot moves to a detected object's pose, grips it, moves to the assigned bin, and releases it. Object-to-bin assignment uses a predefined mapping.

**Criteria:**
- Subscribes to object poses from the perception pipeline (John's subsystem)
- Full pick-and-place cycle: approach → grip → lift → move to bin → release
- Object-to-bin mapping is defined (e.g., config file or lookup table)
- Sequence is repeatable across multiple objects and trials
- Mode switching between teleop and autonomous is functional (operator can toggle)

---

## High Distinction (HD)

The autonomous sorting pipeline handles multiple objects in sequence. Trajectory planning accounts for bin locations and avoids collisions with workspace obstacles. Sorting performance is logged with success/failure metrics.

**Criteria:**
- Multiple objects sorted in a single autonomous session without manual resets
- Bin locations are parameterised (not hardcoded trajectories) — the planner targets the correct bin based on object type
- Collision avoidance includes bins, workspace boundaries, and any known obstacles
- Sorting metrics logged: objects sorted, success/failure per object, cycle time per sort
- Robust to minor pose estimation errors (approach strategy handles small offsets)
