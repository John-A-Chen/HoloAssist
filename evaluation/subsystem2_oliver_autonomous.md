# Subsystem 2 — Autonomous Sorting

**Lead:** Oliver Lau (24770051)

## Scope

Autonomous pick-and-place sorting using MoveIt 2 trajectory planning. The robot sorts detected objects into the correct bins without human input.

---

## Pass (P) — MoveIt 2 in simulation

- MoveIt 2 planning group configured for UR3e (arm)
- Trajectory planning and execution demonstrated in RViz simulation

---

## Credit (C) — Collision detection + gripper driver

- Gripper opens/closes as part of a scripted sequence (approach → grip → lift)
- Collision boundaries (at minimum, the workspace table) prevent unsafe trajectories
- OnRobot gripper ROS driver functional — open/close commands work

---

## Distinction (D) — Full pick-and-place from perception

- Subscribes to object poses from the perception pipeline
- Full pick-and-place cycle: approach → grip → lift → move to bin → release
- Object-to-bin mapping is defined (e.g., config file or lookup table)
- Sequence is repeatable across multiple objects and trials
- Mode switching between teleop and autonomous is functional

---

## High Distinction (HD) — Multi-object sessions with metrics

- Multiple objects sorted in a single autonomous session without manual resets
- Bin locations are parameterised — planner targets the correct bin based on object type
- Collision avoidance includes bins, workspace boundaries, and known obstacles
- Sorting metrics logged: objects sorted, success/failure per object, cycle time per sort
- Robust to minor pose estimation errors (approach strategy handles small offsets)
