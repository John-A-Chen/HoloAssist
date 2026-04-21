# Subsystem 4 — XR Scene and Visualisation

**Lead:** Sebastian Baudille (24804940)

## Scope

Unity XR environment design, virtual object overlay system, and operator-facing visualisation. Responsible for rendering the mixed reality scene on the Quest 3, including the real-to-virtual object mapping that creates the gamified sorting experience.

---

## Pass (P) — XR Unity UI

- XR user can observe pose markers and basic robot transforms information within the Unity XR UI

---

## Credit (C) — XR UI and Environment

- System visualisation supports repeated demonstration trials by clearly presenting robot state, task progress, and safety context to the operator
- Subscribes to object pose data from ROS (via ros_tcp_endpoint)

---

## Distinction (D) — Real-to-virtual mapping + bin visualisation

- Mapping system translates object type from perception (e.g., "tomato") to a virtual representation (e.g., bomb 3D model)
- Virtual objects are visually distinct and clearly identifiable by the operator
- Bin locations rendered in XR with labels, colours, or icons indicating target object type
- Operator can see at a glance which object goes where

---

## High Distinction (HD) — Polished UX with progress tracking and feedback

- Sorting progress UI: objects remaining, objects sorted, current score or tally
- Visual feedback on sort outcome — success (green flash, checkmark) and failure (red flash, warning) on bin placement
- Task guidance overlays: highlights on the next object to sort, directional indicators toward target bin
- Scene is visually polished — consistent art style, readable UI at arm's length, no placeholder assets in final build
- Smooth performance on Quest 3 with all overlays active
