# Subsystem 4 — XR Scene and Visualisation

**Lead:** Sebastian

## Scope

Unity XR environment design, virtual object overlay system, and operator-facing visualisation. Responsible for rendering the mixed reality scene on the Quest 3, including the real-to-virtual object mapping that creates the gamified sorting experience.

---

## Pass (P)

The Unity XR scene renders a stable mixed reality environment on the Quest 3 with passthrough enabled. The workspace area and robot are visible and correctly positioned in the operator's view.

**Criteria:**
- Mixed reality passthrough functional on Quest 3
- Robot digital twin visible and correctly placed in the scene
- Workspace area represented (floor plane, table, or boundary markers)
- Scene runs at acceptable frame rate on Quest 3 (no major drops or visual artefacts)

---

## Credit (C)

Virtual objects are rendered in the mixed reality scene at poses received from the perception pipeline. Objects appear at the correct real-world location in the operator's passthrough view.

**Criteria:**
- Subscribes to object pose data from ROS (via ros_tcp_endpoint)
- Virtual object spawned/positioned at the detected real-world location
- Object position is aligned with the physical workspace (coordinate frame calibration working)
- Objects update or reposition as new detections arrive

---

## Distinction (D)

The real-to-virtual object mapping system is implemented — physical objects are displayed as different virtual objects in XR (e.g., a tomato appears as a bomb). Sorting bins are visualised with labels or visual indicators showing which object type belongs where.

**Criteria:**
- Mapping system translates object type from perception (e.g., "tomato") to a virtual representation (e.g., bomb 3D model)
- Virtual objects are visually distinct and clearly identifiable by the operator
- Bin locations rendered in XR with labels, colours, or icons indicating target object type
- Operator can see at a glance which object goes where

---

## High Distinction (HD)

The full XR operator experience is polished: sorting progress is tracked and displayed, visual feedback confirms correct/incorrect bin placement, and the scene provides clear task guidance. The environment is suitable for demonstration.

**Criteria:**
- Sorting progress UI: objects remaining, objects sorted, current score or tally
- Visual feedback on sort outcome — success (e.g., green flash, checkmark) and failure (e.g., red flash, warning) when an object is placed in a bin
- Task guidance overlays: highlights on the next object to sort, directional indicators toward target bin
- Scene is visually polished — consistent art style, readable UI at arm's length, no placeholder assets in the final build
- Smooth performance on Quest 3 with all overlays active
