# Subsystem 3 — Interaction and Execution: Proposed Evaluation Renegotiation

**Lead:** Nicholas Sabatta (24796955)

## Rationale

The current Subsystem 3 evaluation criteria at the HD and Perfect levels depend heavily on the completion and integration of other subsystems (Perception, Planning, Visualisation). This creates a situation where my grade is gated by the progress of other team members' work rather than the quality and depth of my own contributions.

The proposed revision reframes the evaluation around deliverables I directly own and can independently demonstrate, while still contributing to the integrated system. The scope of work is equivalent or greater — it shifts from *waiting for others' features to appear in my subsystem* to *building robust, independently testable operator tooling*.

## Current vs Proposed Evaluation

### Pass (P) — No change

**Current:** XR headset displays a stable robot model and end-effector pose, and robot state updates reliably in XR during motion.

**Proposed:** *Same.* Already demonstrated.

---

### Credit (C) — Minor addition

**Current:** A basic teleoperation mode is operational through XR input, with task completion achievable in repeated trials.

**Proposed:** A basic teleoperation mode is operational through XR input (RMRC and Direct Joint), with task completion achievable in repeated trials. A software emergency stop dashboard is functional, capable of halting robot motion and deactivating the velocity controller with a safe resume sequence.

**Justification:** The e-stop dashboard is a safety-critical operator tool that I have designed and built. It publishes zero-velocity commands and manages controller state transitions — this is core interaction and execution functionality.

---

### Distinction (D) — Revised

**Current:** The operator can switch between supported teleoperation modes, and passthrough or spatial marker visualisation is operational.

**Proposed:** The operator can switch between supported teleoperation modes. The operator dashboard streams the headset camera view via ROS and displays real-time system telemetry including topic publish rates, joint states, and controller status across a tabbed interface controllable from a secondary device (Steam Deck).

**Justification:** The current D criterion ("passthrough or spatial marker visualisation") overlaps significantly with Subsystem 4 (XR and Scene Visualisation, led by Sebastian). Replacing it with the operator dashboard — which I have independently designed, built, and integrated with the ROS network — better reflects work within my subsystem boundary. The dashboard provides equivalent operational value: it gives the operator (or a supervisor) real-time awareness of system state without requiring XR.

---

### High Distinction (HD) — Revised

**Current:** Mixed reality UI supports visualisation and debugging of the depth camera feed, such as highlighting or painting regions of interest in the workspace.

**Proposed:** Teleoperation includes a hand-guided tracking mode using Jacobian IK, allowing the operator to intuitively guide the end-effector by moving the controller in 3D space. Teleoperation sessions are logged with quantitative performance metrics (task completion time, number of e-stop interventions, control mode usage, command latency). The dashboard displays real-time system telemetry including ROS topic health, joint states, and latency metrics.

**Justification:** The current HD criterion requires John's full perception pipeline to be rendered inside Unity's MR environment — a deep cross-subsystem dependency that gates my grade on another team member's progress. The proposed criterion replaces this with two independently-owned deliverables: (1) a hand-guided tracking mode using Jacobian IK with joint bias weighting and speed limiting — a technically demanding interaction mode that goes well beyond basic teleoperation, and (2) quantitative session logging that adds rigour to the testing deliverable (D7) and supports the system evaluation's requirement for "quantitative comparison between freeform and assisted modes." Both are fully self-contained.

---

### Perfect — Revised

**Current:** XR interaction is stable and low jitter, with smooth trajectory preview visualisation and a polished user experience suitable for public demonstration.

**Proposed:** The full operator station (XR teleoperation + dashboard monitoring) is robust under adverse conditions: WiFi communication dropouts trigger automatic e-stop with clear operator feedback, latency spikes are detected and displayed in real-time, and the system recovers gracefully when connectivity is restored. The operator experience is polished and suitable for public demonstration.

**Justification:** The current Perfect criterion requires trajectory preview visualisation, which depends on Oliver's MoveIt planning output and Sebastian's XR rendering — two subsystems outside my control. The proposed criterion tests resilience and graceful degradation of the interaction layer, which is arguably more important for a teleoperation system operating over wireless links. It is independently testable by simulating network conditions and does not require other subsystems to be feature-complete.

---

## Summary of Changes

| Level | Current dependency | Proposed dependency |
|---|---|---|
| P | None | None |
| C | Oliver (control) | None (dashboard is standalone) |
| D | Sebastian (visualisation) | None (dashboard is standalone) |
| HD | John (perception in MR) | None (hand-guide mode + logging + dashboard telemetry) |
| Perfect | Oliver + John + Sebastian (full integration) | None (resilience testing is self-contained) |

The total scope of work increases (dashboard development, headset streaming, metrics logging, resilience testing) while cross-subsystem risk decreases. All proposed criteria still serve the project's goals of safe, observable, constraint-aware teleoperation.
