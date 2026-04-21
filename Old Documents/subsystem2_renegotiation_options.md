# Subsystem 2 — Motion Planning and Control: Proposed Renegotiation Options

**Lead:** Oliver Lau (24770051)

## Why a change is needed

The current Subsystem 2 evaluation criteria overlap heavily with work already completed in Subsystem 3 (Interaction and Execution). Specifically:

- **P (reactive Cartesian velocity control)** — RMRC is already implemented in Unity via `RobotController.cs` and `UR3eKinematics.cs` (Jacobian-based velocity resolution, DLS pseudoinverse).
- **D (multiple teleoperation modes, EE control + joint manipulation)** — three modes (RMRC, Direct Joint, Hand Guide) are already operational on Quest 3.

This means Oliver's current P and D criteria describe work that is already done by someone else. The criteria need to be reframed so Oliver's grade reflects his own contributions.

Below are three options. Each keeps the subsystem name and maintains equivalent scope, but differentiates clearly from Subsystem 3.

---

## Option A: Gripper and Hardware Integration

Plays to Oliver's embedded systems / hardware expertise. Focuses on the physical end-effector control and collaborative task execution.

| Level | Proposed Criterion |
|---|---|
| **P** | Gripper (OnRobot RG2/RG6) is controllable via ROS, with open/close commands triggerable from the XR interface or dashboard. |
| **C** | Gripper force and/or position feedback is published to ROS and visible to the operator (dashboard or XR HUD). |
| **D** | A simple collaborative task (pick-and-place or tool handover) is repeatable using gripper + teleoperation, with consistent success across repeated trials. |
| **HD** | Gripper control adapts based on object type or force/torque thresholds (e.g., soft grip for fragile objects). Task execution is logged with success/failure metrics. |
| **Perfect** | Hardware integration is robust — gripper + robot + XR work seamlessly during the final integrated demonstration under varying task conditions. |

**Pros:** Leverages Oliver's hardware/embedded strengths. Adds genuine new capability to the system (the project lists a parallel gripper in the hardware table). No overlap with Subsystem 3.

**Cons:** Requires physical gripper hardware to be available and working. Less "software" than the original criteria.

---

## Option B: MoveIt Planning and Assisted Mode (ROS-side)

Keeps the "planning and control" scope but draws a clear boundary: **Nic owns Unity-side interaction, Oliver owns ROS-side planning/MoveIt.** This aligns with the project's planned "Assisted" teleoperation mode (deliverable D5).

| Level | Proposed Criterion |
|---|---|
| **P** | MoveIt 2 is configured for the UR3e and can plan and execute collision-free trajectories in simulation (fake hardware). |
| **C** | Assisted mode uses MoveIt collision boundaries (predefined collision box or mesh) to replan trajectories, maintaining a safe working distance from obstacles. |
| **D** | The operator can switch between freeform teleoperation (Nic's RMRC) and assisted/planned mode (MoveIt) — mode switching is functional and both paths execute on the real robot. |
| **HD** | Constraint-aware velocity scaling is applied based on proximity to collision objects — the robot automatically slows as it approaches safety boundaries. Proximity bands are configurable. |
| **Perfect** | Assisted mode is smooth, responsive, and robust during the final integrated demonstration. Trajectory replanning handles dynamic obstacle updates without operator intervention. |

**Pros:** Directly addresses deliverable D3 (obstacle avoidance) and D5 (mode management with assisted mode). Clear separation from Subsystem 3 — Oliver works in ROS/MoveIt, Nic works in Unity/XR. The system evaluation explicitly requires "assisted mode" at Credit level and above.

**Cons:** MoveIt 2 integration has a learning curve. HD/Perfect depend on the perception pipeline publishing collision data (John's subsystem), though predefined collision objects can be used as a fallback.

---

## Option C: Embedded Safety and Sensor Integration

Leans fully into Oliver's STM32/embedded expertise. Focuses on hardware-level safety and real-time sensor telemetry that operates independently of the software stack.

| Level | Proposed Criterion |
|---|---|
| **P** | A hardware e-stop circuit is integrated with the UR3e safety system, independently testable without software running. |
| **C** | A sensor-based workspace monitoring system (e.g., IR/ultrasonic boundary detection) publishes safety data to ROS. |
| **D** | The hardware safety layer enforces workspace limits independently of the software control stack — the robot halts if physical boundaries are breached regardless of software state. |
| **HD** | An embedded controller provides real-time safety telemetry (motor current, joint torque estimates, proximity readings) to the operator dashboard via ROS. |
| **Perfect** | The hardware safety layer is robust under fault injection (software crash, network dropout, sensor occlusion) during the final integrated demonstration. |

**Pros:** Completely independent of all other subsystems. Directly addresses safety — a core project theme. Unique contribution that no other subsystem covers.

**Cons:** Requires sourcing/building hardware (STM32 + sensors). May be harder to justify under the "Motion Planning and Control" subsystem name — might need a name change (e.g., "Hardware Safety and Control").

---

## Recommendation

**Option B** is probably the path of least resistance for the change request:
- It keeps the subsystem name and scope recognisable.
- It directly delivers planned project features (assisted mode, obstacle avoidance).
- The boundary with Subsystem 3 is clean: **Unity/XR = Nic, ROS/MoveIt = Oliver**.
- At P and C level, it only requires MoveIt + fake hardware — no dependency on other subsystems.

That said, if Oliver wants to play to his hardware strengths, **Option A** (gripper integration) would add the most tangible new capability to the project and would make the final demo significantly more impressive.

Oliver should pick whichever option he's most confident he can deliver — he's starting from scratch so the criteria should match where he can realistically get to.
