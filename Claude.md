# HoloAssist Assessment Plan (Final)

## Comparison: GPT Recommendations vs Repo-Aware Roadmap

Your `GPT recommendations.md` is on the right track and is technically strong.

Where it matches the repo-aware approach:

1. Hybrid architecture is correct: perception + planning + control first, learning second.
2. Start narrow with one-object tabletop pick-and-place.
3. Camera-to-robot calibration is a must-do early milestone.
4. Scripted non-ML baseline before imitation learning is the safest and fastest path.
5. Use VR primarily for demonstration data collection, not direct end-to-end policy control.

What must be added because of this specific repo snapshot:

1. The repo is incomplete and must be stabilized first (missing files, broken entry points, placeholder package setup).
2. Frontend and ROS packaging fixes are immediate blockers before any robotics evaluation can run end-to-end.
3. Several runtime nodes are expected by launch files but are external or missing in this checkout.

Decision:
Use the GPT technical strategy as the core robotics method, and execute it through a repo-first stabilization plan so you can actually demo and assess results.

---

## Step-by-Step Plan for This Assessment

## Phase 0: Define Assessment Scope (Day 0)

1. Freeze scope to: one known rigid object, flat table, one fixed place zone, overhead D435i, UR3e + gripper.
2. Freeze success criteria:
   - Pick success rate target for demo day.
   - Place accuracy threshold in cm.
   - Maximum cycle time target.
3. Freeze safety envelope:
   - Reduced speed mode.
   - Workspace bounds.
   - Human exclusion zone during autonomous execution.

Deliverable: one-page assessment rubric with pass/fail criteria.

---

## Phase 1: Make This Repo Runnable (Day 1-2)

1. Fix ROS package integrity in `ros2_ws/src/holo_assist_depth_tracker`:
   - Add missing Python package markers/resources.
   - Fix `setup.py` entry points that reference missing modules.
   - Remove or implement missing `depth_tracker_node` references.
2. Fix Unity bridge package metadata and installability in `ros2_ws/src/holoassist_unity_bridge`.
3. Fix dashboard project completeness in `holoassist-dashboard`:
   - Restore missing app files or simplify imports to only existing files.
   - Ensure `npm run dev` works from source (not only `dist` output).
4. Confirm relay API works:
   - `/api/perception/status`
   - `/api/perception/debug.jpg`

Deliverable: local runbook with exact commands that start dashboard + relay without errors.

---

## Phase 2: Hardware Bringup and Smoke Tests (Day 3-4)

1. Bring up UR3e via ROS 2 driver and verify basic motion commands.
2. Bring up gripper control and verify open/close commands from ROS.
3. Bring up RealSense D435i via ROS wrapper and verify RGB, depth, pointcloud topics.
4. Validate topic contract expected by relay/dashboard:
   - Joint states visible.
   - Command topics visible.
   - Image stream visible.

Deliverable: recorded short video of robot home->pregrasp->home and live camera feed in dashboard.

---

## Phase 3: Calibration and Frames (Day 5)

1. Perform eye-to-hand extrinsic calibration (`camera -> base_link`).
2. Save transform in launch/config and publish consistently.
3. Validate with 10-point spot check:
   - Detect marker/object in camera frame.
   - Move TCP above transformed point.
   - Measure XY error.

Deliverable: calibration report with transform and mean/max positioning error.

---

## Phase 4: Deterministic Pick-and-Place Baseline (Week 2)

1. Implement simple depth pipeline:
   - Table plane segmentation.
   - Object cluster extraction.
   - Object centroid estimation.
2. Generate top-down grasp from centroid + safety offsets.
3. Execute pick-and-place sequence with MoveIt:
   - Home -> pregrasp -> grasp -> close -> lift -> preplace -> place -> open -> home.
4. Add failure handling:
   - No-object case.
   - Grasp failure retry once.
   - Timeout abort and safe return.

Deliverable: repeatable scripted baseline with metrics on at least 30 trials.

---

## Phase 5: Assessment Instrumentation and Logging (Week 2)

1. Add synchronized episode logging:
   - RGB/depth.
   - Pointcloud or object pose.
   - Joint states, TCP pose.
   - Gripper state.
   - Command/action topics.
   - Success/failure label.
2. Define dataset folder schema and metadata format.
3. Add automatic run summary:
   - Success rate.
   - Place error.
   - Average cycle time.
   - Failure categories.

Deliverable: reproducible dataset from baseline + teleop runs.

---

## Phase 6: VR Demonstration Collection (Week 3)

1. Validate VR teleop publishes stable target/action commands.
2. Collect demonstrations in controlled curriculum:
   - Stage A: one object, clean table.
   - Stage B: varied start position.
   - Stage C: simple multi-object variation.
3. Log both successful and failed demos.

Deliverable: minimum demonstration set for training (start with 50-100 clean episodes for one object type).

---

## Phase 7: Learning Component (Week 3-4)

1. Train first model for grasp pose prediction, not full joint control.
2. Inputs:
   - RGB-D crop and/or compact geometric features.
3. Outputs:
   - Grasp position offsets, yaw, grasp height, optional grasp score.
4. Keep planner in control:
   - Model proposes grasp.
   - MoveIt validates and executes safely.
5. Evaluate learned grasping against deterministic baseline.

Deliverable: A/B comparison table showing whether learning improves success rate or robustness.

---

## Phase 8: Final Assessment Package (Week 4)

1. Final demo script:
   - Baseline run.
   - Learned grasp run.
   - Dashboard monitoring view.
2. Final metrics report:
   - Pick success rate.
   - Place accuracy.
   - Time per cycle.
   - Failure modes and mitigations.
3. Final architecture diagram and limitations section.
4. Clear next steps:
   - More objects.
   - Clutter.
   - Better grasp ranking.

Deliverable: assessment submission bundle (code, runbook, logs, results, demo video).

---

## Immediate Next Actions (Do These First)

1. Fix repo blockers (packaging + missing dashboard source dependencies).
2. Get relay + dashboard + webcam fallback running from source.
3. Bring up UR3e + gripper + D435i topics and verify end-to-end visibility in dashboard.
4. Perform camera-to-robot calibration and validate point-to-point accuracy.
5. Implement deterministic top-down pick-and-place baseline before any ML training.

This is the highest-probability path to a strong, defensible assessment result.
