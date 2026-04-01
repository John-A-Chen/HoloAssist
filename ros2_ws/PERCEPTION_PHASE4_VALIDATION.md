# HoloAssist Perception Phase 4 Validation Pack

This document formalises three subsystem tests for Sprint 2 presentation use.

Scope:
- Subsystem: Perception and Mapping
- Focus: defensible validation of current implementation and collected evidence
- Out of scope: claiming completed MoveIt planning integration

## 1) Verified Current Interfaces (from repo)

Perception input topics:
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`

Perception output topics:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

Relay HTTP endpoints used by dashboard:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

Current diagnostics exposure:
- No dedicated ROS `/diagnostics` topic in the perception stack.
- Operational diagnostics are exposed through relay status JSON fields (`tracker_connection`, `rates_hz`, `counters`, `latest`, `capabilities`).

## 2) Evidence Baseline Already Collected

Recorded bags:
- `2026-04-01_pass_workspace_monitor_remote_ui`
- `2026-04-01_credit_boundary_candidate_human_entry`
- `2026-04-01_rate_gate_trial_01`

Measured live rate evidence (primary):
- `/holo_assist_depth_tracker/bbox` about 4.6 Hz
- `/holo_assist_depth_tracker/obstacle_marker` about 4.0 to 4.5 Hz

Dashboard evidence support:
- Rolling 60 s time-series chart for bbox and obstacle marker rates
- Rolling freshness and point-count chart
- Status badges for relay, tracker, source mode, and obstacle state

## 3) Formal Subsystem Tests

### Test 1 - Independent Workspace Monitoring on Remote Desktop UI (Pass)

Requirement validated:
- Depth camera monitors workspace independently of robot and publishes to desktop UI on a separate device.

Assumptions:
- RealSense D435i is running and visible to ROS 2.
- Dashboard relay is reachable from second device.
- Source mode is `depth_tracker` (not RGB fallback).

Procedure:
1. Launch RealSense + depth tracker + dashboard relay.
2. Open dashboard on a second device over network.
3. Observe live debug image and status badges.
4. Move hand/object in and out of monitored workspace for ~20 s.

Pass criteria:
- Dashboard on second device remains live and responsive.
- `Tracker` shows live (not stale).
- Camera source indicates RealSense depth mode.
- BBox and obstacle activity update with workspace changes.

Fail criteria:
- Remote dashboard cannot connect or repeatedly stalls.
- Source drops to `no_data` for sustained period.
- No bbox/obstacle updates during visible workspace motion.

Evidence collected:
- Remote dashboard screenshot showing live stream and status badges.
- Optional short clip showing hand/object entry and live update.

Supporting rosbag:
- `2026-04-01_pass_workspace_monitor_remote_ui`

Presentation one-liner:
- "This test confirms independent depth-based workspace monitoring on a separate desktop UI in real time."

### Test 2 - Obstacle Region Extraction and Boundary Candidate Generation (Early Credit Progress)

Requirement validated:
- Perception output supports assisted mode groundwork by producing boundary candidates suitable for planning handoff.

Assumptions:
- Depth tracker running in RealSense depth mode.
- Obstacle marker publication enabled.
- Test scene includes a human hand or object entering the workspace band.

Procedure:
1. Run tracker and RViz/dashboard.
2. Start with clear workspace, then introduce hand/object into target region.
3. Hold briefly, then remove object.
4. Observe bbox and obstacle marker behaviour.

Pass criteria:
- Valid bbox appears when obstacle region is present.
- Obstacle marker publishes a cube candidate when region is detected.
- Marker deactivates/removes when region exits or no valid blob remains.

Fail criteria:
- No valid bbox during clear obstacle presence.
- No obstacle marker add/delete behaviour tied to scene change.

Evidence collected:
- RViz screenshot with pointcloud + obstacle marker.
- Dashboard screenshot of bbox details and obstacle active state.

Supporting rosbag:
- `2026-04-01_credit_boundary_candidate_human_entry`

Presentation one-liner:
- "This test shows depth-derived obstacle regions converted into a live 3D boundary candidate for later planning integration."

### Test 3 - Perception Update-Rate and Instrumentation Evidence (Monitored Distinction Groundwork)

Requirement validated:
- Perception update behaviour is measurable and monitored, providing groundwork toward gated ~5 Hz safety-boundary updates.

Assumptions:
- Dashboard relay status endpoint and rolling charts are active.
- Live rate checks are taken from bbox and obstacle topics.
- Rate claim is based on measured evidence, not on pointcloud rate.

Procedure:
1. Run perception pipeline and dashboard.
2. Observe rolling 60 s rate chart.
3. Measure live rates using `ros2 topic hz` for bbox and obstacle marker.
4. Capture screenshot of chart + terminal rate output.

Pass criteria:
- Dashboard shows stable rolling time-series for bbox and obstacle rates.
- Live rates are consistently in approximately 4 to 5 Hz range.
- Freshness metrics indicate active stream (not stale) during measurement.

Fail criteria:
- Rates are not observable in dashboard/CLI.
- Stream frequently stale during measurement window.

Evidence collected:
- Dashboard screenshot of rolling rate chart.
- Terminal screenshot for `ros2 topic hz` outputs.

Supporting rosbag:
- `2026-04-01_rate_gate_trial_01`

Presentation one-liner:
- "This test demonstrates measured and instrumented perception update behaviour, currently around 4-5 Hz, as groundwork toward gated updates."

## 4) Test-to-Evidence Mapping

| Test | Primary screenshots | Rosbag | Notes |
|---|---|---|---|
| Test 1 (Pass) | Remote dashboard Overview with live stream + status badges | `2026-04-01_pass_workspace_monitor_remote_ui` | Proves independent remote monitoring path |
| Test 2 (Early Credit) | RViz pointcloud + obstacle marker, Perception tab bbox/obstacle panel | `2026-04-01_credit_boundary_candidate_human_entry` | Proves extraction + boundary-candidate output |
| Test 3 (Rate instrumentation) | Rolling rate chart (bbox/obstacle), `ros2 topic hz` terminal capture | `2026-04-01_rate_gate_trial_01` | Use bbox + obstacle as primary evidence |

## 5) Claim Boundaries (Do Not Overclaim)

Allowed claims now:
- Independent workspace monitoring on a separate desktop UI is working.
- Obstacle-region extraction and 3D boundary-candidate output are working.
- Update rates are instrumented and measurable at approximately 4 to 5 Hz for bbox/obstacle outputs.

Avoid claiming now:
- Completed MoveIt planning-scene integration from this perception path.
- Fully tuned distinction-level gated boundary update logic.
- Pointcloud frequency as primary rate evidence.

## 6) Suggested Slide Captions

- Test 1: "Remote desktop dashboard showing independent depth-based workspace monitoring."
- Test 2: "Obstacle extraction with bbox and 3D marker boundary candidate from live depth data."
- Test 3: "Rolling telemetry confirms monitored perception update behaviour (bbox/obstacle ~4-5 Hz)."

