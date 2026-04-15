# HoloAssist Perception Phase 4 Validation Pack

Last updated: 2026-04-15
Scope: Perception + Foxglove observability evidence path

## 1) Verified Interfaces (Current Branch)

Perception input topics:
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`

Perception output topics:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

Foxglove observability overlays now available:
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/runtime`
- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/pointcloud_hz`

Legacy relay endpoints (compat path):
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

## 2) Evidence Baseline

Recorded bag baseline:
- `2026-04-01_pass_workspace_monitor_remote_ui`
- `2026-04-01_credit_boundary_candidate_human_entry`
- `2026-04-01_rate_gate_trial_01`

Observed rate evidence target:
- `/holo_assist_depth_tracker/bbox` around 4 to 5 Hz
- `/holo_assist_depth_tracker/obstacle_marker` around 4 to 5 Hz

## 3) Formal Test Set

### Test 1: Remote workspace monitoring (pass path)

Requirement:
- Perception stream visible remotely and usable in runtime operations.

Procedure:
1. Start depth tracker stack.
2. Start Foxglove runtime (`holoassist_foxglove`) and connect from second device.
3. Move object/hand in and out of workspace for ~20 s.

Pass criteria:
- Debug image updates remotely.
- BBox/marker react to scene changes.
- `/holoassist/diagnostics` shows perception section healthy or at most transient WARN.

Evidence:
- Foxglove screenshot (Image + 3D + diagnostics panel)
- Optional relay dashboard screenshot for compatibility proof

Supporting bag:
- `2026-04-01_pass_workspace_monitor_remote_ui`

### Test 2: Obstacle extraction + boundary candidate generation

Requirement:
- Depth-derived obstacle region and marker candidate available for planning handoff.

Procedure:
1. Start depth tracker and visualization.
2. Insert object/hand into monitored region.
3. Hold briefly, then remove.

Pass criteria:
- BBox appears when obstacle present.
- Obstacle marker appears/updates in 3D.
- Marker deactivates when region clears.

Evidence:
- Foxglove 3D panel screenshot with pointcloud + marker
- Optional RViz screenshot for secondary confirmation

Supporting bag:
- `2026-04-01_credit_boundary_candidate_human_entry`

### Test 3: Rate/instrumentation evidence

Requirement:
- Perception stream timing is measurable and visible in runtime telemetry.

Procedure:
1. Run perception + observability stack.
2. Observe rates in Foxglove plots and CLI.
3. Capture 30-60 s window for stable evidence.

Pass criteria:
- Measurable BBox/obstacle rates (~4-5 Hz target zone).
- Foxglove metrics and `ros2 topic hz` broadly align.
- Stream freshness remains active during capture window.

Evidence:
- Foxglove plot screenshot
- Terminal `ros2 topic hz` output screenshot

Supporting bag:
- `2026-04-01_rate_gate_trial_01`

## 4) Test-to-Evidence Mapping

| Test | Primary evidence | Bag | Notes |
|---|---|---|---|
| Test 1 | Foxglove Image + diagnostics + remote access screenshot | `*_pass_workspace_monitor_remote_ui` | Proves remote monitoring path |
| Test 2 | Foxglove 3D (pointcloud + marker) screenshot | `*_credit_boundary_candidate_human_entry` | Proves boundary candidate extraction |
| Test 3 | Foxglove rate plots + `ros2 topic hz` output | `*_rate_gate_trial_01` | Proves measurable update behavior |

## 5) Claim Boundaries

Allowed claims now:
- Remote perception monitoring works.
- Obstacle extraction + marker candidate output works.
- Update-rate behavior is instrumented and observable.

Avoid claiming now:
- Fully completed planning-scene integration via perception path.
- Fully tuned distinction-grade gating logic.
- Hardware-grade safety guarantee from perception alone.

## 6) No-Hardware Behavior

If no RealSense is connected:
- Perception diagnostics will show stale/missing stream WARN/ERROR.
- Foxglove stack still runs and records these states for bringup validation.
- Webcam fallback path can still be used for basic image relay checks:

```bash
ros2 launch holo_assist_depth_tracker webcam_rgb_monitor.launch.py
```

## 7) Rosbag Workflow (Merged)

### Naming

Pattern:
- `YYYY-MM-DD_<test_suffix>`

Suggested suffixes:
- `pass_workspace_monitor_remote_ui`
- `credit_boundary_candidate_human_entry`
- `rate_gate_trial_01`

### Recording setup

```bash
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash
```

### Test A recording command

```bash
ros2 bag record \
  -o "$HOME/rosbags/holoassist/$(date +%F)_pass_workspace_monitor_remote_ui" \
  --compression-mode file --compression-format zstd \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /holo_assist_depth_tracker/debug_image \
  /holo_assist_depth_tracker/bbox \
  /holo_assist_depth_tracker/obstacle_marker \
  /holoassist/diagnostics \
  /holoassist/events \
  /holoassist/state/runtime \
  /tf /tf_static
```

### Test B recording command

```bash
ros2 bag record \
  -o "$HOME/rosbags/holoassist/$(date +%F)_credit_boundary_candidate_human_entry" \
  --compression-mode file --compression-format zstd \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /holo_assist_depth_tracker/debug_image \
  /holo_assist_depth_tracker/bbox \
  /holo_assist_depth_tracker/pointcloud \
  /holo_assist_depth_tracker/obstacle_marker \
  /holoassist/diagnostics \
  /holoassist/events \
  /tf /tf_static
```

### Test C recording command

```bash
ros2 bag record \
  -o "$HOME/rosbags/holoassist/$(date +%F)_rate_gate_trial_01" \
  --compression-mode file --compression-format zstd \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /holo_assist_depth_tracker/debug_image \
  /holo_assist_depth_tracker/bbox \
  /holo_assist_depth_tracker/obstacle_marker \
  /holo_assist_depth_tracker/pointcloud \
  /holoassist/diagnostics \
  /holoassist/metrics/debug_image_hz \
  /holoassist/metrics/pointcloud_hz \
  /tf /tf_static
```

### Scripted helpers

```bash
cd ~/git/RS2-HoloAssist/john
./ros2_ws/scripts/perception_rosbag_record.sh test_a 45
./ros2_ws/scripts/perception_rosbag_record.sh test_b 60
./ros2_ws/scripts/perception_rosbag_record.sh test_c 75
```

Playback:

```bash
./ros2_ws/scripts/perception_rosbag_play.sh 2026-04-01_pass_workspace_monitor_remote_ui
./ros2_ws/scripts/perception_rosbag_play.sh "$HOME/rosbags/holoassist/2026-04-01_rate_gate_trial_01" 0.5
```

### Verification

```bash
ros2 bag info "$HOME/rosbags/holoassist/<bag_name>"
ros2 bag play "$HOME/rosbags/holoassist/<bag_name>" --clock
ros2 topic hz /holo_assist_depth_tracker/bbox
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker
```
