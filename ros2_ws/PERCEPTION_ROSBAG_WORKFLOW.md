# HoloAssist Perception Rosbag Workflow (Phase 3)

This workflow is scoped to the perception subsystem evidence path and is designed to be short, repeatable, and presentation-friendly.

## 1) Confirmed Current Interfaces

From current repository code:

- Depth tracker input:
  - `/camera/camera/depth/image_rect_raw`
  - `/camera/camera/depth/camera_info`
- Depth tracker outputs:
  - `/holo_assist_depth_tracker/debug_image`
  - `/holo_assist_depth_tracker/bbox`
  - `/holo_assist_depth_tracker/pointcloud`
  - `/holo_assist_depth_tracker/obstacle_marker`
- Relay subscriptions relevant to perception:
  - `/holo_assist_depth_tracker/debug_image`
  - `/holo_assist_depth_tracker/bbox`
  - `/holo_assist_depth_tracker/pointcloud`
  - `/holo_assist_depth_tracker/obstacle_marker`
- Relay status endpoints (HTTP, not ROS topics):
  - `GET /api/perception/status`
  - `GET /api/perception/debug.jpg`
- Current diagnostics state:
  - No dedicated ROS diagnostics topic is currently published by the perception stack.

## 2) Bag Naming Scheme

Use:

`YYYY-MM-DD_<test_suffix>`

Primary suffixes for the three subsystem tests:

- `pass_workspace_monitor_remote_ui`
- `credit_boundary_candidate_human_entry`
- `rate_gate_trial_01`

Examples:

- `2026-04-01_pass_workspace_monitor_remote_ui`
- `2026-04-01_credit_boundary_candidate_human_entry`
- `2026-04-01_rate_gate_trial_01`

## 3) Recording Matrix (3 Tests)

| Test | Bag Name Suffix | Duration | Topics Profile | Physical Action | Expected Outcome | Screenshot / Clip |
|---|---|---:|---|---|---|---|
| Test A: Independent workspace monitoring on remote desktop UI | `pass_workspace_monitor_remote_ui` | 45 s | `pass_workspace_monitor_remote_ui.topics` | Keep camera and workspace visible; open dashboard on second device; slowly move hand/object in and out of view | Remote dashboard stays live; bbox and obstacle updates track workspace changes | Dashboard on second device with live image + status badges |
| Test B: Obstacle extraction and boundary candidate generation | `credit_boundary_candidate_human_entry` | 60 s | `credit_boundary_candidate_human_entry.topics` | Enter object/human hand into robot region, pause, then exit | BBox appears over obstacle region; obstacle marker toggles active/inactive; pointcloud context captured | RViz marker + pointcloud + dashboard perception panel |
| Test C: Update-rate and gating instrumentation evidence | `rate_gate_trial_01` | 75 s | `rate_gate_trial_01.topics` | Run stable scene 20 s, then controlled motion for 30 s, then still scene | BBox and obstacle marker streams show rate behaviour over time; replay supports rate verification | Dashboard rate charts + `ros2 topic hz` terminal capture for bbox/obstacle |

## 4) Topic Sets

Topic profiles are stored in:

- `ros2_ws/scripts/rosbag_profiles/pass_workspace_monitor_remote_ui.topics`
- `ros2_ws/scripts/rosbag_profiles/credit_boundary_candidate_human_entry.topics`
- `ros2_ws/scripts/rosbag_profiles/rate_gate_trial_01.topics`

Main evidence path for rates:

- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/obstacle_marker`

Pointcloud is recorded as secondary context (not primary rate evidence).

## 5) Exact Record Commands

Run from workspace root with ROS sourced:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

### Test A

```bash
ros2 bag record \
  -o "$HOME/rosbags/holoassist/$(date +%F)_pass_workspace_monitor_remote_ui" \
  --compression-mode file --compression-format zstd \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /holo_assist_depth_tracker/debug_image \
  /holo_assist_depth_tracker/bbox \
  /holo_assist_depth_tracker/obstacle_marker \
  /tf /tf_static
```

### Test B

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
  /tf /tf_static
```

### Test C

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
  /tf /tf_static
```

## 6) Helper Scripts

Record (profile based):

```bash
./ros2_ws/scripts/perception_rosbag_record.sh test_a 45
./ros2_ws/scripts/perception_rosbag_record.sh test_b 60
./ros2_ws/scripts/perception_rosbag_record.sh test_c 75
```

Play:

```bash
./ros2_ws/scripts/perception_rosbag_play.sh 2026-04-01_pass_workspace_monitor_remote_ui
./ros2_ws/scripts/perception_rosbag_play.sh "$HOME/rosbags/holoassist/2026-04-01_rate_gate_trial_01" 0.5
```

## 7) Playback and Verification Commands

```bash
ros2 bag info "$HOME/rosbags/holoassist/<bag_name>"
ros2 bag play "$HOME/rosbags/holoassist/<bag_name>" --clock
```

Rate check during playback:

```bash
ros2 topic hz /holo_assist_depth_tracker/bbox
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker
```

## 8) Storage Guidance

- Keep bags short (45 to 75 seconds) for fast replay and clean evidence.
- Use one bag per test objective (avoid giant mixed recordings).
- Use zstd compression (`--compression-mode file --compression-format zstd`).
- Store under:
  - `$HOME/rosbags/holoassist`
- Archive final selected evidence bags separately before demo day.
