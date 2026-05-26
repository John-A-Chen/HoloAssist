# Perception Subsystem

**Lead:** John Chen  
**Package:** `holo_assist_depth_tracker`  
**Camera:** Logitech Brio 4K (auto-detected; RealSense also supported)

---

## AprilTag Cube Detection

The perception pipeline tracks up to 4 physical 40 mm cubes using AprilTag stickers (36h11 family, 32 mm printed size). Tag IDs are grouped per cube:

| Cube | Tag IDs | RViz colour |
|---|---|---|
| Cube 1 | 10, 11, 12, 13, 14, 15 | Red |
| Cube 2 | 16, 17, 18, 19, 20, 21 | Green |
| Cube 3 | 22, 23, 24, 25, 26, 27 | Blue |
| Cube 4 | 28, 29, 30, 31, 32, 33 | Orange |

Each face maps to a cube axis in order: `+X, -X, +Y, -Y, +Z, -Z`.

### Multi-face fusion

When multiple faces are visible simultaneously, the node:

1. Computes a candidate cube centre from each visible face (tag position + half-edge offset)
2. Runs a consensus filter — candidates within 5 cm of each other are averaged; outliers are rejected
3. Applies a 3 mm deadzone to suppress jitter

### Object persistence

Once a cube is detected for the first time, its last known position is remembered. If all tags leave the camera frame:

- The RViz marker **freezes** at the last position (50% alpha, lifetime = 0 = never expires)
- Pose topics stop publishing until the cube is visible again
- Bin verification continues against the frozen position

---

## Published Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/holoassist/perception/april_cube_N_pose` | `PoseStamped` | 30 Hz | Cube N centre pose in `camera_color_optical_frame` |
| `/holoassist/perception/april_cube_N_marker` | `Marker` | 30 Hz | RViz cube marker |
| `/holoassist/perception/april_cube_N_status` | `String` | 30 Hz | Detection state, visible tags, spread, method |
| `/holoassist/perception/april_cube_N_bin_check` | `String` | 20 Hz | Bin verification result (see below) |
| `/holoassist/perception/april_cube_pose` | `PoseStamped` | 30 Hz | Legacy: cube 1 pose (or nearest detected cube) |

---

## Bin Verification

After calibration, the node checks whether each cube's position (in `base_link` frame) is within the XY footprint of a known bin.

Bins are loaded from `moveit_robot_control/config/bin_poses.json`:

| Bin | X (m) | Y (m) |
|---|---|---|
| bin_1 | +0.30 | 0.00 |
| bin_2 | −0.30 | 0.00 |
| bin_3 | +0.30 | −0.20 |
| bin_4 | +0.30 | −0.10 |

### Message format

```
cube_id=1 sorted=true  bin=bin_1 distance_xy_m=0.034 margin_m=0.080
cube_id=2 sorted=false bin=none  distance_xy_m=0.241 margin_m=0.080
```

- `sorted=true` when `distance_xy < bin_xy_margin_m` (default **8 cm**)
- Z is ignored — only XY footprint is checked
- Works on frozen cubes (cube picked, tags now hidden, but last position remembered)
- Publishes at 20 Hz for all cubes seen at least once in the session

### Tuning the margin

If calibration drift causes false negatives, increase the margin in `cubes.yaml`:

```yaml
bin_xy_margin_m: 0.10   # 10 cm — more forgiving
```

### Monitor live

```bash
ros2 topic echo /holoassist/perception/april_cube_1_bin_check
```

### Prerequisites

- Hand-eye calibration must be loaded so TF `camera_color_optical_frame → base_link` is available
- Without calibration, the node logs a throttled warning and skips the check (no crash)

---

## Configuration (`cubes.yaml`)

Key parameters in `ros2_ws/src/holo_assist_depth_tracker/config/cubes.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `cube_edge_size_m` | 0.040 | Physical cube side length |
| `cube_tag_size_m` | 0.032 | Printed AprilTag side length |
| `detections_timeout_s` | 0.5 | Age after which a detection is stale |
| `candidate_consensus_threshold_m` | 0.05 | Max spread between face estimates to agree |
| `bin_check_frame` | `base_link` | TF frame for bin comparison |
| `bin_xy_margin_m` | 0.08 | XY tolerance for bin match (m) |
| `timer_hz` | 20.0 | Publishing rate for timers |

---

## Camera Auto-Detection

The launch script detects cameras in priority order: **RealSense → Brio → generic webcam**. No flag is needed; the camera is auto-selected. The Brio preset uses:

- Resolution: 848 × 480
- FPS: 30
- HFOV: 78°
- Frame ID: `camera_color_optical_frame`

---

## Status Message Format

Each `april_cube_N_status` message contains:

```
cube_id=1 state=visible reason=ok visible_tag_ids=[10, 12] selected_tag_id=10
rejected_tag_ids=[] candidate_count=2 candidate_spread_m=0.0023
method=consensus_center_keep_previous_orientation age_s=0.012
frame_id=camera_color_optical_frame position_m=(0.123,-0.045,0.312)
```

`state` values: `visible` | `stale` | `waiting_for_detections`
