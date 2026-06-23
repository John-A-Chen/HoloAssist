# Perception — Reference

**Package:** `holoassist_perception`
**Entrypoint (real camera):** `./scripts/launch.sh --robot-ip <IP>`
**Entrypoint (fake hardware):** `./scripts/launch.sh`

---

## Pipeline

```
RealSense D435i
  → /camera/camera/color/image_raw
  → apriltag_ros (36h11, 0.032 m)
  → /detections_all
  → CubePoseNode (holoassist_cube_pose_node)
  → /holoassist/perception/april_cube_{1..4}_pose  (PoseStamped, ~20 Hz)
  → /holoassist/perception/april_cube_{1..4}_marker (RViz markers)
```

TF chain (after calibration):
```
world → base_link → camera_link → camera_color_optical_frame → tag36h11:XX
```
`base_link → camera_link` is the **static TF published from the saved calibration file**.

---

## Cube Tag Groups (`config/cubes.yaml`)

| Cube | Tag IDs | Colors (RViz) |
|---|---|---|
| `april_cube_1` | 10–15 | Red |
| `april_cube_2` | 16–21 | Green |
| `april_cube_3` | 22–27 | Blue |
| `april_cube_4` | 28–33 | Yellow |

Face order per cube (index 0–5): `+X, -X, +Y, -Y, +Z, -Z`

Physical tag size: **32 mm** (`cube_tag_size_m: 0.032`)
Cube edge size: **40 mm** (`cube_edge_size_m: 0.040`)
Face-to-centre offset: **20 mm** (`cube_face_offset_m: 0.020`)

---

## Key Parameters

| Param | File | Default | Effect if wrong |
|---|---|---|---|
| `size` | `config/apriltag_all.yaml` | `0.032` | Metric pose scale error |
| `cube_edge_size_m` | `config/cubes.yaml` | `0.040` | Wrong centre offset |
| `cube_face_offset_m` | `config/cubes.yaml` | `0.020` | Cube centre shifted |
| `detections_timeout_s` | `config/cubes.yaml` | `0.5` | Pose goes stale faster/slower |
| `timer_hz` | `config/cubes.yaml` | `20.0` | Publish rate |
| `candidate_consensus_threshold_m` | `config/cubes.yaml` | `0.05` | Multi-face fusion spread tolerance |

---

## Published Topics

| Topic | Type | Rate |
|---|---|---|
| `/holoassist/perception/april_cube_N_pose` | `PoseStamped` | ~20 Hz |
| `/holoassist/perception/april_cube_N_marker` | `Marker` | ~20 Hz |
| `/holoassist/perception/april_cube_pose` (legacy) | `PoseStamped` | ~20 Hz |
| `/holoassist/perception/debug_image` | `Image` | camera rate |

---

## RViz Config (`config/holoassist_full.rviz`)

Fixed frame: `base`
Displays:
- Grid (XY plane, 0.1 m cells)
- Marker per cube (namespaces `holoassist_april_cube_1..4`)
- TF tree (all robot + camera + tag frames)
- Image: `/holoassist/perception/debug_image` (RGB overlay with tag detections)
- Image: `/camera/camera/color/image_raw` (disabled by default)

---

## Launch Files

| File | What it does |
|---|---|
| `camera.launch.py` | RealSense color + depth streams only |
| `visualize_depth_tracker.launch.py` | camera + apriltag + cube_pose + RViz |
| `holoassist_4tag_board_4cube.launch.py` | apriltag + cube_pose; camera optional via `start_camera:=` |
| `sim_holoassist_trolley.launch.py` | Sim-only scene with trolley |

---

## Troubleshooting

| Symptom | Check | Fix |
|---|---|---|
| No detections | `ros2 topic echo /detections_all --once` | Lighting, tag family, printed size must be 32 mm |
| Detections but no cube poses | `ros2 node list \| grep cube_pose` | Node not running; tag IDs must be 10–33 |
| Cube poses but Unity not showing | `ros2 topic echo /holoassist/unity/cube_1_pose` | Run topic_tools relay (see run-guide.md §4) |
| TF disconnected | `ros2 run tf2_ros tf2_echo base_link camera_link` | Re-run `./scripts/calibrate.sh` |
| Pose offset/scale wrong | Measure printed tag edge | Update `size:` in `apriltag_all.yaml` |
