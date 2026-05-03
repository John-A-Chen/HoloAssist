# Perception Pipeline Reference (`holo_assist_depth_tracker`)

This document covers the AprilTag-based perception pipeline used in the `j0hn` branch for both hardware operation and as the reference model for simulation.

---

## 1. Launch entry points

### Full XR perception + rosbridge (primary hardware entry point)

```bash
ros2 launch holo_assist_depth_tracker holoassist_full_xr.launch.py
```

Starts: RealSense camera, apriltag_ros, workspace_board_node, cube_pose_node, rosbridge WebSocket (port 9090).

Unity/HoloLens connects to `ws://<host>:9090` and subscribes to cube pose topics.

Optional args:
- `start_camera:=false` — skip camera launch (use existing stream)
- `start_rosbridge:=false` — skip rosbridge
- `rosbridge_port:=9090` — change WebSocket port

### AprilTag pipeline only (no rosbridge)

```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

Optional: `start_camera:=true` to start RealSense.

### Workspace perception from depth pointcloud

```bash
ros2 launch holo_assist_depth_tracker workspace_perception.launch.py
```

Uses `workspace_perception_node` — RANSAC plane fitting + depth blob tracking alternative.

---

## 2. Startup dependency chain

```
RealSense camera starts
  → apriltag_ros sees tags → publishes /detections_all + TF frames
      → workspace_board_node sees all 4 board tags (IDs 0–3)
          → SVD solve passes geometry checks
              → workspace_frame TF locked (published continuously)
                  → cube_pose_node can now look up tag TFs in workspace_frame
                      → cube poses published when face tags visible
```

If the board is not fully visible (fewer than 4 corner tags), workspace_frame is not published and cube poses are silently absent. Call `/holoassist/perception/realign_workspace` to re-solve after repositioning.

---

## 3. `holoassist_workspace_board_node`

**File:** `holo_assist_depth_tracker/nodes/workspace_board_node.py`

**Role:** Camera self-localisation — derives the camera's position and orientation relative to the workspace from 4 board corner tags, then publishes the inverse as a TF.

### Algorithm

1. apriltag_ros publishes each board tag as a TF: `camera_optical_frame → tag36h11:<id>`
2. workspace_board_node looks up all 4 tag positions in camera frame
3. SVD (Kabsch algorithm) aligns model board corner positions to observed positions:
   - Solves rotation `R` and translation `t` such that `observed ≈ R·model + t`
4. Validates: RMS residual < `geometry_rms_tolerance_m` (0.02 m) and all pairwise distances within `dimension_tolerance_m` (0.04 m)
5. On first valid solve: **locks** — publishes the locked transform continuously until realign service is called
6. Also publishes `workspace_frame → ur3e_base_link0` at the known physical robot position

### Board model (workspace.yaml)

```
Tag IDs:  [0, 1, 2, 3]  — corners: [TL, TR, BL, BR] in workspace XY plane
Board:    700 mm wide (X), 500 mm deep (Y)
Tag size: 32 mm printed; tag centres are 16 mm from board edges
Origin:   board corner (tag 0 centre) = workspace_frame origin
```

Corner positions in workspace_frame (metres):
```
Tag 0 (TL): (0.016, 0.016, 0)
Tag 1 (TR): (0.684, 0.016, 0)
Tag 2 (BL): (0.016, 0.484, 0)
Tag 3 (BR): (0.684, 0.484, 0)
```

### Robot position in workspace_frame

Published as TF `workspace_frame → ur3e_base_link0` and topic `/holoassist/perception/ur3e_base_link0_pose`:
```yaml
robot_x_m: 0.450   # 450 mm from left board edge
robot_y_m: 0.564   # 564 mm from front board edge (500 mm board depth + 64 mm overhang)
robot_z_m: -0.015  # UR3e base joint is 15 mm below board surface
```

### Published TFs

| Parent | Child | Description |
|---|---|---|
| `camera_color_optical_frame` | `workspace_frame` | Camera-relative workspace (locked after first solve) |
| `workspace_frame` | `ur3e_base_link0` | Known robot base position in workspace |

### Key parameters (workspace.yaml)

| Parameter | Default | Description |
|---|---|---|
| `board_tag_ids` | [0,1,2,3] | AprilTag IDs at board corners |
| `board_width_m` | 0.700 | Board width (workspace X extent) |
| `board_depth_m` | 0.500 | Board depth (workspace Y extent) |
| `board_tag_center_edge_offset_m` | 0.016 | Tag centre distance from board edge |
| `geometry_rms_tolerance_m` | 0.02 | Maximum RMS residual for valid solve |
| `dimension_tolerance_m` | 0.04 | Maximum pairwise distance error |
| `robot_tf_child_frame` | ur3e_base_link0 | Frame name for robot base TF child |

**Integration note:** On hardware without MoveIt, `ur3e_base_link0` and the robot URDF's `base_link` are separate TF trees. To merge them (for future MoveIt integration), change `robot_tf_child_frame: base_link` in workspace.yaml.

### Published topics

| Topic | Type | Description |
|---|---|---|
| `/holoassist/perception/workspace_mode` | String | "LOCKED" or "INVALID" |
| `/holoassist/perception/workspace_diagnostics` | DiagnosticArray | RMS residual, visible tags, lock state |
| `/holoassist/perception/bench_plane_coefficients` | Float32MultiArray | Plane equation [nx, ny, nz, d] |
| `/holoassist/perception/ur3e_base_link0_pose` | PoseStamped | Robot base in workspace_frame |
| `/holoassist/perception/ur3e_base_link0_marker` | Marker | Cylinder marker for robot visualisation |

### Re-lock service

```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```

Clears the lock; next valid full-board detection re-locks.

---

## 4. `holoassist_cube_pose_node`

**File:** `holo_assist_depth_tracker/nodes/cube_pose_node.py`

**Role:** Track all 4 physical cubes independently by aggregating visible face-tag detections into stable cube centre poses.

### Algorithm

1. apriltag_ros publishes each detected tag as a TF in camera frame
2. cube_pose_node looks up each tag's TF in `workspace_frame`
3. For each tag: compute candidate cube centre = tag_centre + face_offset in tag-local +Z direction
4. For each cube: collect all candidate centres from visible face tags
5. Consensus filter: keep candidates within `candidate_consensus_threshold_m` (0.05 m) of each other; discard outliers
6. Average remaining candidates → cube centre
7. Select best-facing tag for orientation
8. Publish pose + marker + status per cube; broadcast TF

### Cube tag assignment

| Cube | Tag IDs | Colours |
|---|---|---|
| april_cube_1 | 10–15 | Red |
| april_cube_2 | 16–21 | Green |
| april_cube_3 | 22–27 | Blue |
| april_cube_4 | 28–33 | Orange |

Face order per cube (mapped to tag index 0–5):
```
["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
```

### Physical dimensions (cubes.yaml)

```yaml
cube_edge_size_m: 0.040       # 40 mm cube body
cube_tag_size_m: 0.032        # 32 mm printed tag
cube_face_offset_m: 0.020     # distance from cube surface to cube centre (half edge)
candidate_consensus_threshold_m: 0.05  # max spread for multi-face fusion
```

### Published topics (per cube N = 1–4)

| Topic | Type | Description |
|---|---|---|
| `/holoassist/perception/april_cube_N_pose` | PoseStamped | Cube centre in workspace_frame |
| `/holoassist/perception/april_cube_N_marker` | Marker | Coloured cube visual |
| `/holoassist/perception/april_cube_N_status` | String | Tracking status message |
| `/holoassist/perception/april_cube_pose` | PoseStamped | Legacy: first visible cube |

### Published TFs (per cube N = 1–4)

`workspace_frame → apriltag_cube_N`

### Dependency

Cube poses are published in `workspace_frame`. If `workspace_frame` is not in TF (board not yet locked), the node silently produces no output.

---

## 5. XR streaming

rosbridge WebSocket server (port 9090) is included in `holoassist_full_xr.launch.py` and `full_holoassist_hardware.launch.py`.

Unity or HoloLens apps connect to `ws://<host>:9090` using:
- **Unity Robotics Hub** package → `ROSBridgeWebSocketConnection`
- **roslibjs** for web-based apps

**Topics to subscribe from Unity:**

| Topic | Type | Description |
|---|---|---|
| `/holoassist/perception/april_cube_1_pose` | PoseStamped | Cube 1 position in workspace_frame |
| `/holoassist/perception/april_cube_2_pose` | PoseStamped | Cube 2 |
| `/holoassist/perception/april_cube_3_pose` | PoseStamped | Cube 3 |
| `/holoassist/perception/april_cube_4_pose` | PoseStamped | Cube 4 |
| `/holoassist/perception/workspace_mode` | String | "LOCKED" — board is calibrated |
| `/holoassist/perception/ur3e_base_link0_pose` | PoseStamped | Robot base in workspace_frame |

Unity should treat `workspace_frame` as its world anchor. Cube positions are relative to the board corner (tag 0).

**Workspace frame is the XR anchor.** All cube poses are expressed relative to the board corner (tag 0). Unity should place the virtual board at a known physical location and apply cube positions relative to that origin.

---

## 6. AprilTag configuration

**File:** `config/apriltag_all.yaml`

Tag family: `tag36h11`
Tag size: 0.032 m (32 mm printed)
All 34 IDs active: `[0, 1, 2, 3, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33]`

Detection settings:
```yaml
max_hamming: 1        # allow 1-bit error correction
pose_estimation_method: pnp
detector:
  threads: 2
  decimate: 1.0       # no downsampling
  refine: true
  sharpening: 0.25
```

---

## 7. `holoassist_detection_merge_node`

Use only when two independent detection streams exist (e.g. two cameras or two apriltag_ros instances). Merges `/detections_a` and `/detections_b` into `/detections_all`, deduplicating by tag ID and discarding detections older than `stale_timeout_s` (default 1.0 s).

---

## 8. Operational checks

```bash
# Is the board being detected?
ros2 topic echo /detections_all --once
ros2 topic echo /holoassist/perception/workspace_mode --once

# Is workspace_frame in TF?
ros2 run tf2_ros tf2_echo camera_color_optical_frame workspace_frame

# Are cubes publishing?
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/perception/april_cube_1_status --once

# Full TF chain check
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1

# Re-lock workspace if camera moved
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```
