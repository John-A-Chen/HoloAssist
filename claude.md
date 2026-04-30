# CLAUDE Master Context

Generated on: 2026-04-30

## Purpose
This document consolidates markdown documentation across the workspace into one canonical reference, and adds a current-state summary of active goals/work.

## Current Goals (Active)
- Complete deterministic AprilTag-first perception pipeline for robotics workspace.
- Use dual detector architecture (board + cubes), merged into one deduplicated detection stream.
- Keep legacy compatibility while introducing locked 4-tag board workspace frame and 4-cube grouped pose estimation.
- Maintain RViz/Foxglove/Unity interoperability and production-quality ROS2 node/launch/docs structure.

## Current Work Completed (Latest Refactor)
- Added two-detector pipeline config/launch paths and merge node output `/detections_all`.
- Added `workspace_board_node` with strict 4-tag board solve, lock/realign behavior, and fixed robot TF `workspace_frame -> ur3e_base_link0` at `(0.450, 0.564, 0.0)`.
- Added `cube_pose_node` for 4 cubes (IDs 10–33 grouped by six), publishing TF/pose/marker/status per cube and legacy alias `/holoassist/perception/april_cube_pose`.
- Added `overlay_node` with merged tag overlay and `tags=<count> age=<seconds>` footer.
- Updated depth tracker defaults to subscribe to `/detections_all` and track only cube IDs 10–33.
- Added new launch profile `holoassist_4tag_board_4cube.launch.py` and optional stack integration flag in `holoassist_foxglove` (`enable_4tag_board_4cube_pipeline`, default off).
- Added package-level README and algorithm unit tests for merge + workspace solver logic.

## Included Markdown Sources
- claude.md (repository snapshot)
- README.md
- Unity/My project/Assets/Samples/XR Hands/1.7.1/HandVisualizer/README.md
- aprilcubes/rg2_cube_1/README.md
- aprilcubes/rg2_cube_2/README.md
- aprilcubes/rg2_cube_3/README.md
- aprilcubes/rg2_cube_4/README.md
- holoassist-dashboard/README.md
- ros2_ws/FOXGLOVE_DISCOVERY_REPORT.md
- ros2_ws/FOXGLOVE_RUNTIME.md
- ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md
- ros2_ws/PERCEPTION_PHASE4_VALIDATION.md
- ros2_ws/src/ROS-TCP-Endpoint/CHANGELOG.md
- ros2_ws/src/ROS-TCP-Endpoint/CODE_OF_CONDUCT.md
- ros2_ws/src/ROS-TCP-Endpoint/CONTRIBUTING.md
- ros2_ws/src/ROS-TCP-Endpoint/README.md
- ros2_ws/src/holo_assist_depth_tracker/.pytest_cache/README.md
- ros2_ws/src/holo_assist_depth_tracker/README.md
- ros2_ws/src/holoassist_foxglove/README.md
- ros2_ws/src/holoassist_manager/README.md
- ros2_ws/src/ur3_joint_position_controller/README.md
- ros2_ws/src/ur3_keyboard_teleop/README.md

---

## Full Markdown Corpus

### Source: claude.md (repository snapshot)

# HoloAssist Architecture Notes (Foxglove Era)

Last updated: 2026-04-15
Branch: `john`

## What Was Completed

This branch has been consolidated into a Foxglove-first runtime flow.

Completed integration work:
- Added `holoassist_foxglove` package for runtime observability aggregation and Foxglove-oriented topic contracts.
- Integrated `holoassist_manager` into the runtime flow for mode + heartbeat diagnostics.
- Integrated motion/teleop packages from other branches:
  - `holoassist_movement`
  - `ur3_keyboard_teleop`
  - `ur3_joint_position_controller`
- Wired `foxglove_bridge` into launch paths:
  - `holoassist_foxglove/launch/observability.launch.py`
  - `holoassist_foxglove/launch/holoassist_foxglove_runtime.launch.py`
  - `holoassist_unity_bridge/launch/unity_movement_bringup.launch.py`
- Added/updated documentation for Foxglove runtime, discovery, and perception validation.
- Synced Unity source-of-truth content from `origin/nic` into this branch:
  - `Unity/My project/Assets`
  - `Unity/My project/Packages`
  - `Unity/My project/ProjectSettings`
- Added `.gitattributes` for Unity mesh/artifact handling (`*.dae`, `*.apk` with LFS).
- Tightened `.gitignore` to exclude generated Unity and local dependency repo noise.

Recent push history on `origin/john`:
- `98e132d` feat: integrate manager and UR3 teleop packages from seb/ollie branches
- `e364f9e` feat: add foxglove-first observability package and runtime launch flow
- `43cde86` chore: ignore Unity generated build and local config artifacts
- `6980535` chore: sync Unity/XR workspace from nic and clean local artifact ignores

## Source-of-Truth Runtime Stack

Core packages for runtime bringup and observability:
- `ros2_ws/src/holoassist_foxglove`
- `ros2_ws/src/holoassist_manager`
- `ros2_ws/src/holoassist_unity_bridge`
- `ros2_ws/src/holo_assist_depth_tracker`
- `ros2_ws/src/holoassist_manipulation`
- `ros2_ws/src/holoassist_servo_tools`
- `ros2_ws/src/holoassist_movement`
- `ros2_ws/src/ur3_keyboard_teleop`
- `ros2_ws/src/ur3_joint_position_controller`

## Main Launch Entry Points

Build:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Foxglove observability only:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

Integrated runtime baseline:
```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

Unity bringup with observability enabled:
```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

Official Foxglove bridge only:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Test Commands

Perception only (no camera launch):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

Perception + RealSense camera:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Keyboard motion path:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

XR/Unity transport endpoint:
```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## Key Observability Topics

Aggregated runtime topics:
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

Manager topics:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

Perception topics:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

## No-Hardware Expectations

If no robot/camera/headset hardware is online:
- Runtime still starts and publishes Foxglove-facing observability topics.
- `/holoassist/diagnostics` will show WARN/ERROR for stale streams (expected).
- `/holoassist/state/teleop` and `/holoassist/state/planner` will usually show `IDLE`.
- `/holoassist/state/safety` may show `ERROR` when `/joint_states` is absent.
- Foxglove remains useful for bringup validation (topic graph, diagnostics transitions, event stream).

## Dashboard Naming Direction

Legacy retained path:
- `holoassist-dashboard` (compatibility/fallback)

Forward path:
- `holoassist_foxglove` package + Foxglove Studio layouts

Rule:
- New runtime status should be published to ROS topics first, then visualized in Foxglove.
- Avoid terminal-only operational state when the same signal can be topicized.

---

### Source: README.md

# HoloAssist

XR-assisted human-robot collaboration stack for ROS 2 Humble on Ubuntu 22.04.

## Current Status (2026-04-15)

This branch is now Foxglove-first for runtime observability.

Delivered in this branch:
- Unified observability package: `ros2_ws/src/holoassist_foxglove`
- Integrated manager diagnostics/mode supervision: `ros2_ws/src/holoassist_manager`
- Integrated keyboard motion path:
  - `ros2_ws/src/ur3_keyboard_teleop`
  - `ros2_ws/src/ur3_joint_position_controller`
  - `ros2_ws/src/holoassist_movement`
- Integrated Foxglove Bridge into runtime launch flows
- Updated perception + Foxglove docs and runbooks
- Synced Unity source-of-truth project folders from `origin/nic`

## Repository Layout

```text
john/
├── ros2_ws/
│   ├── src/
│   │   ├── holoassist_foxglove/
│   │   ├── holoassist_manager/
│   │   ├── holoassist_unity_bridge/
│   │   ├── holo_assist_depth_tracker/
│   │   ├── holoassist_manipulation/
│   │   ├── holoassist_servo_tools/
│   │   ├── holoassist_movement/
│   │   ├── ur3_keyboard_teleop/
│   │   └── ur3_joint_position_controller/
│   └── *.md runbooks
├── holoassist-dashboard/           # legacy dashboard path (compat/fallback)
├── dashboard/                      # nic branch desktop dashboard snapshot
├── Unity/My project/               # Unity source-of-truth (Assets/Packages/ProjectSettings)
└── claude.md
```

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Runtime Launches

Unified stack entrypoint (default profile = no-hardware-safe):
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py
```

Unified stack full hardware profile:
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py profile:=full_hardware
```

Unified stack with AprilTag tracking (for workbench calibration):
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  enable_apriltag_tracking:=true apriltag_start_camera:=false
```

Recommended integrated runtime:
```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

Observability-only:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

Unity bridge bringup (includes Foxglove observability by default):
```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

Official Foxglove Bridge only:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Test Commands

Perception pipeline:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Perception without camera (software-only smoke test):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

Keyboard motion path:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

Unity ROS TCP endpoint:
```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## What You Should See Without Hardware

Expected when no robot/camera/headset is connected:
- Runtime and Foxglove Bridge launch successfully.
- `/holoassist/diagnostics` is still published.
- Diagnostics show WARN/ERROR for stale missing streams (expected in offline mode).
- `/holoassist/events` logs transitions and missing stream changes.
- `/holoassist/state/teleop` and `/holoassist/state/planner` generally `IDLE`.
- `/holoassist/state/safety` may be `ERROR` when `/joint_states` is unavailable.

This is normal and useful for validating the observability layer before hardware is online.

## Foxglove Connection (Steam Deck / Browser)

1. Run runtime on Ubuntu host.
2. Confirm TCP 8765 is reachable from client network.
3. Open Foxglove Studio in browser on Steam Deck.
4. Connect to:

```text
ws://<host-ip>:8765
```

Layout guidance:
- `ros2_ws/src/holoassist_foxglove/config/foxglove_layout_spec.yaml`
- `ros2_ws/FOXGLOVE_RUNTIME.md`
- `ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md`

## Workspace Pipeline (Bench Plane + Tags)

Bench-understanding adapter node:
- `holo_assist_depth_tracker/workspace_perception_node.py`
- Fits bench plane from depth pointcloud
- Publishes `workspace_frame` TF
- Uses two AprilTag corner frames (when available) to stabilize in-plane orientation
- Publishes cropped + foreground clouds and final object pose/marker

### Plane-only bringup (today)

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_unity_bringup:=false
```

### Plane + AprilTag stabilization (2 corner tags)

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false \
  enable_unity_bringup:=false
```

### Validation topic checklist

```bash
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash

ros2 topic list | rg "^/holoassist/perception/(bench_plane|cropped|foreground|object|workspace)"
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
ros2 topic echo /holoassist/perception/bench_plane_coefficients --once
ros2 topic hz /holoassist/perception/cropped_pointcloud
ros2 topic hz /holoassist/perception/foreground_pointcloud
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```

## Key Runtime Topics

Core observability:
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`
- `/holoassist/metrics/*`

Perception:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

Manager:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Dashboard Migration Direction

Legacy UI remains for compatibility:
- `holoassist-dashboard`

Forward path:
- Foxglove Studio + `holoassist_foxglove` topic contracts

If a runtime status exists only in terminal output, publish it to a ROS topic so Foxglove can visualize and record it.

---

### Source: Unity/My project/Assets/Samples/XR Hands/1.7.1/HandVisualizer/README.md

# Hand Visualizer Sample

Demonstrates driving meshes and free-floating debug-draw objects on an XR Origin by using `XRHandSubsystem`.

---

### Source: aprilcubes/rg2_cube_1/README.md

# ArUco Cube — 1x1x1

![Cube preview](thumbnail.png)

## Parameters

| Parameter | Value |
|-----------|-------|
| Dictionary | `apriltag_36h11` |
| Grid | 1x1x1 (X x Y x Z tags) |
| Box dimensions | 56.25 x 56.25 x 56.25 mm |
| Tag size | 45 mm (8x8 cells) |
| Cell size | 5.625 mm |
| Margin | 1 cell (5.625 mm) |
| Border | 1 cell (5.625 mm) |
| Total tags | 6 |
| Tag IDs | 10–15 |

## Face Layout

| Face | Tag IDs |
|------|---------|
| +X | 10 |
| -X | 11 |
| +Y | 12 |
| -Y | 13 |
| +Z | 14 |
| -Z | 15 |

## Files

| File | Description |
|------|-------------|
| `cube.3mf` | Multi-color 3MF for Bambu Studio |
| `config.json` | Detector config (used by `detect_cube.py`) |
| `thumbnail.png` | 6-view preview |
| `mujoco/cube.xml` | MuJoCo MJCF model |
| `mujoco/cube.obj` | Wavefront OBJ mesh (UV-mapped) |
| `mujoco/cube.mtl` | OBJ material file |
| `mujoco/cube_atlas.png` | Texture atlas |

## Config JSON

```json
{
  "dict": "apriltag_36h11",
  "grid": "1x1x1",
  "tag_ids": [
    10,
    11,
    12,
    13,
    14,
    15
  ],
  "faces": {
    "+X": [
      10
    ],
    "-X": [
      11
    ],
    "+Y": [
      12
    ],
    "-Y": [
      13
    ],
    "+Z": [
      14
    ],
    "-Z": [
      15
    ]
  },
  "tag_size_mm": 45.0,
  "cell_size_mm": 5.625,
  "margin_cells": 1,
  "border_cells": 1,
  "marker_pixels": 8,
  "box_dims": [
    56.25,
    56.25,
    56.25
  ]
}
```

## Regenerate

```bash
python generate_cube.py --grid 1x1x1 --dict apriltag_36h11 --tag-size 45 --margin-cell 1 --border-cell 1 -o rg2_cube_60mm
```

---

### Source: aprilcubes/rg2_cube_2/README.md

# ArUco Cube — 1x1x1

![Cube preview](thumbnail.png)

## Parameters

| Parameter | Value |
|-----------|-------|
| Dictionary | `apriltag_36h11` |
| Grid | 1x1x1 (X x Y x Z tags) |
| Box dimensions | 56.25 x 56.25 x 56.25 mm |
| Tag size | 45 mm (8x8 cells) |
| Cell size | 5.625 mm |
| Margin | 1 cell (5.625 mm) |
| Border | 1 cell (5.625 mm) |
| Total tags | 6 |
| Tag IDs | 16–21 |

## Face Layout

| Face | Tag IDs |
|------|---------|
| +X | 16 |
| -X | 17 |
| +Y | 18 |
| -Y | 19 |
| +Z | 20 |
| -Z | 21 |

## Files

| File | Description |
|------|-------------|
| `cube.3mf` | Multi-color 3MF for Bambu Studio |
| `config.json` | Detector config (used by `detect_cube.py`) |
| `thumbnail.png` | 6-view preview |
| `mujoco/cube.xml` | MuJoCo MJCF model |
| `mujoco/cube.obj` | Wavefront OBJ mesh (UV-mapped) |
| `mujoco/cube.mtl` | OBJ material file |
| `mujoco/cube_atlas.png` | Texture atlas |

## Config JSON

```json
{
  "dict": "apriltag_36h11",
  "grid": "1x1x1",
  "tag_ids": [
    16,
    17,
    18,
    19,
    20,
    21
  ],
  "faces": {
    "+X": [
      16
    ],
    "-X": [
      17
    ],
    "+Y": [
      18
    ],
    "-Y": [
      19
    ],
    "+Z": [
      20
    ],
    "-Z": [
      21
    ]
  },
  "tag_size_mm": 45.0,
  "cell_size_mm": 5.625,
  "margin_cells": 1,
  "border_cells": 1,
  "marker_pixels": 8,
  "box_dims": [
    56.25,
    56.25,
    56.25
  ]
}
```

## Regenerate

```bash
python generate_cube.py --grid 1x1x1 --dict apriltag_36h11 --tag-size 45 --margin-cell 1 --border-cell 1 -o rg2_cube_2
```

---

### Source: aprilcubes/rg2_cube_3/README.md

# ArUco Cube — 1x1x1

![Cube preview](thumbnail.png)

## Parameters

| Parameter | Value |
|-----------|-------|
| Dictionary | `apriltag_36h11` |
| Grid | 1x1x1 (X x Y x Z tags) |
| Box dimensions | 56.25 x 56.25 x 56.25 mm |
| Tag size | 45 mm (8x8 cells) |
| Cell size | 5.625 mm |
| Margin | 1 cell (5.625 mm) |
| Border | 1 cell (5.625 mm) |
| Total tags | 6 |
| Tag IDs | 22–27 |

## Face Layout

| Face | Tag IDs |
|------|---------|
| +X | 22 |
| -X | 23 |
| +Y | 24 |
| -Y | 25 |
| +Z | 26 |
| -Z | 27 |

## Files

| File | Description |
|------|-------------|
| `cube.3mf` | Multi-color 3MF for Bambu Studio |
| `config.json` | Detector config (used by `detect_cube.py`) |
| `thumbnail.png` | 6-view preview |
| `mujoco/cube.xml` | MuJoCo MJCF model |
| `mujoco/cube.obj` | Wavefront OBJ mesh (UV-mapped) |
| `mujoco/cube.mtl` | OBJ material file |
| `mujoco/cube_atlas.png` | Texture atlas |

## Config JSON

```json
{
  "dict": "apriltag_36h11",
  "grid": "1x1x1",
  "tag_ids": [
    22,
    23,
    24,
    25,
    26,
    27
  ],
  "faces": {
    "+X": [
      22
    ],
    "-X": [
      23
    ],
    "+Y": [
      24
    ],
    "-Y": [
      25
    ],
    "+Z": [
      26
    ],
    "-Z": [
      27
    ]
  },
  "tag_size_mm": 45.0,
  "cell_size_mm": 5.625,
  "margin_cells": 1,
  "border_cells": 1,
  "marker_pixels": 8,
  "box_dims": [
    56.25,
    56.25,
    56.25
  ]
}
```

## Regenerate

```bash
python generate_cube.py --grid 1x1x1 --dict apriltag_36h11 --tag-size 45 --margin-cell 1 --border-cell 1 -o rg2_cube_3
```

---

### Source: aprilcubes/rg2_cube_4/README.md

# ArUco Cube — 1x1x1

![Cube preview](thumbnail.png)

## Parameters

| Parameter | Value |
|-----------|-------|
| Dictionary | `apriltag_36h11` |
| Grid | 1x1x1 (X x Y x Z tags) |
| Box dimensions | 56.25 x 56.25 x 56.25 mm |
| Tag size | 45 mm (8x8 cells) |
| Cell size | 5.625 mm |
| Margin | 1 cell (5.625 mm) |
| Border | 1 cell (5.625 mm) |
| Total tags | 6 |
| Tag IDs | 28–33 |

## Face Layout

| Face | Tag IDs |
|------|---------|
| +X | 28 |
| -X | 29 |
| +Y | 30 |
| -Y | 31 |
| +Z | 32 |
| -Z | 33 |

## Files

| File | Description |
|------|-------------|
| `cube.3mf` | Multi-color 3MF for Bambu Studio |
| `config.json` | Detector config (used by `detect_cube.py`) |
| `thumbnail.png` | 6-view preview |
| `mujoco/cube.xml` | MuJoCo MJCF model |
| `mujoco/cube.obj` | Wavefront OBJ mesh (UV-mapped) |
| `mujoco/cube.mtl` | OBJ material file |
| `mujoco/cube_atlas.png` | Texture atlas |

## Config JSON

```json
{
  "dict": "apriltag_36h11",
  "grid": "1x1x1",
  "tag_ids": [
    28,
    29,
    30,
    31,
    32,
    33
  ],
  "faces": {
    "+X": [
      28
    ],
    "-X": [
      29
    ],
    "+Y": [
      30
    ],
    "-Y": [
      31
    ],
    "+Z": [
      32
    ],
    "-Z": [
      33
    ]
  },
  "tag_size_mm": 45.0,
  "cell_size_mm": 5.625,
  "margin_cells": 1,
  "border_cells": 1,
  "marker_pixels": 8,
  "box_dims": [
    56.25,
    56.25,
    56.25
  ]
}
```

## Regenerate

```bash
python generate_cube.py --grid 1x1x1 --dict apriltag_36h11 --tag-size 45 --margin-cell 1 --border-cell 1 -o rg2_cube_4
```

---

### Source: holoassist-dashboard/README.md

# RS2 HoloAssist Dashboard (Legacy Compatibility Path)

This React dashboard is retained for compatibility, fallback debugging, and UI prototyping.

Primary runtime observability is now Foxglove-first.

## Runtime Positioning

Current preferred path:
- `ros2_ws/src/holoassist_foxglove`
- `foxglove_bridge` for live transport

Legacy/fallback path (this dashboard):
- Pulls perception and robot status from the dashboard relay HTTP API.
- Useful when Foxglove is unavailable or for quick local checks.

## Supported Relay Endpoints

Expected relay API routes:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

Relay launch options live in:
- `ros2_ws/src/holo_assist_depth_tracker/launch/dashboard_bridge.launch.py`
- `ros2_ws/src/holo_assist_depth_tracker/launch/foxglove_relay.launch.py`

## How It Relates to Foxglove

This dashboard is not the source of truth for runtime observability anymore.

When adding new runtime telemetry:
1. Publish ROS topics first.
2. Wire Foxglove panels to those topics.
3. Optionally expose reduced HTTP summaries for this dashboard.

## Run (Local)

Requires Node.js 18+.

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm install
npm run dev
```

Open `http://localhost:5173`.

If relay runs on same machine, Vite proxies `/api/*` to `http://127.0.0.1:8765`.

Optional override:

```bash
VITE_PERCEPTION_API_BASE=http://<relay-host>:8765 npm run dev
```

Build preview:

```bash
npm run build
npm run preview
```

## Foxglove-Equivalent Topics (Reference)

Most dashboard widgets map to topics already available in Foxglove runtime:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`
- `/joint_states`
- `/servo_target_pose`
- `/servo_node/delta_twist_cmds`
- `/holoassist/diagnostics`
- `/holoassist/events`

## Recommendation

Use this dashboard as fallback only.

For operations, demos, validation, and recordings, use Foxglove Studio connected through `foxglove_bridge`.

---

### Source: ros2_ws/FOXGLOVE_DISCOVERY_REPORT.md

# Foxglove Integration Discovery Report

Date: 2026-04-15
Workspace root: `~/git/RS2-HoloAssist`

## Discovery Scope

Goal of this pass:
- identify relevant repos/branches/packages
- map runtime interfaces (launches, nodes, topics)
- resolve overlap across dashboard/teleop/manager/unity work
- produce a Foxglove-first source-of-truth path in `john`

## Repositories Found

Top-level repos in workspace:
- `john` (active integration branch)
- `nic`
- `seb`
- `ollie`

Related nested Unity repos discovered:
- `ROS-TCP-Connector` (nested git repo)
- `URDF-Importer` (nested git repo)

No git submodules detected in `john`.

## Branch Ownership (Practical)

`origin/john` owned:
- core perception/manipulation/servo/unity bridge packages
- prior web dashboard path (`holoassist-dashboard`)

`origin/seb` owned:
- `holoassist_manager` (mode + heartbeat diagnostics)
- manager-focused runtime supervision

`origin/ollie` owned:
- `holoassist_movement`
- `ur3_keyboard_teleop`
- `ur3_joint_position_controller`

`origin/nic` owned:
- Unity-heavy XR project updates
- desktop dashboard scripts (`dashboard/`, `launch.py`, `launch.sh`, `dashboard.sh`)

## Overlaps and Fragmentation Found

Main overlap areas:
- Multiple dashboard paradigms: React (`holoassist-dashboard`), PyQt (`dashboard/`), HTTP relay (`dashboard_relay_node`), and now Foxglove.
- Runtime status previously split between terminal logs and package-specific topics.
- Perception evidence mostly exposed through custom HTTP relay rather than unified ROS observability topics.

Main fragmentation issues resolved:
- Added a single runtime aggregation node (`runtime_observability_node`) under `holoassist_foxglove`.
- Added consistent `/holoassist/*` topic namespace for diagnostics, events, state, and metrics.
- Wired `foxglove_bridge` into launch flow.

## Final Source-of-Truth Decision

Chosen source of truth:
- Base runtime in `john`
- Import/merge manager + motion/teleop from `seb` and `ollie`
- Introduce new Foxglove-focused package (`holoassist_foxglove`)
- Keep legacy dashboards as compatibility/fallback only

## Integration Commits on `john`

Key delivered commits:
- `98e132d` integrate manager and UR3 teleop packages from seb/ollie branches
- `e364f9e` add foxglove-first observability package and runtime launch flow
- `43cde86` tighten ignore rules for Unity-generated artifacts
- `6980535` sync Unity/XR source tree from `origin/nic` and clean ignore noise

## Unity Sync Outcome

`john` now includes `origin/nic` source-of-truth Unity folders:
- `Unity/My project/Assets`
- `Unity/My project/Packages`
- `Unity/My project/ProjectSettings`

Tracked build junk from `nic` was intentionally excluded via `.gitignore` cleanup, so `john` remains cleaner while retaining usable source content.

## Open Follow-Ups

- Convert any remaining terminal-only runtime signals to ROS topics.
- Add a concrete exported Foxglove `.json` layout file (current repo has panel spec YAML).
- Standardize launcher scripts that were synced from `nic` but still use absolute `nic` paths.

---

### Source: ros2_ws/FOXGLOVE_RUNTIME.md

# HoloAssist Foxglove Runtime Guide

Last updated: 2026-04-15
Primary branch: `john`

For the full operator runbook (dependencies, calibration, verification, troubleshooting, parity mapping), use:
- `ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md`

## Runtime Model

Foxglove is the default runtime observability layer.

Transport:
- `foxglove_bridge` (WebSocket)

Runtime aggregation:
- `holoassist_foxglove/runtime_observability_node`

Manager supervision:
- `holoassist_manager`

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Paths

### Unified stack entrypoint (recommended)

Default no-hardware-safe profile:

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py
```

Full-hardware profile:

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py profile:=full_hardware
```

No-hardware profile with depth + AprilTag calibration path:

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false
```

No-hardware profile with workspace understanding (bench plane + foreground object localization):

```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_unity_bringup:=false
```

### Observability-only

```bash
ros2 launch holoassist_foxglove observability.launch.py
```

### Integrated runtime baseline (recommended)

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

### Integrated runtime with optional stacks

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_pointcloud_obstacle:=true \
  enable_ur3_keyboard_teleop:=true \
  enable_ur3_joint_controller:=true \
  enable_robot_demo:=false
```

### Unity bringup path (still valid)

```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

### Bridge only

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Subsystem Online Tests

### Perception

RealSense + tracker + RViz:
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=true start_tracker:=true start_rviz:=true
```

Tracker only (no hardware camera):
```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py \
  start_camera:=false start_tracker:=true start_rviz:=false
```

### Motion (keyboard teleop stack)

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

### XR/Unity transport

```bash
ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py \
  ros_ip:=0.0.0.0 ros_tcp_port:=10000
```

## Simulation Bringup Paths (Merged)

### Fast software-only observability smoke test

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py \
  enable_unity_bringup:=false \
  enable_depth_tracker:=false \
  enable_pointcloud_obstacle:=false \
  enable_ur3_keyboard_teleop:=false \
  enable_ur3_joint_controller:=false
```

### UR fake hardware + keyboard motion

Terminal 1:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=0.0.0.0 \
  use_fake_hardware:=true fake_sensor_commands:=true launch_rviz:=false
```

Terminal 2:
```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate forward_velocity_controller
```

Terminal 3:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

Terminal 4:
```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

Terminal 5:
```bash
ros2 launch holoassist_foxglove observability.launch.py
```

### URSim + real UR driver path (optional)

If URSim External Control is running and reachable:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=<URSIM_IP> launch_rviz:=false
```

Then ensure:
- External Control program is started in URSim.
- Controller/output selection matches your motion path.

## What to Expect With No Hardware Plugged In

Expected behavior in offline mode:
- Launch succeeds and observability topics are published.
- Foxglove connection works.
- `/holoassist/diagnostics` reports stale streams (WARN/ERROR) for missing sensors/robot.
- `/holoassist/state/teleop` and `/holoassist/state/planner` generally stay `IDLE`.
- `/holoassist/state/safety` may become `ERROR` if `/joint_states` is missing.
- `/holoassist/events` still logs diagnostics state transitions.

This is normal and useful for validating runtime wiring before hardware availability.

## Core Topics for Foxglove Panels

### Aggregate observability
- `/holoassist/diagnostics`
- `/holoassist/events`
- `/holoassist/state/teleop`
- `/holoassist/state/planner`
- `/holoassist/state/safety`
- `/holoassist/state/runtime`

### Metrics
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

### Perception and scene
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`
- `/clicked_goal_marker`
- `/tf`

### Manager
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`

## Steam Deck / Browser Connection

1. Run runtime on Ubuntu host.
2. Ensure port `8765` reachable from Steam Deck network.
3. Open Foxglove Studio in browser.
4. Connect WebSocket:

```text
ws://<host-ip>:8765
```

Panel mapping baseline:
- `ros2_ws/src/holoassist_foxglove/config/foxglove_layout_spec.yaml`

## Legacy Dashboard Position

`holoassist-dashboard` remains available as fallback.

Forward direction:
- ROS topics + Foxglove panels are the source of truth.
- Legacy dashboards should consume topic-derived summaries, not define new runtime contracts.

---

### Source: ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md

# HoloAssist Runtime Unification Runbook (Ubuntu 22.04 / ROS 2 Humble)

Last updated: 2026-04-16  
Branch: `john`

This is the operator runbook for HoloAssist runtime bringup, Foxglove-first observability, perception validation, Unity bridge, and object localization debugging.

## 1) Objectives and Current System Model

Primary runtime goals:
- Keep Foxglove as the primary operations and debugging UI.
- Provide one single runtime launch entrypoint for no-hardware and full-hardware modes.
- Preserve existing subsystem interfaces and use incremental adapters.
- Track depth-derived objects in camera/workspace frames while maintaining human/blob safety gating.

Current integrated package set:
- `holoassist_foxglove`
- `holoassist_manager`
- `holoassist_unity_bridge`
- `holo_assist_depth_tracker`
- `holoassist_manipulation`
- `holoassist_servo_tools`
- `holoassist_movement`
- `ur3_keyboard_teleop`
- `ur3_joint_position_controller`

## 2) Dependencies (Exact Commands)

### 2.1 Apt prerequisites

```bash
sudo apt update
sudo apt install -y \
  git curl \
  python3-rosdep python3-colcon-common-extensions python3-vcstool \
  python3-numpy python3-opencv python3-pyqt5 \
  ros-humble-foxglove-bridge \
  ros-humble-realsense2-camera \
  ros-humble-cv-bridge \
  ros-humble-rviz2 \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ur-robot-driver \
  ros-humble-ur-dashboard-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-controller-manager-msgs \
  ros-humble-control-msgs \
  ros-humble-joint-trajectory-controller \
  ros-humble-trajectory-msgs
```

### 2.2 Unity ROS-TCP endpoint (source install in workspace)

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
if [ ! -d ROS-TCP-Endpoint ]; then
  git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
else
  cd ROS-TCP-Endpoint
  git fetch
  git checkout main-ros2
  git pull --ff-only
fi
```

### 2.3 rosdep

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd ~/git/RS2-HoloAssist/john/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys "ament_python"
```

### 2.4 Optional legacy React UI

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm ci
```

## 3) Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4) Launch Matrix

| Mode | Command |
|---|---|
| Unified stack (default no hardware) | `ros2 launch holoassist_foxglove holoassist_stack.launch.py` |
| Unified stack full hardware profile | `ros2 launch holoassist_foxglove holoassist_stack.launch.py profile:=full_hardware` |
| AprilTag workbench (standalone) | `ros2 launch holoassist_foxglove apriltag_workbench.launch.py start_camera:=true` |
| Unified stack + AprilTag tracking | `ros2 launch holoassist_foxglove holoassist_stack.launch.py enable_apriltag_tracking:=true apriltag_start_camera:=false` |
| Observability only | `ros2 launch holoassist_foxglove observability.launch.py` |
| Integrated runtime (existing) | `ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py` |
| Integrated runtime no-hardware-safe | `ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py enable_unity_bringup:=false enable_depth_tracker:=false enable_pointcloud_obstacle:=false enable_ur3_keyboard_teleop:=false enable_ur3_joint_controller:=false` |
| Perception only (RealSense + tracker + RViz) | `ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py start_camera:=true start_tracker:=true start_rviz:=true` |
| Perception only (no RealSense) | `ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py start_camera:=false start_tracker:=true start_rviz:=false` |
| Pointcloud -> MoveIt obstacle | `ros2 launch holoassist_manipulation pointcloud_to_moveit_obstacle.launch.py` |
| Keyboard motion path | `ros2 run ur3_joint_position_controller ur3_joint_position_controller` and `ros2 run ur3_keyboard_teleop keyboard_joint_teleop` |
| Unity TCP endpoint only | `ros2 launch holoassist_unity_bridge tcp_endpoint.launch.py ros_ip:=0.0.0.0 ros_tcp_port:=10000` |
| Unity movement bringup path | `ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py` |
| Foxglove bridge only | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` |

## 5) No-Hardware Expected Behavior

| Signal | Expected | Why |
|---|---|---|
| `/holoassist/diagnostics` | Continuous publish | observability node runs regardless of hardware |
| `holoassist/perception` | `WARN` then `ERROR` | pointcloud/debug stream stale |
| `holoassist/teleop` | `WARN` | target/clicked/twist idle |
| `holoassist/planning` | `WARN` | clicked/planner marker idle |
| `holoassist/safety` | `ERROR` | `/joint_states` stale or absent |
| `holoassist/network` | `WARN` possible | Unity endpoint may be unreachable |
| `/holoassist/state/teleop` | `IDLE` | no active teleop commands |
| `/holoassist/state/planner` | `IDLE` | no active planner marker stream |
| `/holoassist/state/safety` | `ERROR` or `WARN` | safety inputs missing |
| `/holoassist/events` | active transitions | expected diagnostic state change logging |

## 6) Topic Verification Checklist

```bash
# 1) Namespace presence
ros2 topic list | sort | rg "^/(holoassist|holo_assist_depth_tracker|holoassist_manager|ur3_keyboard|unity|tf)"

# 2) Core observability snapshots
ros2 topic echo /holoassist/diagnostics --once
ros2 topic echo /holoassist/events --once
ros2 topic echo /holoassist/state/teleop --once
ros2 topic echo /holoassist/state/planner --once
ros2 topic echo /holoassist/state/safety --once
ros2 topic echo /holoassist/state/runtime --once

# 3) Rate checks
ros2 topic hz /holoassist/diagnostics
ros2 topic hz /holo_assist_depth_tracker/debug_image
ros2 topic hz /holo_assist_depth_tracker/pointcloud
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker

# 4) Perception payload sanity
ros2 topic echo /holo_assist_depth_tracker/bbox --once
ros2 topic echo /holo_assist_depth_tracker/pointcloud --once

# 5) Manager
ros2 topic echo /holoassist_manager/mode --once
ros2 topic echo /holoassist_manager/diagnostics --once

# 6) Object localization adapter topics
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
ros2 topic echo /holoassist/perception/object_marker --once
```

## 7) UI Setup

### 7.1 Foxglove (primary)
- Start runtime with bridge enabled.
- Connect Foxglove Studio to `ws://<host-ip>:8765`.
- Recommended panels:
  - Image: `/holo_assist_depth_tracker/debug_image`
  - 3D: `/tf`, `/holo_assist_depth_tracker/pointcloud`, `/holo_assist_depth_tracker/obstacle_marker`, `/holoassist/perception/object_marker`, `/clicked_goal_marker`
  - Diagnostics: `/holoassist/diagnostics`
  - Log: `/holoassist/events`
  - Plot: `/holoassist/metrics/*`

### 7.2 React dashboard (legacy fallback)

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm ci
npm run dev
```

Uses relay endpoints:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

### 7.3 PyQt dashboard (legacy fallback)

```bash
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash
python3 ~/git/RS2-HoloAssist/john/dashboard/main.py
```

## 8) Troubleshooting

### 8.1 Missing `foxglove_bridge`

```bash
sudo apt install -y ros-humble-foxglove-bridge
ros2 pkg list | rg "^foxglove_bridge$"
```

### 8.2 Missing `realsense2_camera` or RealSense stream issues

```bash
sudo apt install -y ros-humble-realsense2-camera
ros2 pkg list | rg "^realsense2_camera$"
ros2 launch realsense2_camera rs_launch.py --show-args
```

If unstable (USB bandwidth/transport):
- reduce profile in launch: `depth_profile:=640,480,15`

### 8.3 Missing `ros_tcp_endpoint`

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | rg "^ros_tcp_endpoint$"
```

### 8.4 Hardcoded `/home/nic` paths
Already patched in this branch:
- `launch.py`
- `launch.sh`
- `dashboard.sh`
- `Unity/My project/Packages/manifest.json`
- `Unity/My project/Packages/packages-lock.json`

### 8.5 Stale diagnostics when hardware is absent
- Expected in offline bringup.
- Treat as failure only if `/holoassist/diagnostics` is missing entirely.

## 9) Team Quickstart (Copy/Paste)

```bash
# Terminal 1: build + source
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Terminal 1: runtime (camera available, robot absent)
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true

# Terminal 2: verify observability + perception
source /opt/ros/humble/setup.bash
source ~/git/RS2-HoloAssist/john/ros2_ws/install/setup.bash
ros2 topic echo /holoassist/diagnostics --once
ros2 topic hz /holo_assist_depth_tracker/pointcloud
ros2 topic hz /holo_assist_depth_tracker/obstacle_marker
ros2 topic echo /holoassist/perception/object_pose_workspace --once

# Foxglove: connect ws://<host-ip>:8765
```

## 10) Feature-Parity Mapping (Legacy UI -> Foxglove)

| Legacy runtime feature | Foxglove panel/topic | Parity | Adapter needed |
|---|---|---|---|
| Runtime health badges | `/holoassist/diagnostics` | Yes | No |
| Event stream | `/holoassist/events` | Yes | No |
| Joint monitor | `/joint_states`, `/holoassist/metrics/joint_states_hz` | Mostly | Optional normalized summary |
| Camera debug feed | `/holo_assist_depth_tracker/debug_image` | Yes | No |
| Headset feed | `/headset/image_compressed` | Yes (if source present) | No |
| Pointcloud + obstacle | `/holo_assist_depth_tracker/pointcloud`, `/holo_assist_depth_tracker/obstacle_marker` | Yes | No |
| Planner click marker | `/clicked_goal_marker` | Yes | No |
| Unity TCP reachability | `/holoassist/metrics/unity_tcp_latency_ms`, diagnostics network status | Yes | No |
| Object pose topicized for app consumers | `/holoassist/perception/object_pose*` + `/holoassist/perception/object_marker` | New in this branch | No |
| Controller active/inactive latch | N/A | Gap | Add `/holoassist/state/controller` adapter |
| Session mode/durations (`/session/status`) | N/A | Gap | Add `/holoassist/session/*` adapters |
| UI e-stop/resume semantics | N/A | Gap | command/service bridge topics/services |

## 11) Unified Stack Launch Entry Point

New entrypoint:
- `ros2 launch holoassist_foxglove holoassist_stack.launch.py`

Profiles:
- `profile:=no_hardware` (default): observability-first safe startup.
- `profile:=full_hardware`: enables Unity/robot/perception paths by default.

Override flags (all support `auto|true|false`):
- `enable_observability`
- `enable_foxglove_bridge`
- `enable_manager`
- `enable_tf_marker_bridge`
- `enable_object_pose_adapter`
- `enable_apriltag_tracking`
- `enable_depth_tracker`
- `enable_depth_camera`
- `enable_pointcloud_obstacle`
- `enable_unity_endpoint`
- `enable_unity_bringup`
- `enable_click_to_plan`
- `enable_ur3_keyboard_teleop`
- `enable_ur3_joint_controller`
- `enable_robot_demo`

Frame/network args:
- `ros_ip`, `ros_tcp_port`, `command_frame`, `eef_frame`, `object_workspace_frame`

AprilTag args:
- `apriltag_start_camera`
- `apriltag_image_topic`
- `apriltag_camera_info_topic`
- `apriltag_params_file`

## 12) Pointcloud + Object Localization Debug Workflow

### 12.1 Topic contract (standardized)
- `geometry_msgs/PoseStamped`: `/holoassist/perception/object_pose` (camera frame)
- `geometry_msgs/PoseStamped`: `/holoassist/perception/object_pose_workspace` (workspace frame)
- `visualization_msgs/Marker`: `/holoassist/perception/object_marker`

### 12.2 Camera/workspace frame checks

```bash
# Camera frame consistency
ros2 topic echo /holoassist/perception/object_pose --once

# Workspace transform consistency
ros2 run tf2_ros tf2_echo camera_depth_optical_frame base_link
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```

### 12.3 Human/blob safety gating
- Keep publishing:
  - `/holo_assist_depth_tracker/obstacle_marker`
  - `/holo_assist_depth_tracker/bbox`
- Keep `pointcloud_to_moveit_obstacle` active when planning scene gating is required.
- Policy: if blob/human obstacle marker active in workspace, planner must not place next trajectory through that volume.

### 12.4 Validation for your workflow
1. Object enters workspace and appears in pointcloud + object pose topics.
2. Object pose is visible in both RViz and Foxglove 3D.
3. Unity consumes same pose semantics via ROS bridge.
4. Teleop moves object to tray/bin; pose updates remain coherent.
5. Autonomous phase consumes object pose list / latest pose.
6. If human/blob enters scene, obstacle marker activates and planning obstacle remains enforced until scene clears.

## 13) AprilTag / ArUco / Hand-Eye Integration (Research + Recommended Path)

Your requirement (known workbench model + known robot base + known end-effector pose) maps directly to a staged calibration stack:

1. Camera intrinsics (already required for depth projection).
2. Camera extrinsics vs robot/workbench (`easy_handeye2`, eye-on-base).
3. Tag-based workspace anchoring (`apriltag_ros` or `aruco_ros`) for robust frame alignment and periodic recalibration checks.

### 13.1 Recommended practical architecture
- Start with one AprilTag on workbench for quick bringup.
- Move to a multi-tag board/grid (or multiple fixed tags) for robustness to partial occlusion and better pose stability.
- Publish a stable `workspace` frame from tag observations and calibrated camera extrinsics.
- Convert depth/object outputs to `workspace` for bin placement checks.

### 13.2 Package options (Humble)

```bash
sudo apt install -y ros-humble-apriltag ros-humble-apriltag-ros
sudo apt install -y ros-humble-aruco-ros ros-humble-aruco-msgs
apt-cache policy ros-humble-apriltag-ros ros-humble-aruco-ros
```

`easy_handeye2` (source install):

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -iyr --from-paths src --ignore-src
colcon build --symlink-install
```

### 13.3 Practical calibration execution (single tag -> multi-tag)

1. Print one `36h11` AprilTag with measured edge length (for example `0.035 m`) and mount it rigidly on the workbench.
2. Launch depth + AprilTag in one runtime:

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false
```

3. Verify detections and TF:

```bash
ros2 topic echo /detections --once
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
```

4. Run hand-eye calibration (`eye_on_base`) once tag tracking is stable:

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=realsense_eye_on_base \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=tag36h11:0
```

5. Publish saved calibration in normal runtime launches:

```bash
ros2 launch easy_handeye2 publish.launch.py name:=realsense_eye_on_base
```

6. Upgrade to multi-tag by editing:
- `ros2_ws/src/holoassist_foxglove/config/apriltag_workbench_36h11.yaml`
- Fill `tag.ids`, `tag.frames`, and `tag.sizes` so only known bench tags publish TF.

### 13.4 ArUco vs AprilTag (research-backed recommendation)
- `apriltag_ros` is the recommended default for workbench anchoring in this stack.
- `apriltag_ros` exposes per-tag filtering and per-tag frame/size settings (`tag.ids`, `tag.frames`, `tag.sizes`), which directly supports fixed multi-tag bench layouts.
- `aruco_ros` is still useful when board-based tracking is preferred. Its README explicitly lists enhanced precision using boards of markers.
- Keep one tag family per deployment to avoid decoder ambiguity and reduce false positives.

### 13.5 Research references
- `apriltag_ros` ROS 2 node README (topics/config, per-tag IDs/frames/sizes, TF publishing):
  - https://github.com/christianrauch/apriltag_ros
- Unity Robotics ROS-TCP endpoint setup for ROS 2 and `default_server_endpoint` usage:
  - https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md
  - https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2
- `easy_handeye2` ROS 2 hand-eye calibration (eye-on-base and eye-in-hand):
  - https://github.com/marcoesposito1988/easy_handeye2
- `aruco_ros` (single/board marker tracking, marker arrays, fiducial tracking):
  - https://github.com/pal-robotics/aruco_ros

### 13.6 Why this fits your stated workflow
- You can detect and track object ingress/pose in workspace using depth + CV.
- You can keep blob/human safety exclusion zones in planning via marker/pointcloud obstacle outputs.
- You can anchor both Unity and RViz to the same calibrated frames, reducing frame drift/misalignment during teleop/autonomy transitions.

## 14) Implementation Notes in This Branch

Implemented runtime changes:
- New unified launch entrypoint:
  - `ros2_ws/src/holoassist_foxglove/launch/holoassist_stack.launch.py`
- New optional AprilTag launch path:
  - `ros2_ws/src/holoassist_foxglove/launch/apriltag_workbench.launch.py`
  - `ros2_ws/src/holoassist_foxglove/config/apriltag_workbench_36h11.yaml`
- New object localization adapter node:
  - `holoassist_foxglove/obstacle_to_object_pose_adapter.py`
  - maps `/holo_assist_depth_tracker/obstacle_marker` to:
    - `/holoassist/perception/object_pose`
    - `/holoassist/perception/object_pose_workspace`
    - `/holoassist/perception/object_marker`
- Observability/runtime/unity launch wiring updated to include object pose adapter controls.
- Hardcoded `/home/nic` script/Unity path issues removed.

## 15) Workspace Understanding Pipeline (Bench Plane + Tags)

New node:
- `ros2_ws/src/holo_assist_depth_tracker/holo_assist_depth_tracker/workspace_perception_node.py`

New config:
- `ros2_ws/src/holo_assist_depth_tracker/config/workspace_perception_params.yaml`

New launch:
- `ros2_ws/src/holo_assist_depth_tracker/launch/workspace_perception.launch.py`

### 15.1 What it does
- Fits a loose bench plane from `/holo_assist_depth_tracker/pointcloud` with robust RANSAC + SVD refinement.
- Publishes bench plane coefficients and a bench plane marker.
- Publishes `workspace_frame` TF:
  - origin: inlier centroid projected onto bench plane.
  - `+Z`: plane normal pointing from bench toward camera origin.
  - `+X`: from tag0->tag1 projected onto plane when both tags are available; otherwise camera-x projected onto plane.
  - `+Y`: `cross(Z, X)`.
- Falls back to `plane_only` when tags are missing and reports `invalid` when plane fit fails.
- Crops points in workspace ROI and optional pedestal exclusion zone.
- Builds a simple voxel persistence background model and extracts foreground points.
- Clusters foreground and publishes best object estimate:
  - `/holoassist/perception/object_pose` (`geometry_msgs/PoseStamped`, camera frame)
  - `/holoassist/perception/object_marker` (`visualization_msgs/Marker`, camera frame)
  - `/holoassist/perception/object_pose_workspace` (`geometry_msgs/PoseStamped`, workspace frame)

### 15.2 Run commands (today)

Plane-only:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_unity_bringup:=false
```

Plane + AprilTags:
```bash
ros2 launch holoassist_foxglove holoassist_stack.launch.py \
  profile:=no_hardware \
  enable_depth_tracker:=true \
  enable_depth_camera:=true \
  enable_workspace_perception:=true \
  enable_object_pose_adapter:=false \
  enable_apriltag_tracking:=true \
  apriltag_start_camera:=false \
  enable_unity_bringup:=false
```

### 15.3 Debug/validation topics
- Raw depth cloud (unchanged): `/holo_assist_depth_tracker/pointcloud`
- Cropped ROI cloud: `/holoassist/perception/cropped_pointcloud`
- Foreground cloud: `/holoassist/perception/foreground_pointcloud`
- Plane marker: `/holoassist/perception/bench_plane_marker`
- Workspace axes marker: `/holoassist/perception/workspace_axes_marker`
- Tag markers: `/holoassist/perception/workspace_tag_markers`
- Plane coefficients: `/holoassist/perception/bench_plane_coefficients`
- Workspace mode + diagnostics:
  - `/holoassist/perception/workspace_mode`
  - `/holoassist/perception/workspace_diagnostics`
- Final object outputs:
  - `/holoassist/perception/object_pose`
  - `/holoassist/perception/object_marker`
  - `/holoassist/perception/object_pose_workspace`

Quick checks:
```bash
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
ros2 topic hz /holoassist/perception/cropped_pointcloud
ros2 topic hz /holoassist/perception/foreground_pointcloud
ros2 topic echo /holoassist/perception/object_pose --once
ros2 topic echo /holoassist/perception/object_pose_workspace --once
```

---

### Source: ros2_ws/PERCEPTION_PHASE4_VALIDATION.md

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

---

### Source: ros2_ws/src/ROS-TCP-Endpoint/CHANGELOG.md

# Changelog

All notable changes to this repository will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased

### Upgrade Notes

### Known Issues

### Added

Added Sonarqube scanner

### Changed

### Deprecated

### Removed

### Fixed


## [0.7.0] - 2022-02-01

### Added

Added Sonarqube scanner

Send information during hand shaking for ros and package version checks

Send service response as one queue item


## [0.6.0] - 2021-09-30

Add the [Close Stale Issues](https://github.com/marketplace/actions/close-stale-issues) action

### Upgrade Notes

### Known Issues

### Added

Support for queue_size and latch for publishers. (https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues/82)

### Changed

### Deprecated

### Removed

### Fixed

## [0.5.0] - 2021-07-15

### Upgrade Notes

Upgrade the ROS communication to support ROS2 with Unity

### Known Issues

### Added

### Changed

### Deprecated

### Removed

### Fixed

## [0.4.0] - 2021-05-27

Note: the logs only reflects the changes from version 0.3.0

### Upgrade Notes

RosConnection 2.0: maintain a single constant connection from Unity to the Endpoint. This is more efficient than opening one connection per message, and it eliminates a whole bunch of user issues caused by ROS being unable to connect to Unity due to firewalls, proxies, etc.

### Known Issues

### Added

Add a link to the Robotics forum, and add a config.yml to add a link in the Github Issues page

Add linter, unit tests, and test coverage reporting

### Changed

Improving the performance of the read_message in client.py, This is done by receiving the entire message all at once instead of reading 1024 byte chunks and stitching them together as you go.

### Deprecated

### Removed

Remove outdated handshake references

### Fixed

---

### Source: ros2_ws/src/ROS-TCP-Endpoint/CODE_OF_CONDUCT.md

# Contributor Covenant Code of Conduct

## Our Pledge

In the interest of fostering an open and welcoming environment, we as
contributors and maintainers pledge to making participation in our project and
our community a harassment-free experience for everyone, regardless of age, body
size, disability, ethnicity, gender identity and expression, level of experience,
nationality, personal appearance, race, religion, or sexual identity and
orientation.

## Our Standards

Examples of behavior that contributes to creating a positive environment
include:

* Using welcoming and inclusive language
* Being respectful of differing viewpoints and experiences
* Gracefully accepting constructive criticism
* Focusing on what is best for the community
* Showing empathy towards other community members

Examples of unacceptable behavior by participants include:

* The use of sexualized language or imagery and unwelcome sexual attention or
  advances
* Trolling, insulting/derogatory comments, and personal or political attacks
* Public or private harassment
* Publishing others' private information, such as a physical or electronic
  address, without explicit permission
* Other conduct which could reasonably be considered inappropriate in a
  professional setting

## Our Responsibilities

Project maintainers are responsible for clarifying the standards of acceptable
behavior and are expected to take appropriate and fair corrective action in
response to any instances of unacceptable behavior.

Project maintainers have the right and responsibility to remove, edit, or
reject comments, commits, code, wiki edits, issues, and other contributions
that are not aligned to this Code of Conduct, or to ban temporarily or
permanently any contributor for other behaviors that they deem inappropriate,
threatening, offensive, or harmful.

## Scope

This Code of Conduct applies both within project spaces and in public spaces
when an individual is representing the project or its community. Examples of
representing a project or community include using an official project e-mail
address, posting via an official social media account, or acting as an appointed
representative at an online or offline event. Representation of a project may be
further defined and clarified by project maintainers.

## Enforcement

Instances of abusive, harassing, or otherwise unacceptable behavior may be
reported by contacting the project team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com). All
complaints will be reviewed and investigated and will result in a response that
is deemed necessary and appropriate to the circumstances. The project team is
obligated to maintain confidentiality with regard to the reporter of an incident.
Further details of specific enforcement policies may be posted separately.

Project maintainers who do not follow or enforce the Code of Conduct in good
faith may face temporary or permanent repercussions as determined by other
members of the project's leadership.

## Attribution

This Code of Conduct is adapted from the [Contributor Covenant][homepage],
version 1.4, available at
https://www.contributor-covenant.org/version/1/4/code-of-conduct/

[homepage]: https://www.contributor-covenant.org
---

### Source: ros2_ws/src/ROS-TCP-Endpoint/CONTRIBUTING.md

# Contribution Guidelines

Thank you for your interest in contributing to Unity Robotics! To facilitate your
contributions, we've outlined a brief set of guidelines to ensure that your extensions
can be easily integrated.

## Communication

First, please read through our
[code of conduct](CODE_OF_CONDUCT.md),
as we expect all our contributors to follow it.

Second, before starting on a project that you intend to contribute to any of our
Unity Robotics packages or tutorials, we **strongly** recommend posting on the repository's
[Issues page](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues) and
briefly outlining the changes you plan to make. This will enable us to provide
some context that may be helpful for you. This could range from advice and
feedback on how to optimally perform your changes or reasons for not doing it.

## Git Branches

The `main` branch corresponds to the most recent stable version of the project. The `dev` branch
contains changes that are staged to be merged into `main` as the team sees fit.

When contributing to the project, please make sure that your Pull Request (PR)
does the following:

- Is up-to-date with and targets the `dev` branch
- Contains a detailed description of the changes performed
- Has corresponding changes to documentation, unit tests and sample environments (if
  applicable)
- Contains a summary of the tests performed to validate your changes
- Links to issue numbers that the PR resolves (if any)

<!-- ## Continuous Integration (CI)

We run continuous integration on all PRs; all tests must be passing before the PR is merged. -->

## Code style

All Python code should follow the [PEP 8 style guidelines](https://pep8.org/).

All C# code should follow the [Microsoft C# Coding Conventions](https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/inside-a-program/coding-conventions).
Additionally, the [Unity Coding package](https://docs.unity3d.com/Packages/com.unity.coding@0.1/manual/index.html)
can be used to format, encode, and lint your code according to the standard Unity
development conventions. Be aware that these Unity conventions will supersede the
Microsoft C# Coding Conventions where applicable.

Please note that even if the code you are changing does not adhere to these guidelines,
we expect your submissions to follow these conventions.

## Contributor License Agreements

When you open a pull request, you will be asked to acknowledge our Contributor
License Agreement. We allow both individual contributions and contributions made
on behalf of companies. We use an open source tool called CLA assistant. If you
have any questions on our CLA, please
[submit an issue](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues) or
email us at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## Contribution review

Once you have a change ready following the above ground rules, simply make a
pull request in GitHub.
---

### Source: ros2_ws/src/ROS-TCP-Endpoint/README.md

# ROS TCP Endpoint

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Introduction

[ROS](https://www.ros.org/) package used to create an endpoint to accept ROS messages sent from a Unity scene using the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) scripts.

Instructions and examples on how to use this ROS package can be found on the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/master/tutorials/ros_unity_integration/README.md) repository.

## Community and Feedback

The Unity Robotics projects are open-source and we encourage and welcome contributions.
If you wish to contribute, be sure to review our [contribution guidelines](CONTRIBUTING.md)
and [code of conduct](CODE_OF_CONDUCT.md).

## Support
For questions or discussions about Unity Robotics package installations or how to best set up and integrate your robotics projects, please create a new thread on the [Unity Robotics forum](https://forum.unity.com/forums/robotics.623/) and make sure to include as much detail as possible.

For feature requests, bugs, or other issues, please file a [GitHub issue](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues) using the provided templates and the Robotics team will investigate as soon as possible.

For any other questions or feedback, connect directly with the
Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## License
[Apache License 2.0](LICENSE)
---

### Source: ros2_ws/src/holo_assist_depth_tracker/.pytest_cache/README.md

# pytest cache directory #

This directory contains data from the pytest's cache plugin,
which provides the `--lf` and `--ff` options, as well as the `cache` fixture.

**Do not** commit this to version control.

See [the docs](https://docs.pytest.org/en/stable/cache.html) for more information.

---

### Source: ros2_ws/src/holo_assist_depth_tracker/README.md

# holo_assist_depth_tracker

Deterministic AprilTag-first perception stack for HoloAssist workspace tracking.

## 1) System Overview

This package now provides a deterministic dual-detector AprilTag pipeline:

- Board detector tracks workspace board tags `0..3` (size `0.15 m`) on `/detections_board`
- Cube detector tracks cube tags `10..33` (size `0.045 m`) on `/detections_cubes`
- Merge node outputs deduplicated union on `/detections_all`
- Workspace node locks `workspace_frame` from the first valid full 4-tag board solve
- Cube node publishes stable poses/TF/markers/status for 4 cubes
- Overlay and depth tracker use merged detections (`/detections_all`)

Legacy nodes/launch files are kept for backward compatibility.

## 2) Architecture Diagram

```text
RGB Image + CameraInfo
        |
        +---------------------------+
        |                           |
  apriltag_board               apriltag_cubes
(ids 0..3, 0.15m)           (ids 10..33, 0.045m)
        |                           |
 /detections_board           /detections_cubes
        +-----------+   +-----------+
                    |   |
            detection_merge_node
                    |
              /detections_all
               /      |      \
              /       |       \
 overlay_node   workspace_board_node   cube_pose_node
      |              |      \             |
 overlay image   workspace TF  robot TF   cube TF/pose/marker/status (x4)
                                 \
                             ur3e_base_link0
```

## 3) Node Descriptions

### `holoassist_detection_merge_node`
- Inputs: `/detections_board`, `/detections_cubes`
- Output: `/detections_all`
- Behavior:
  - Union by AprilTag ID
  - Per-ID latest timestamp wins
  - Removes duplicates
  - Prunes stale IDs by timeout

### `holoassist_workspace_board_node`
- Input: `/detections_board` + TF `tag36h11:0..3`
- Output:
  - TF `<camera_frame> -> workspace_frame`
  - TF `workspace_frame -> ur3e_base_link0` at `(0.450, 0.564, 0.0)`
  - `/holoassist/perception/ur3e_base_link0_pose`
  - `/holoassist/perception/ur3e_base_link0_marker`
  - `/holoassist/perception/workspace_mode`
  - `/holoassist/perception/workspace_diagnostics`
  - `/holoassist/perception/bench_plane_coefficients`
- Behavior:
  - Uses only board tags `0,1,2,3`
  - Enforces board geometry (0.700 x 0.500)
  - Locks on first valid full board solve
  - No drift updates after lock
  - `realign_workspace` service clears lock

### `holoassist_cube_pose_node`
- Input: `/detections_all` + TF `tag36h11:<id>`
- Cube groups:
  - Cube 1: `10..15`
  - Cube 2: `16..21`
  - Cube 3: `22..27`
  - Cube 4: `28..33`
- Output per cube `n=1..4`:
  - TF `workspace_frame -> apriltag_cube_<n>`
  - `/holoassist/perception/april_cube_<n>_pose`
  - `/holoassist/perception/april_cube_<n>_marker`
  - `/holoassist/perception/april_cube_<n>_status`
- Additional legacy alias:
  - `/holoassist/perception/april_cube_pose` (Cube 1)
- Behavior:
  - Sequential face-axis mapping per group: `+X,-X,+Y,-Y,+Z,-Z`
  - Robust to partial visibility
  - Stable orientation via sign continuity + multi-tag rotation fit
  - No obstacle-only fallback

### `holoassist_overlay_node`
- Input: RGB image + `/detections_all`
- Output: `/holoassist/perception/apriltag_overlay`
- Behavior:
  - Draws all merged detections
  - Footer text: `tags=<count> age=<seconds>`

### Existing `depth_tracker_node` (preserved)
- Updated defaults for AprilTag integration:
  - `apriltag_topic=/detections_all`
  - Tracking IDs default to `10..33` (cube tags only)

## 4) Topics

### Detector/Merge
- `/detections_board` (`apriltag_msgs/AprilTagDetectionArray`)
- `/detections_cubes` (`apriltag_msgs/AprilTagDetectionArray`)
- `/detections_all` (`apriltag_msgs/AprilTagDetectionArray`)

### Workspace/Robot
- `/holoassist/perception/workspace_mode` (`std_msgs/String`)
- `/holoassist/perception/workspace_diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- `/holoassist/perception/bench_plane_coefficients` (`std_msgs/Float32MultiArray`)
- `/holoassist/perception/ur3e_base_link0_pose` (`geometry_msgs/PoseStamped`)
- `/holoassist/perception/ur3e_base_link0_marker` (`visualization_msgs/Marker`)

### Cubes
- `/holoassist/perception/april_cube_1_pose` ... `_4_pose`
- `/holoassist/perception/april_cube_1_marker` ... `_4_marker`
- `/holoassist/perception/april_cube_1_status` ... `_4_status`
- Legacy alias: `/holoassist/perception/april_cube_pose`

### Overlay
- `/holoassist/perception/apriltag_overlay` (`sensor_msgs/Image`)

## 5) TF Tree

```text
<camera_frame>
├── tag36h11:0
├── tag36h11:1
├── tag36h11:2
├── tag36h11:3
├── tag36h11:10 ... tag36h11:33
└── workspace_frame
    ├── ur3e_base_link0
    ├── apriltag_cube_1
    ├── apriltag_cube_2
    ├── apriltag_cube_3
    └── apriltag_cube_4
```

## 6) Launch Instructions

### Primary new launch

```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

Useful arguments:
- `start_camera:=false|true`
- `image_topic:=/camera/camera/color/image_raw`
- `camera_info_topic:=/camera/camera/color/camera_info`
- `start_tracker:=true|false`
- `start_overlay:=true|false`
- `board_params_file:=.../apriltag_board.yaml`
- `cubes_params_file:=.../apriltag_cubes.yaml`
- `workspace_params_file:=.../workspace.yaml`
- `cube_pose_params_file:=.../cubes.yaml`

### Realign workspace lock

```bash
ros2 service call /holoassist/perception/realign_workspace std_srvs/srv/Trigger "{}"
```

### Backward-compatible launch files
Legacy launch files under `launch/` are preserved and still usable.

## 7) Parameter Highlights

### `config/apriltag_board.yaml`
- Board detector family/size/IDs (`0..3`, `0.15m`)

### `config/apriltag_cubes.yaml`
- Cube detector family/size/IDs (`10..33`, `0.045m`)

### `config/workspace.yaml`
- Workspace frame name and board tag IDs
- Lock/geometry tolerances
- Realign service name
- Fixed robot pose in workspace coordinates

### `config/cubes.yaml`
- Per-cube tag groups
- Cube size
- Timeout/TF lookup/timer rates
- Legacy alias topic

## 8) Testing Procedure

1. Build and source:
```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
colcon build --packages-select holo_assist_depth_tracker holoassist_foxglove
source install/setup.bash
```

2. Run launch:
```bash
ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py
```

3. Verify detection streams:
```bash
ros2 topic hz /detections_board
ros2 topic hz /detections_cubes
ros2 topic echo /detections_all --once
```

4. Verify workspace lock and diagnostics:
```bash
ros2 topic echo /holoassist/perception/workspace_mode --once
ros2 topic echo /holoassist/perception/workspace_diagnostics --once
```

5. Verify TF:
```bash
ros2 run tf2_ros tf2_echo workspace_frame ur3e_base_link0
ros2 run tf2_ros tf2_echo workspace_frame apriltag_cube_1
```

6. Verify overlay and cube outputs:
```bash
ros2 topic echo /holoassist/perception/april_cube_1_status --once
ros2 topic hz /holoassist/perception/apriltag_overlay
```

## 9) Failure Cases and Behavior

- Missing/stale board detections:
  - Workspace status publishes `INVALID`
  - No fallback to alternate geometry
- Insufficient board tags (`<4`):
  - Status `INVALID`
  - Existing lock (if any) is retained until realign
- Invalid board geometry (dimension/residual tolerance fail):
  - Status `INVALID`
  - No lock acquisition
- Cube tags partially visible:
  - Cube pose still solved from available tags where possible
- Cube has no valid tag transforms:
  - Cube status indicates failure and marker is deleted

## 10) Integration Notes

### RViz
- Set fixed frame to `workspace_frame` for board-relative visualization.
- Display TF, cube poses/markers, robot marker, and overlay image topic.

### Foxglove
- Subscribe to:
  - `/detections_all`
  - `/holoassist/perception/workspace_diagnostics`
  - `/holoassist/perception/april_cube_*_pose`
  - `/holoassist/perception/april_cube_*_status`
- Optional stack integration is available in `holoassist_stack.launch.py` via:
  - `enable_4tag_board_4cube_pipeline:=true`

### Unity
- Consume TF for:
  - `workspace_frame`
  - `ur3e_base_link0`
  - `apriltag_cube_1..4`
- The workspace frame is deterministic once locked; call realign service when board setup changes.

---

### Source: ros2_ws/src/holoassist_foxglove/README.md

# holoassist_foxglove

Foxglove-first runtime observability package for HoloAssist.

## What It Provides

- Runtime aggregation node: `runtime_observability_node`
- Object pose adapter node: `obstacle_to_object_pose_adapter`
- Unified diagnostics: `/holoassist/diagnostics`
- Event stream: `/holoassist/events`
- Runtime state summaries: `/holoassist/state/*`
- Metrics: `/holoassist/metrics/*`
- Heartbeat publishers compatible with `holoassist_manager`
- Launch integration with `foxglove_bridge`

## Key Metrics Topics

- `/holoassist/metrics/debug_image_hz`
- `/holoassist/metrics/pointcloud_hz`
- `/holoassist/metrics/bbox_hz`
- `/holoassist/metrics/pointcloud_points`
- `/holoassist/metrics/debug_image_age_s`
- `/holoassist/metrics/pointcloud_age_s`
- `/holoassist/metrics/perception_pipeline_ok` (`1.0` = fresh, `0.0` = stale)
- `/holoassist/metrics/joint_states_hz`
- `/holoassist/metrics/target_transport_latency_ms`
- `/holoassist/metrics/twist_transport_latency_ms`
- `/holoassist/metrics/unity_tcp_latency_ms`
- `/holoassist/metrics/foxglove_tcp_latency_ms`

## Launch Files

- `launch/observability.launch.py`
  - starts runtime observability node
  - optional `holoassist_manager`
  - optional `foxglove_bridge`

- `launch/holoassist_foxglove_runtime.launch.py`
  - integrated runtime convenience launch
  - optional perception/motion stacks
  - optional Unity bringup + observability

- `launch/apriltag_workbench.launch.py`
  - optional AprilTag tracker + optional RealSense launch for workbench calibration

## Quick Start

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch holoassist_foxglove observability.launch.py
```

Integrated runtime:

```bash
ros2 launch holoassist_foxglove holoassist_foxglove_runtime.launch.py
```

## Useful Launch Arguments

From `observability.launch.py`:
- `enable_foxglove_bridge` (default `true`)
- `enable_manager` (default `true`)
- `enable_tf_marker_bridge` (default `false`)
- `diagnostics_rate_hz` (default `1.0`)
- `stale_timeout_s` (default `2.5`)
- `unity_tcp_host` / `unity_tcp_port`
- `foxglove_bridge_host` / `foxglove_bridge_port`

## Expected Offline Behavior

Without robot/camera/headset online:
- package still publishes diagnostics/events/state topics
- diagnostics will report stale streams
- this is expected and useful for pipeline bringup validation

## Recommended Foxglove Layout Spec

- `config/foxglove_layout_spec.yaml`

---

### Source: ros2_ws/src/holoassist_manager/README.md

# holoassist_manager

Central runtime supervisor for HoloAssist mode + heartbeat diagnostics.

## Operating Modes

| Mode | Description |
|---|---|
| `MANUAL` | Human-in-the-loop teleoperation |
| `HYBRID` | Combined manual + autonomous behavior |

## Supervised Heartbeats

Expected heartbeat topics:
- `/xr_interface/heartbeat`
- `/perception/heartbeat`
- `/planning/heartbeat`
- `/control/heartbeat`
- `/rviz/heartbeat`
- `/unity_bridge/heartbeat`
- `/robot_bringup/heartbeat`

`holoassist_foxglove/runtime_observability_node` publishes compatible heartbeats.

## ROS Interface

Published:
- `~/mode` (`std_msgs/String`, latched)
- `~/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

Services:
- `~/set_manual` (`std_srvs/Trigger`)
- `~/set_hybrid` (`std_srvs/Trigger`)
- `~/get_mode` (`std_srvs/Trigger`)
- `~/system_status` (`std_srvs/Trigger`)

Common absolute names when node name is `holoassist_manager`:
- `/holoassist_manager/mode`
- `/holoassist_manager/diagnostics`
- `/holoassist_manager/set_manual`
- `/holoassist_manager/set_hybrid`
- `/holoassist_manager/get_mode`
- `/holoassist_manager/system_status`

Parameters:
- `initial_mode` (default `MANUAL`)
- `status_publish_rate` (default `1.0` Hz)
- `heartbeat_timeout_sec` (default `5.0` sec)

## Build and Run

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holoassist_manager --symlink-install
source install/setup.bash

ros2 launch holoassist_manager manager.launch.py
```

## Foxglove Integration

With observability launch enabled:
- manager diagnostics appear in Foxglove on `/holoassist_manager/diagnostics`
- manager mode appears on `/holoassist_manager/mode`

---

### Source: ros2_ws/src/ur3_joint_position_controller/README.md

# ur3_joint_position_controller

Keyboard-command bridge that converts selected-joint jog intent into UR3 joint trajectory commands.

## Topics

Subscribed:
- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/scaled_joint_trajectory_controller/state` (`control_msgs/msg/JointTrajectoryControllerState`)
- `/io_and_status_controller/robot_program_running` (`std_msgs/msg/Bool`)
- `/io_and_status_controller/robot_mode` (`ur_dashboard_msgs/msg/RobotMode`)
- `/io_and_status_controller/safety_mode` (`ur_dashboard_msgs/msg/SafetyMode`)

Published:
- `/scaled_joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/ur3_keyboard/robot_command_text` (`std_msgs/msg/String`)

## Behavior

- Uses current `/joint_states` as motion baseline.
- Validates robot/controller readiness before publishing.
- While key command is active, jogs selected joint continuously.
- On key release timeout, publishes hold-position command.
- Publishes human-readable command status on `/ur3_keyboard/robot_command_text`.

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_joint_position_controller --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

or

```bash
ros2 launch ur3_joint_position_controller ur3_joint_position_controller.launch.py
```

## Minimum Bringup

1. UR driver path running (real robot, URSim, or fake hardware).
2. `scaled_joint_trajectory_controller` active.
3. `ur3_joint_position_controller` running.
4. `ur3_keyboard_teleop` running.

## Fake Hardware Example

Terminal 1:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=0.0.0.0 \
  use_fake_hardware:=true fake_sensor_commands:=true launch_rviz:=false
```

Terminal 2:
```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate forward_velocity_controller
```

Terminal 3:
```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller
```

Terminal 4:
```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

## Foxglove Visibility

These keyboard-motion signals are consumed by observability stack and visible in Foxglove:
- `/ur3_keyboard/command_text`
- `/ur3_keyboard/robot_command_text`
- `/holoassist/events`
- `/holoassist/diagnostics`

## Useful Parameters

```bash
ros2 run ur3_joint_position_controller ur3_joint_position_controller --ros-args \
  -p jog_speed_rad_per_sec:=0.25 \
  -p control_rate_hz:=40.0 \
  -p hold_timeout_sec:=0.10
```

---

### Source: ros2_ws/src/ur3_keyboard_teleop/README.md

# ur3_keyboard_teleop

Terminal keyboard teleop for selecting UR3 joints and publishing jog direction commands.

## Keys

- `1` to `6`: select active UR3 joint
- `a`: move selected joint anticlockwise
- `d`: move selected joint clockwise
- `s`: stop selected joint
- `q`: quit node

Holding `a`/`d` keeps publishing until key repeat stops.

## Published Topics

- `/ur3_keyboard/selected_joint` (`std_msgs/msg/Int32`)
- `/ur3_keyboard/joint_direction` (`std_msgs/msg/Int8`)
- `/ur3_keyboard/command_text` (`std_msgs/msg/String`)

## Build

```bash
cd ~/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_keyboard_teleop --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop
```

or

```bash
ros2 launch ur3_keyboard_teleop keyboard_joint_teleop.launch.py
```

## Parameters

- `publish_rate_hz` (default `30.0`)
- `hold_timeout_sec` (default `0.08`)

Example:

```bash
ros2 run ur3_keyboard_teleop keyboard_joint_teleop --ros-args \
  -p publish_rate_hz:=30.0 \
  -p hold_timeout_sec:=0.15
```

## Downstream Usage

Typical consumer:
- `ur3_joint_position_controller` subscribes and converts commands into trajectory points.

Foxglove runtime visibility:
- `/ur3_keyboard/command_text` is surfaced via observability event stream and teleop diagnostics.

---

