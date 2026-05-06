# HoloAssist ROS 2 Workspace — Full Engineering Notes

Everything we have built, tried, broken, fixed, and planned. Written as a living
reference so any team member can pick up where we left off without re-deriving
decisions from first principles.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Package Map](#3-package-map)
4. [TF Tree Design — The Central Problem](#4-tf-tree-design--the-central-problem)
5. [Perception Pipeline](#5-perception-pipeline)
6. [Simulation Stack](#6-simulation-stack)
7. [Hardware Stack](#7-hardware-stack)
8. [Hybrid Mode — Fake Robot + Real Camera](#8-hybrid-mode--fake-robot--real-camera)
9. [Camera Self-Calibration — What We Tried](#9-camera-self-calibration--what-we-tried)
10. [Pick-and-Place Pipeline](#10-pick-and-place-pipeline)
11. [MoveIt Integration](#11-moveit-integration)
12. [RViz Configuration](#12-rviz-configuration)
13. [Launch File Reference](#13-launch-file-reference)
14. [Diagnostic Cheatsheet](#14-diagnostic-cheatsheet)
15. [Problems We Hit and How We Solved Them](#15-problems-we-hit-and-how-we-solved-them)
16. [What Still Needs Doing](#16-what-still-needs-doing)
17. [Build Reference](#17-build-reference)

---

## 1. Project Overview

HoloAssist is a ROS 2 Humble system on Ubuntu 22.04 for a UR3e collaborative arm
that sorts physical objects into bins. The robot operates in two modes:

- **Teleoperation** — a human in a Meta Quest 3 headset controls the arm in mixed
  reality via RMRC, Direct Joint, or Hand Guide control.
- **Autonomous** — the robot plans and executes pick-and-place via MoveIt 2 without
  human input.

Physical objects are mapped to different virtual objects in the XR scene to create a
gamified operator experience (e.g. a tomato appears as a bomb that must be "defused"
into the correct bin).

**Hardware:**
- UR3e collaborative arm
- OnRobot RG2 gripper (Modbus over RS-485 via tool serial port)
- Intel RealSense D435i depth/colour camera (mounted above workspace looking down)
- 4-tag AprilTag board defining the workspace coordinate frame
- 4 physical cubes each tagged with 6 AprilTag stickers (IDs 10–33)
- Meta Quest 3 headset (XR interface)
- Steam Deck OLED (dashboard display)
- Dedicated UR router (192.168.0.x subnet)

**Software stack:**
- ROS 2 Humble
- MoveIt 2 (OMPL, Cartesian planning)
- apriltag_ros (AprilTag detection)
- Intel RealSense ROS 2 driver (realsense2_camera)
- ur_onrobot driver (combined UR + RG2 bringup, C++ Modbus)
- Unity 6.3 LTS + Meta XR SDK v85 (Quest app)
- rosbridge_server (WebSocket bridge for Unity)
- PyQt5 dashboard

---

## 2. System Architecture

```
Intel RealSense D435i
  │  /camera/camera/color/image_raw
  │  /camera/camera/color/camera_info
  ▼
apriltag_ros (apriltag_node)
  │  /detections_all
  ├──► workspace_board_node ──► workspace_frame TF (dynamic)
  └──► cube_pose_node ──────► /holoassist/perception/april_cube_{1-4}_pose
                                          │
                            ┌─────────────┘
                            ▼
              sim_cube_moveit_bridge_node
                  │  /planning_scene (collision objects)
                  │  /holoassist/teleop/selected_cube_pose
                  ▼
         selected_cube_to_moveit_target_node
                  │  /moveit_robot_control/target_point
                  │  /moveit_robot_control/target_pose
                  ▼
           coordinate_listener (MoveIt)
                  │  /joint_trajectory_controller/joint_trajectory
                  ▼
           ros2_control (fake or real)
                  │
         ┌────────┴────────┐
         ▼                 ▼
    Fake robot          Real UR3e
    (sim/hybrid)        (hardware)

                ┌──────────────────┐
                │  pick_place_     │
                │  service_node    │◄── /holoassist/pick_cube_to_bin (srv)
                └────────┬─────────┘
                         │ /pick_place/command
                         ▼
                pick_place_sequencer
                         │ target_pose / target_point
                         ▼
                coordinate_listener
```

**Teleoperation path (Nic's subsystem):**
```
Quest 3 controllers
  │  XR Input System
  ▼
RobotController.cs (Unity)
  │  /forward_velocity_controller/commands (Float64MultiArray)
  │  /finger_width_controller/commands     (Float64MultiArray)
  ▼
ros_tcp_endpoint → UR driver → UR3e + RG2
```

---

## 3. Package Map

| Package | What it does |
|---|---|
| `holo_assist_depth_tracker` | Real camera perception: apriltag board detection, cube pose estimation, overlay node, board calibration |
| `holo_assist_depth_tracker_sim` | Geometry-based sim: truth cubes, synthetic perception, moveit bridge, selected-cube adapter, pick-place service, hybrid mode launch, camera alignment node |
| `holo_assist_depth_tracker_sim_interfaces` | Custom ROS 2 message/service types (CubePerceptionStatus, PickCubeToBin, SetAprilCubePose, SetCameraPose) |
| `moveit_robot_control` | Coordinate listener, pick-place sequencer, workspace scene manager, workspace_frame_tf, full hardware launch, full sim launch, RViz configs |
| `moveit_robot_control_msgs` | Custom message types for the coordinate listener (TargetRPY) |
| `ur_onrobot` | Combined UR3e + OnRobot RG2 driver, URDF/xacro, MoveIt config, controller YAML |
| `onrobot_driver` | C++ hardware interface implementing Modbus for the RG2 gripper |
| `onrobot_description` | OnRobot RG2 URDF meshes and xacro |
| `holoassist_unity_bridge` | rosbridge-adjacent Unity ↔ ROS bridge utilities |

---

## 4. TF Tree Design — The Central Problem

Getting the TF tree right was the single most complex engineering challenge in this
project. Every approach has tradeoffs. Here is the full history.

### 4.1 What workspace_frame is

`workspace_frame` is a coordinate frame placed on the physical workspace surface.
Its origin is the centre of the 4-tag AprilTag board. All cube poses are reported
in this frame by `cube_pose_node`. The frame is what links the camera's perception
to the robot's planning frame (`base_link`).

### 4.2 The fundamental TF constraint

**A TF frame can only have one parent.** This drives every design decision below.

### 4.3 Approach 1 — workspace_board_node only (dynamic mode)

`workspace_board_node` subscribes to `/detections_all`, finds the 4 board tags (IDs
0–3), and publishes a dynamic TF:

```
camera_color_optical_frame → workspace_frame
```

**Problem:** `workspace_frame` is a child of `camera_color_optical_frame`. For the
robot planner (`coordinate_listener`) to work in `base_link`, TF must be able to
resolve `workspace_frame ↔ base_link`. This requires `camera_color_optical_frame`
to be reachable from `base_link`, which means `camera_link` must be somewhere in the
same TF tree as `base_link`.

On real hardware, the camera is a standalone mount — it is not in the robot URDF —
so `camera_link` has no parent unless we add one manually.

**Solution used:** Add a static `world → camera_link` TF. Since `world → base_link`
comes from RSP (the URDF has a world root link), both subtrees connect through world:

```
world ──┬── base_link ──► [robot joints]           (RSP)
        └── camera_link ──► camera_color_optical_frame (RealSense driver)
                               └── workspace_frame  (workspace_board_node)
```

This works, but the static camera TF must be measured/approximated.

### 4.4 Approach 2 — workspace_frame_tf (static calibrated mode)

`workspace_frame_tf` loads a calibration YAML saved by `board_calibration_node` and
publishes:

```
base_link → workspace_frame  (static)
```

**Problem:** Now `workspace_frame`'s parent is `base_link`. For `cube_pose_node` to
transform cube detections (which are in camera frame via apriltag_ros TF) to
`workspace_frame`, there must be a path from `camera_color_optical_frame` to
`workspace_frame`. But `camera_link` still has no parent, so there is no path.

In the real hardware launch this is solved because:
- The board was previously calibrated (board_calibration_node ran once)
- The calibration YAML stores `base_link → workspace_frame`
- cube_pose_node detects cubes in camera frame; the TF path goes
  `camera_color_optical_frame → workspace_frame → base_link` but it requires the
  camera to be in the tree

**The real hardware launch adds an implicit static camera TF** even in calibrated
mode because without it `cube_pose_node`'s TF lookups would fail.

### 4.5 Approach 3 — board_calibration_node (one-shot calibration)

`board_calibration_node` is a special node that:
1. Runs `workspace_board_node` to get live `camera → workspace_frame`
2. Uses robot FK from `/joint_states` to know where the robot EE is
3. Mathematically derives `base_link → workspace_frame` from the camera pose +
   board geometry
4. Saves the result to `~/.holoassist/calibration/calibration_latest.yaml`

After calibration, stop `board_calibration_node` and start `workspace_frame_tf`
loading the YAML. Do not run both simultaneously — they would both try to own
`workspace_frame`.

### 4.6 Approach 4 — workspace_align_camera_tf_node (automatic snapping, hybrid)

This is the newest approach, written specifically for the hybrid mode.

**Insight:** If we know `base_link → workspace_frame` (from calibration or the
same defaults as `workspace_frame_tf`), and `workspace_board_node` gives us
`camera_color_optical_frame → workspace_frame` live, we can back-compute the camera
position that makes the two consistent.

**Math:**
```
T(world → camera_link) =
    T(world → base_link)                       [from RSP]
  × T(base_link → workspace_frame)             [from parameters / calibration]
  × inv(T(camera_opt → workspace_frame))       [from workspace_board_node TF]
  × inv(T(camera_link → camera_opt))           [from RealSense driver TF]
```

The node publishes `world → camera_link` dynamically, updating at 10 Hz whenever
the board is visible. When the board is hidden it republishes the last valid TF.

**Key insight:** We can look up `camera_opt → workspace_frame` and
`camera_link → camera_opt` from TF even when `camera_link` is not connected to
`world`, because those frames form their own connected subtree via the RealSense
driver and workspace_board_node publications.

**Result:** When the board becomes visible, the log says:
```
Board detected — camera TF locked. world→camera_link computed and publishing.
```
At that point `workspace_frame` in RViz snaps to exactly the right position
relative to the robot with no manual camera measurement needed.

### 4.7 Summary of TF approaches

| Mode | workspace_frame parent | camera_link TF source | Accuracy |
|---|---|---|---|
| Sim only | `workspace_frame` | Fake sim node | N/A (geometry-driven) |
| Hardware, dynamic | `camera_color_optical_frame` | Static hand-tuned | Depends on measurement |
| Hardware, calibrated | `base_link` | Static (implicit, needed for cube TF) | High (robot FK calibrated) |
| Hybrid, approximate | `camera_color_optical_frame` | Static hand-tuned defaults | Poor (rough approximation) |
| Hybrid, aligned | `camera_color_optical_frame` | `workspace_align_camera_tf_node` (computed) | Good (snaps to calibrated position) |

---

## 5. Perception Pipeline

### 5.1 Nodes

**`apriltag_node`** (from `apriltag_ros`)
- Input: `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`
- Output: `/detections_all` (AprilTagDetectionArray), TF frames per detected tag
- Config: `holo_assist_depth_tracker/config/apriltag_all.yaml`
  - Board tags: IDs 0–3 (36h11 family, 32 mm physical size)
  - Cube tags: IDs 10–33 (36h11 family, 32 mm physical size — 6 per cube face)
- The `apriltag_all.yaml` size field must be `0.032` (metres). Passing mm (e.g. 32)
  causes silently wrong 3-D poses. The launch file validates this and warns.

**`holoassist_workspace_board_node`**
- Input: `/detections_all`
- Output: TF `camera_color_optical_frame → workspace_frame` (dynamic)
- Also publishes: `/holoassist/perception/bench_plane_marker`,
  `/holoassist/perception/workspace_tag_markers`,
  `/holoassist/perception/workspace_axes_marker`
- Config: `holo_assist_depth_tracker/config/workspace.yaml`
- Must NOT run alongside `workspace_frame_tf` — they would conflict on `workspace_frame`

**`holoassist_cube_pose_node`**
- Input: `/detections_all`, TF
- Output: `/holoassist/perception/april_cube_{1-4}_pose` (PoseStamped in workspace_frame)
- Also publishes: `/holoassist/perception/april_cube_{1-4}_marker`,
  `/holoassist/perception/ur3e_base_link0_marker`,
  `/holoassist/perception/bench_plane_marker` (duplicate),
  `/holoassist/perception/cropped_pointcloud`,
  `/holoassist/perception/foreground_pointcloud`
- Config: `holo_assist_depth_tracker/config/cubes.yaml`
- Fuses up to 6 tag detections per cube to get a stable pose

**`holoassist_overlay_node`**
- Input: raw image, `/detections_all`
- Output: `/holoassist/perception/apriltag_overlay` (Image with bounding boxes)
- Useful for debugging which tags are being detected

### 5.2 How cube detection works

1. Camera sees a cube. apriltag_ros detects the visible faces' stickers (up to 3 visible at once, 6 total per cube).
2. Each detection publishes a TF frame in `camera_color_optical_frame`.
3. `cube_pose_node` collects all detections for each cube's tag IDs, averages the positions (weighted or median) to produce a single stable 3-D centre in `workspace_frame`.
4. The pose is published at the camera frame rate (~30 Hz).

### 5.3 Camera hardware

Intel RealSense D435i:
- Has a colour camera, an infrared stereo pair for depth, and an IMU (accel + gyro)
- Does NOT do self-localization (not a T265 tracking camera)
- IMU gives orientation (tilt) but not position — cannot replace a static camera mount TF
- Launch via `camera_only.launch.py` inside `holo_assist_depth_tracker`

### 5.4 Why we can't use the depth sensor for camera localization

This was discussed. The D435i depth sensor measures distances to surfaces in front
of it, not its own position in world. The IMU gives rotation (tilt), not translation.
Visual-inertial odometry (like Intel's T265 or ORB-SLAM3) would give position, but
that requires a different algorithm or hardware entirely.

The correct localization approach for our setup: see the board AprilTags via the
colour camera, which gives camera position relative to the board. Combined with the
known board position in `base_link` (from calibration), this gives full camera pose.
This is exactly what `workspace_align_camera_tf_node` implements.

---

## 6. Simulation Stack

### 6.1 Purpose

The simulation provides a complete testbed for the pick-and-place pipeline without
needing the physical robot, camera, or cubes. Everything is geometry-driven — no
image rendering.

### 6.2 Nodes

**`sim_cube_truth_node`**
- Maintains ground-truth cube poses in `workspace_frame`
- Cubes can be moved via service calls or randomised
- Services: `/holoassist/sim/set_april_cube_pose`, `/holoassist/sim/randomise_april_cubes`, `/holoassist/sim/reset_april_cubes`

**`sim_cube_perception_node`**
- Simulates what the camera would see based on geometric frustum checks
- Publishes `/holoassist/sim/perception/april_cube_{N}_pose` (only when cube is in frustum)
- Implements "last-seen" behaviour: holds last known pose when cube leaves frustum
- Publishes `CubePerceptionStatus` messages (visible_now, last_seen_available)
- Camera can be moved via `/holoassist/sim/set_camera_pose` service

**`sim_cube_moveit_bridge_node`**
- Subscribes to cube poses (either sim or real perception, configurable via `cube_pose_topic_prefix`)
- Publishes `/planning_scene` (collision objects for MoveIt)
- Publishes `/holoassist/teleop/selected_cube_pose` when a cube is selected
- Parameter `cube_pose_topic_prefix`:
  - `"/holoassist/sim/perception"` — pure sim
  - `"/holoassist/perception"` — real camera (hybrid or hardware)

**`selected_cube_to_moveit_target_node`**
- Subscribes to `/holoassist/teleop/selected_cube_pose`
- TF-transforms the pose to `base_link`
- Applies hover offset (default +10 cm Z)
- Publishes `/moveit_robot_control/target_point` and `/moveit_robot_control/target_pose`

### 6.3 Sim TF tree

```
world
  └── base_link (workspace_frame_tf, static)  ← in sim, base_link IS the fixed frame
        └── workspace_frame (workspace_frame_tf, static)
              └── camera_link (sim_cube_truth_node, dynamic)
                    └── camera_color_frame
                          └── camera_color_optical_frame
```

Note: in sim, the workspace_frame_tf publishes `base_link → workspace_frame` with
default values (x=-0.10, y=-0.314, z=0.015). These match the physical trolley layout.

### 6.4 Sim launch files

```
sim_april_cube_truth.launch.py    — sim scene only (truth + camera nodes + RViz)
sim_april_cube_perception.launch.py — adds synthetic perception node
sim_april_cube_moveit.launch.py   — adds MoveIt bridge + selected-cube adapter
full_holoassist_moveit_sim.launch.py (in moveit_robot_control) — full sim with MoveIt robot
```

---

## 7. Hardware Stack

### 7.1 Overview

The full hardware stack runs the real UR3e + RG2 gripper with the real RealSense
camera. It is the production configuration.

### 7.2 Launch

```bash
ros2 launch moveit_robot_control full_holoassist_hardware.launch.py \
  robot_ip:=192.168.0.194 \
  start_camera:=true \
  use_calibrated_workspace:=true \
  calibration_yaml:=~/.holoassist/calibration/calibration_latest.yaml \
  velocity_scale:=0.05
```

### 7.3 What it starts (timed sequence)

| Time | What starts |
|---|---|
| t=0s | `ur_onrobot` robot driver (UR3e + RG2), MoveIt (move_group), perception (camera + apriltag + cubes), workspace_frame_tf or workspace_board_node (depending on `use_calibrated_workspace`) |
| t=8s | `workspace_scene_manager` (trolley mesh, needs TF) |
| t=10s | `coordinate_listener`, `pick_place_sequencer` (need move_group + controller) |
| t=12s | `pick_place_service_node`, `selected_cube_to_moveit_target_node` |
| t=15s | RViz |

### 7.4 use_calibrated_workspace modes

**`use_calibrated_workspace:=true` (default)**
- Loads `calibration_yaml` via `workspace_frame_tf`
- Publishes static `base_link → workspace_frame` from previous calibration
- `workspace_board_node` is NOT started (would conflict)
- Camera still needs to be in TF tree for cube_pose_node (camera TF is implicit from RealSense driver + static mount)

**`use_calibrated_workspace:=false`**
- `workspace_board_node` IS started (dynamically publishes workspace_frame from live board detection)
- `workspace_frame_tf` is NOT started
- Board must be visible at all times or workspace_frame becomes stale

### 7.5 Controller layout on real hardware

```
joint_state_broadcaster          (always active)
scaled_joint_trajectory_controller  (active for MoveIt / pick-place)
finger_width_controller          (active for gripper during pick-place)

For teleoperation (RMRC/Direct Joint):
forward_velocity_controller      (active — replaces scaled_joint_trajectory_controller)
finger_width_controller          (active)
```

Switching for teleoperation:
```bash
ros2 control switch_controllers \
  --activate forward_velocity_controller finger_width_controller \
  --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller
```

---

## 8. Hybrid Mode — Fake Robot + Real Camera

### 8.1 Goal

Run the full autonomous pick-and-place pipeline with the real RealSense camera
detecting real cubes, but execute on a simulated (fake hardware) UR3e rather than
the physical robot. Useful for:
- Testing the perception → planning → execution pipeline before robot is available
- Debugging pose accuracy without risk of hardware damage
- Running demos without needing the full robot connected

### 8.2 Launch

```bash
ros2 launch holo_assist_depth_tracker_sim full_holoassist_hybrid.launch.py \
  start_camera:=true \
  velocity_scale:=0.2
```

Optional workspace position overrides (if defaults don't match your physical layout):
```bash
  workspace_x_m:=-0.10 \
  workspace_y_m:=-0.314 \
  workspace_z_m:=0.015 \
  workspace_roll_rad:=0.0 \
  workspace_pitch_rad:=0.0 \
  workspace_yaw_rad:=0.0
```

### 8.3 TF tree in hybrid mode

```
world
  ├── base_link ──► [sim robot joints]     (robot_state_publisher, fake URDF)
  └── camera_link                          (workspace_align_camera_tf_node, dynamic)
        └── camera_color_frame             (RealSense driver)
              └── camera_color_optical_frame
                    └── workspace_frame    (workspace_board_node, real board detection)
                          └── april_cube_N (cube_pose_node, real cube detection)
```

### 8.4 What runs vs. what doesn't

| Node | Runs? | Notes |
|---|---|---|
| Fake UR3e + RG2 (ros2_control, fake HW) | YES | joint_trajectory_controller |
| MoveIt (move_group, OMPL) | YES | Plans trajectories on fake robot |
| RealSense camera driver | YES | Real camera images |
| apriltag_ros | YES | Real AprilTag detections |
| workspace_board_node | YES | Dynamic workspace_frame from real board |
| cube_pose_node | YES | Real cube poses |
| workspace_align_camera_tf_node | YES | Computes camera TF by snapping board |
| coordinate_listener | YES | Executes MoveIt plans |
| pick_place_sequencer | YES | Runs pick-place sequence |
| pick_place_service_node | YES | Exposes PickCubeToBin service |
| sim_cube_moveit_bridge_node | YES | Bridge with `cube_pose_topic_prefix=/holoassist/perception` |
| selected_cube_to_moveit_target_node | YES | Hover target adapter |
| workspace_frame_tf | NO | Would conflict with workspace_board_node |
| sim_cube_truth_node | NO | No fake cubes needed |
| sim_cube_perception_node | NO | No synthetic perception |
| UR3e hardware driver (ur_ros2_control) | NO | Fake hardware only |

### 8.5 Topic routing in hybrid mode

Cube poses flow from real camera:
```
/holoassist/perception/april_cube_{1-4}_pose  ←  cube_pose_node (real)
```

sim_cube_moveit_bridge_node is parameterised:
```yaml
cube_pose_topic_prefix: /holoassist/perception   # not /holoassist/sim/perception
```

pick_place_service_node:
```yaml
cube_pose_topic_prefix: /holoassist/perception
```

Status subscriptions (CubePerceptionStatus) in the bridge node will receive nothing
because cube_pose_node publishes String status, not CubePerceptionStatus. This is
harmless — the bridge just silently skips visibility warnings.

### 8.6 Testing the hybrid pipeline end-to-end

```bash
# Terminal 1 — launch everything
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch holo_assist_depth_tracker_sim full_holoassist_hybrid.launch.py \
  start_camera:=true velocity_scale:=0.2

# Terminal 2 — wait for board detection log, then verify pipeline
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 run tf2_ros tf2_echo workspace_frame base_link
ros2 topic echo /pick_place/status --once

# Terminal 3 — call the pick-place service
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

---

## 9. Camera Self-Calibration — What We Tried

### 9.1 Approach A — Hand-measured static TF (early attempt)

Pass `camera_x/y/z/pitch/yaw` directly to the hybrid launch as approximate values.
Workspace_frame ends up at approximately the right position but with errors on the
order of 5–15 cm depending on measurement accuracy.

**Why we moved on:** Measuring the physical camera position accurately enough is
tedious and error-prone. Any camera repositioning invalidates it. Misalignment in
RViz makes it hard to verify the pipeline is working.

### 9.2 Approach B — Board calibration (one-shot, for hardware)

`board_calibration_node` runs once at setup:
1. Watches workspace_board_node's live `camera → workspace_frame` TF
2. Watches `/joint_states` for robot FK
3. Computes `base_link → workspace_frame` precisely using robot FK as ground truth
4. Saves to YAML file

This is the production approach for the real hardware launch. It works well but
requires the robot to be connected and joint states available.

**Problem in hybrid mode:** The fake robot has no physical FK to use as a reference
for calibration. We can't run board_calibration_node because there is no real robot
position to anchor the calculation.

### 9.3 Approach C — "Can't the camera figure it out from depth?" (discussed, rejected)

The D435i depth sensor sees the workspace surface but cannot derive its own 3-D
position in world space from depth data alone. The IMU gives tilt (orientation) but
not translation. Visual-inertial odometry (T265-style) would work but we don't have
a T265.

The board AprilTags DO allow the camera to know its pose relative to the board,
which IS what workspace_board_node computes. The remaining unknown is where the
board is relative to the robot.

### 9.4 Approach D — workspace_align_camera_tf_node (current, hybrid)

**The key insight:** "Can we snap the real board to the fake board?"

If we know where workspace_frame should be in `base_link` (from calibration or from
the workspace_frame_tf defaults that encode the physical layout), and workspace_board_node
gives us the camera's view of the board, we can back-compute the camera's position
in world space. No manual measurement, no robot arm FK, just the board detection
and the known workspace position.

The node (`workspace_align_camera_tf_node`):
- Reads `base_link → workspace_frame` from ROS parameters (defaults match
  workspace_frame_tf hardcoded values: x=-0.10, y=-0.314, z=0.015)
- Continuously looks up `camera_color_optical_frame → workspace_frame` from TF
- Computes and publishes `world → camera_link` at 10 Hz
- Falls back to last known TF when board is temporarily hidden
- Logs "Board detected — camera TF locked" on first successful computation

**How to improve further:** Run the board_calibration_node on real hardware to get
the precise `base_link → workspace_frame` values, then pass them as `workspace_x_m`,
`workspace_y_m`, `workspace_z_m`, `workspace_roll_rad`, `workspace_pitch_rad`,
`workspace_yaw_rad` to the hybrid launch. This gives near-perfect camera alignment
automatically once the board is visible.

---

## 10. Pick-and-Place Pipeline

### 10.1 The PickCubeToBin service

**Service:** `/holoassist/pick_cube_to_bin`
**Type:** `holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`

```
string cube_name    # "april_cube_1" .. "april_cube_4", or "1" .. "4"
string bin_id       # "bin_1" .. "bin_4", or "1" .. "4"
---
bool success
string message
```

**What it does:**
1. Looks up the latest pose of the requested cube from `/holoassist/perception/april_cube_N_pose`
2. TF-transforms it from `workspace_frame` to `base_link`
3. Publishes `mode=run` to `/pick_place/mode`
4. Publishes a JSON command to `/pick_place/command`:
   ```json
   {"block_id": "april_cube_1", "x": 0.15, "y": -0.30, "z": 0.02, "bin_id": "bin_1"}
   ```
5. Returns immediately with `success=true` — the actual pick-place runs asynchronously

### 10.2 Pick-place sequencer steps

When `pick_place_sequencer` receives a command, it runs this sequence in a background thread:

1. **Pregrasp** — move to (cube_x, cube_y, cube_z + pregrasp_z_offset), gripper orientation from params
2. **Open gripper** — finger_width_trajectory_controller to open_width (default 0.08 m)
3. **Remove cube from MoveIt scene** — ApplyPlanningScene to clear the collision object
4. **Grasp** — move to (cube_x, cube_y, cube_z + grasp_z_offset)
5. **Close gripper** — to close_width (default 0.0 m)
6. **Lift** — move back to pregrasp Z
7. **Move above bin** — to (bin_x, bin_y, bin_z + place_above_z_offset)
8. **Optional: descent** — to (bin_x, bin_y, bin_z + place_z_offset) if place_descent_enabled
9. **Open gripper** — drop the cube
10. **Add cube back to scene at bin** — ApplyPlanningScene at bin position
11. **Move up** — return to place_above Z

### 10.3 Sequencer parameters (tuned for hardware)

```python
pregrasp_z_offset: 0.10    # 10 cm above cube centre
grasp_z_offset: 0.0        # at cube centre (real: no depth offset since cube_pose_node reports 3D centre)
place_above_z_offset: 0.15 # 15 cm above bin
place_z_offset: 0.05       # 5 cm above bin floor (release point)
place_descent_enabled: True
orientation_mode: "auto"   # auto-selects wrist orientation based on target position
```

### 10.4 Bin positions

Defined in `moveit_robot_control/config/bin_poses.json` in `base_link` frame:

```json
{
  "bin_1": {"xyz": [-0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_2": {"xyz": [-0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_3": {"xyz": [0.30, -0.20, 0.05],  "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_4": {"xyz": [0.30, -0.10, 0.05],  "rpy_deg": [180.0, 0.0, 0.0]}
}
```

These are placeholders and will need to be measured against the physical bin placement.

---

## 11. MoveIt Integration

### 11.1 coordinate_listener

The coordinate_listener sits between the pick-place sequencer and MoveIt. It accepts
target poses/points and handles all the planning complexity:

- Tries a Cartesian (straight-line) path first
- Falls back to full OMPL pose-goal planning if Cartesian fails
- In `orientation_mode=auto`: samples multiple wrist orientations, picks first that
  works, ultimate fallback is position-only (free orientation) so MoveIt can choose
- Checks predicted trajectories for UR flange-to-forearm protective stop zone before
  sending to controller (configurable, `avoid_flange_forearm_clamp`)
- Publishes `/moveit_robot_control/state` (PLANNING, EXECUTING, COMPLETE, FAILED, etc.)
  and `/moveit_robot_control/complete` (triggers sequencer to advance)

### 11.2 Fake vs real hardware differences

| Parameter | Fake hardware (sim/hybrid) | Real hardware |
|---|---|---|
| `require_robot_status` | false | true |
| `require_controller_check` | false | true |
| `trajectory_topic` | `/joint_trajectory_controller/joint_trajectory` | `/scaled_joint_trajectory_controller/joint_trajectory` |
| `velocity_scale` | 0.2–1.0 (can go fast) | 0.05 (start slow, safety) |

### 11.3 MoveIt planning group

All planning uses:
- Group: `ur_onrobot_manipulator`
- End-effector link: `gripper_tcp`
- Planning frame: `base_link`

### 11.4 Common MoveIt failures

**Planning fails immediately (0s):** The target is outside the robot's reachable space. Check cube position in base_link — if camera TF is wrong, cube coordinates will be garbage.

**Planning times out (5s default):** Target is reachable but OMPL can't find a path. Try increasing `pose_goal_planning_time` or changing `orientation_mode`.

**Trajectory rejected (clamp check):** `avoid_flange_forearm_clamp` is firing. The planned trajectory would bring the flange dangerously close to the forearm. The target position may be behind the robot or at an unusual angle.

**FAILED immediately after PLANNING:** Controller not active (fake HW: was `joint_trajectory_controller` spawned? real HW: is `scaled_joint_trajectory_controller` active?).

---

## 12. RViz Configuration

### 12.1 holoassist_hw.rviz

Located at `moveit_robot_control/rviz/holoassist_hw.rviz`.

Used by:
- `holoassist_4tag_board_4cube.launch.py` (perception-only)
- `full_holoassist_hardware.launch.py`
- `full_holoassist_hybrid.launch.py`

**Displays included:**
- Grid
- RobotModel (from /robot_description topic)
- TF (disabled by default — enable to debug frame tree)
- MotionPlanning (MoveIt plugin — plan + execute from RViz)
- Trolley Mesh MarkerArray (`/workspace_scene/markers`, Transient Local durability)
- Bench Plane Marker (`/holoassist/perception/bench_plane_marker`)
- Workspace Tag Markers (`/holoassist/perception/workspace_tag_markers`)
- Workspace Axes Marker (`/holoassist/perception/workspace_axes_marker`)
- Cube1–4 Perceived Marker (`/holoassist/perception/april_cube_{N}_marker`)
- Robot Base Marker (`/holoassist/perception/ur3e_base_link0_marker`)
- Cropped PointCloud (`/holoassist/perception/cropped_pointcloud`, disabled)
- Foreground PointCloud (`/holoassist/perception/foreground_pointcloud`, disabled)
- Camera Debug Image (`/holo_assist_depth_tracker/debug_image`, bottom panel)

**Fixed frame:** `base_link`

### 12.2 Issues we hit with RViz

**Empty RViz (only Grid):** First open used wrong path due to `$(ros2 pkg prefix --share)` doubling. `--share` already returns `share/<pkg>`, so the path was doubled. Fixed: use `$(ros2 pkg prefix moveit_robot_control)/share/moveit_robot_control/rviz/holoassist_hw.rviz`.

**Wrong MoveIt plugin class:** Used `moveit_ros_visualization/MotionPlanning` — wrong. RViz logged error and showed nothing. Correct class name: `moveit_rviz_plugin/MotionPlanning`.

**Trolley Mesh MarkerArray empty:** Topic durability must be `Transient Local` in the RViz subscriber config and `workspace_scene_manager` must be running. The mesh is published once with Transient Local on startup. A volatile subscriber will never see it.

**Workspace markers appear in wrong place (misaligned TF):** The static camera TF defaults (camera_y=0.9, camera_z=0.65, etc.) put workspace_frame somewhere far from the robot. Once `workspace_align_camera_tf_node` locks on the board, everything snaps to the correct position.

### 12.3 The Camera Debug Image panel

The holoassist_hw.rviz Camera Debug Image panel subscribes to
`/holo_assist_depth_tracker/debug_image` — the processed depth tracker debug output.
If you want to see the raw camera feed instead, change the topic to
`/camera/camera/color/image_raw` in RViz. The panel appears at the bottom right.

---

## 13. Launch File Reference

### Full launch files

| File | Package | Purpose |
|---|---|---|
| `full_holoassist_hardware.launch.py` | `moveit_robot_control` | Complete hardware run: UR3e + RG2 + MoveIt + perception + pick-place |
| `full_holoassist_moveit_sim.launch.py` | `moveit_robot_control` | Complete sim run: fake robot + MoveIt + sim cubes + pick-place |
| `full_holoassist_hybrid.launch.py` | `holo_assist_depth_tracker_sim` | Fake robot + MoveIt + REAL camera + pick-place |

### Perception-only launch files

| File | Package | Purpose |
|---|---|---|
| `holoassist_4tag_board_4cube.launch.py` | `holo_assist_depth_tracker` | Real camera perception only (with optional RViz + scene) |
| `camera_only.launch.py` | `holo_assist_depth_tracker` | Just the RealSense driver |

### Sim launch files

| File | Package | Purpose |
|---|---|---|
| `sim_april_cube_truth.launch.py` | `holo_assist_depth_tracker_sim` | Truth cubes + fake camera |
| `sim_april_cube_perception.launch.py` | `holo_assist_depth_tracker_sim` | + synthetic perception |
| `sim_april_cube_moveit.launch.py` | `holo_assist_depth_tracker_sim` | + MoveIt bridge + adapter |

### Component launch files

| File | Package | Purpose |
|---|---|---|
| `coordinate_listener.launch.py` | `moveit_robot_control` | Just the coordinate listener node |
| `pick_place.launch.py` | `moveit_robot_control` | Just the pick-place sequencer |
| `pick_place_system.launch.py` | `moveit_robot_control` | Scene + listener + sequencer together |
| `workspace_scene.launch.py` | `moveit_robot_control` | Just the workspace scene manager |
| `board_calibration.launch.py` | `holo_assist_depth_tracker` | One-shot workspace calibration |

### Key launch arguments

**full_holoassist_hardware.launch.py:**
```
robot_ip              IP of UR3e (required)
ur_type               ur3e (default)
onrobot_type          rg2 (default)
start_camera          true (default) — start RealSense driver
start_pick_place      true (default) — start sequencer + service
start_rosbridge       true (default) — Unity WebSocket bridge
start_moveit          true (default)
use_calibrated_workspace  true (default) — use calibration YAML vs live board
calibration_yaml      ~/.holoassist/calibration/calibration_latest.yaml
velocity_scale        0.05 (default) — keep slow for safety
use_rviz              true (default)
```

**full_holoassist_hybrid.launch.py:**
```
start_camera          true (default) — start RealSense driver
velocity_scale        0.05 (default) — raise to 0.2 for faster sim
workspace_x_m         -0.10 (default) — known base_link→workspace_frame X
workspace_y_m         -0.314 (default) — Y
workspace_z_m         0.015 (default) — Z
workspace_roll_rad    0.0
workspace_pitch_rad   0.0
workspace_yaw_rad     0.0
camera_link_frame     camera_link (default)
start_pick_place      true (default)
start_moveit          true (default)
robot_base_yaw_rad    3.14159 (default) — robot mounting angle, π = standard HoloAssist trolley
use_rviz              true (default)
```

---

## 14. Diagnostic Cheatsheet

### Is the camera publishing?
```bash
ros2 topic hz /camera/camera/color/image_raw
```

### Is apriltag_ros detecting anything?
```bash
ros2 topic echo /detections_all --once
```

### Is workspace_frame established?
```bash
ros2 run tf2_ros tf2_echo workspace_frame base_link
```
Times out → board tags not visible or workspace_board_node not running.

### Is the camera TF alignment working?
```bash
ros2 topic echo /tf --once  # look for world → camera_link transforms
# OR check the node log for "Board detected — camera TF locked"
```

### Are cube poses arriving?
```bash
ros2 topic echo /holoassist/perception/april_cube_1_pose --once
ros2 topic echo /holoassist/perception/april_cube_2_pose --once
```

### Is the pick-place service available?
```bash
ros2 service list | grep pick_cube
```

### Is the sequencer ready?
```bash
ros2 topic echo /pick_place/status --once
```

### Is MoveIt up?
```bash
ros2 service list | grep apply_planning_scene
ros2 service list | grep compute_ik
```

### What controllers are active?
```bash
ros2 control list_controllers
```

### See the full TF tree
```bash
ros2 run tf2_tools view_frames  # saves frames.pdf
# OR
ros2 run tf2_ros tf2_monitor
```

### Quick pick-place test
```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: '1', bin_id: '1'}"
```

### Move robot to a test point (XYZ only)
```bash
ros2 topic pub --once /moveit_robot_control/target_point geometry_msgs/msg/Point \
  "{x: 0.15, y: -0.30, z: 0.10}"
```

### Watch pick-place progress
```bash
ros2 topic echo /pick_place/status
ros2 topic echo /moveit_robot_control/state
```

### Move a sim cube
```bash
ros2 service call /holoassist/sim/set_april_cube_pose \
  holo_assist_depth_tracker_sim_interfaces/srv/SetAprilCubePose \
  "{cube_name: april_cube_1, cube_id: 0, x: 0.1, y: 0.0, z: 0.02, yaw: 0.0}"
```

### Select a cube for teleop/hover mode
```bash
ros2 topic pub --once /holoassist/teleop/selected_cube std_msgs/msg/String "{data: 'april_cube_1'}"
```

---

## 15. Problems We Hit and How We Solved Them

### 15.1 RViz opened but only showed the Grid

**Root cause:** The `--share` flag in `ros2 pkg prefix` already includes the
`share/<pkg>` path, so appending `/share/<pkg>` again doubled it. RViz silently
fell back to default config.

**Fix:** Use `$(ros2 pkg prefix moveit_robot_control)/share/moveit_robot_control/rviz/holoassist_hw.rviz`

### 15.2 MoveIt MotionPlanning panel not appearing

**Root cause:** Wrong RViz plugin class name — `moveit_ros_visualization/MotionPlanning`
is incorrect. The actual class is `moveit_rviz_plugin/MotionPlanning`.

**Fix:** Correct the class name in the .rviz file. RViz logs the error on startup.

### 15.3 Hybrid launch file not found after build

**Root cause:** `colcon build --symlink-install` with Python packages creates symlinks
for existing files but does NOT re-evaluate `glob()` calls in setup.py when new
files are added. The glob runs at install time when it was first run.

**Fix:** `touch setup.py` before rebuilding to force re-evaluation:
```bash
touch src/holo_assist_depth_tracker_sim/setup.py
colcon build --packages-select holo_assist_depth_tracker_sim --symlink-install
```

### 15.4 Multiline shell command with trailing spaces failed

**Root cause:** Shell line continuations with `\` + trailing space cause parsing errors.

**Fix:** Keep the command on one line or ensure no trailing spaces after `\`.

### 15.5 workspace_frame not in TF tree (hybrid TF disconnection)

**Root cause:** In hybrid mode, `workspace_board_node` publishes
`camera_color_optical_frame → workspace_frame`, but `camera_link` had no parent
in the TF tree. RSP provides `world → base_link` separately. The two subtrees
were disconnected — TF lookup from workspace_frame to base_link timed out.

**Fix:** Add `world → camera_link` TF (initially static, later dynamic via
`workspace_align_camera_tf_node`) to connect both subtrees through `world`.

### 15.6 workspace_frame misaligned in RViz (wrong camera TF)

**Root cause:** The approximate static camera TF defaults (camera_y=0.9, camera_z=0.65,
etc.) don't match the physical camera position, so workspace_frame lands in the
wrong place in world space. RViz shows TF frames scattered far from the robot.

**Fix:** `workspace_align_camera_tf_node` — instead of guessing the camera position,
derive it from the board detection + known workspace position. Frames snap into
place automatically when the board is visible.

### 15.7 sim_cube_moveit_bridge_node subscribed to wrong topics in hybrid

**Root cause:** The node had hardcoded topic strings `/holoassist/sim/perception/...`.

**Fix:** Added `cube_pose_topic_prefix` parameter (default `/holoassist/sim/perception`).
Hybrid mode passes `/holoassist/perception` to redirect to real camera topics.

### 15.8 pick_place_service_node error message said "Is sim_cube_truth_node running?"

**Root cause:** Error message was sim-specific and misleading in hardware/hybrid mode.

**Fix:** Updated to generic message: "Is the perception pipeline running and the cube visible?"

### 15.9 apriltag_ros size validation warning

**Root cause:** Physical board uses 32 mm tags. If the YAML has `size: 32` instead
of `size: 0.032`, detection works geometrically but reported 3-D poses are ~1000x wrong.

**Fix:** Added validation in `holoassist_4tag_board_4cube.launch.py` (OpaqueFunction)
that reads the YAML and warns if size looks like mm instead of metres.

### 15.10 holo_assist_depth_tracker_sim_interfaces not built

**Root cause:** Building `holo_assist_depth_tracker_sim` alone fails if the interfaces
package hasn't been built first (Python import of the service type fails).

**Fix:** Always build both together:
```bash
colcon build --packages-select holo_assist_depth_tracker_sim_interfaces holo_assist_depth_tracker_sim --symlink-install
```

---

## 16. What Still Needs Doing

### High priority (needed before demo)

**Bin position calibration**
The bin_poses.json coordinates (-0.30, -0.20), (-0.30, -0.10), etc. are placeholder
estimates. They need to be measured in `base_link` frame against the physical bin
positions on the trolley. Use:
```bash
ros2 topic pub --once /moveit_robot_control/target_point geometry_msgs/msg/Point "{x: ?, y: ?, z: ?}"
```
and jog to each bin centre, reading off `/joint_states` + FK.

**Board calibration for hardware**
Run `board_calibration_node` with the physical robot connected:
```bash
ros2 launch holo_assist_depth_tracker board_calibration.launch.py
```
Save the result to `~/.holoassist/calibration/calibration_latest.yaml` and use it
in `full_holoassist_hardware.launch.py` with `use_calibrated_workspace:=true`.

**Hybrid mode end-to-end test**
With real camera detecting real cubes and fake robot: verify the full
PickCubeToBin service → sequencer → coordinate_listener → MoveIt → fake execution
loop completes without errors. Monitor `/pick_place/status` and `/moveit_robot_control/state`.

**Grasp Z offset tuning**
`grasp_z_offset: 0.0` means the robot tries to reach the exact cube centre position
from cube_pose_node. cube_pose_node reports the 3-D cube centre (≈ half cube height
above workspace surface ≈ 20 mm). Verify this is correct and the gripper TCP clears
the table at grasp. May need to tune to +0.005 or similar.

### Medium priority

**workspace_align_camera_tf_node workspace position from calibration YAML**
Currently the node takes individual x/y/z/roll/pitch/yaw parameters. Wire it to
load the calibration YAML automatically (using the same format as workspace_frame_tf:
`x_m`, `y_m`, `z_m`, `roll_rad`, `pitch_rad`, `yaw_rad` parameters).

**Mode switching: teleop ↔ autonomous**
Currently you either run the teleoperation stack (velocity controllers) or the
autonomous stack (trajectory controller + MoveIt). Need a clean switch between them
without restarting everything. The e-stop dashboard already handles this for teleop;
need the equivalent for transitioning back to autonomous.

**RViz sim config**
The existing sim RViz config (`moveit_robot_control/rviz/`) shows the sim cube
markers but doesn't include all the same displays as holoassist_hw.rviz. Merge
the display sets or create a unified config.

**rosbridge for hybrid mode**
The hybrid launch doesn't start rosbridge. If testing Unity ↔ ROS with the hybrid
mode (fake robot + real perception), add `start_rosbridge:=true` arg.

**Collision protection calibration**
On real hardware: `MeshCollisionGuard.cs` in Unity has known-broken arm control
(see CLAUDE.md: MeshCollisionGuard breaks arm control). Needs table height and
self-collision margins tuned with real hardware.

### Low priority / future

**Human vs CPU race mode**
System-level HD evaluation criterion — quantitative comparison between teleop and
autonomous sorting with overlay. Not started.

**WiFi dropout auto e-stop**
System-level HD evaluation criterion — detect Unity connection loss and auto-trigger
e-stop. Not started.

**MR UI for depth camera feed**
System-level HD evaluation criterion — visualise camera feed and debug markers in
the XR headset view.

**Depth image based cube detection**
The current system uses AprilTag stickers on each cube face. An alternative approach
using the D435i depth image for 3-D segmentation would allow detecting cubes without
tags — useful if tags are occluded or damaged.

**Dynamic bin assignment**
The bin_poses.json is static. A future version could publish bin positions from ROS
topics (e.g. updated by the XR scene) and have the sequencer look them up at
pick-place time.

---

## 17. Build Reference

### Full workspace build

```bash
cd /home/john/git/RS2-HoloAssist/main/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Selective build (faster)

```bash
# Perception + sim + interfaces (most common during dev)
colcon build --packages-select \
  holo_assist_depth_tracker_sim_interfaces \
  holo_assist_depth_tracker_sim \
  holo_assist_depth_tracker \
  --symlink-install

# MoveIt control package only
colcon build --packages-select moveit_robot_control --symlink-install

# After adding a new launch/config file (forces glob re-evaluation)
touch src/holo_assist_depth_tracker_sim/setup.py
colcon build --packages-select holo_assist_depth_tracker_sim --symlink-install
```

### After build, always re-source

```bash
source install/setup.bash
```

### Check what's installed

```bash
ros2 pkg list | grep -E "holo_assist|moveit_robot"
ros2 launch holo_assist_depth_tracker_sim full_holoassist_hybrid.launch.py --show-args
```

---

*Last updated: 2026-05-06. This document covers all engineering decisions made
during the John perception/autonomous subsystem development sessions.*
