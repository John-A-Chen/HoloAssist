# HoloAssist Integration Reference

This document covers the `j0hn` branch integration: the full sim + hardware launch architecture, parameter tuning reference, and merge guide.

---

## 1. Integration state

The `j0hn` branch is a complete integration of:

- **John's perception side** (`holo_assist_depth_tracker`): AprilTag board solver, 4-cube tracker, camera self-localisation
- **Ollie's motion side** (`moveit_robot_control`): topic-driven MoveIt planner, trajectory executor, workspace scene manager
- **Extended layers** (integrated on `j0hn`): pick-place sequencer, pick-place service (PickCubeToBin), hardware launch, XR rosbridge streaming

All three operational modes are functional:

| Mode | Launch | Status |
|---|---|---|
| Full simulation | `full_holoassist_moveit_sim.launch.py` | Working — fake HW + full pick-place |
| Hardware XR only | `holoassist_full_xr.launch.py` | Built — needs hardware test with RealSense |
| Full hardware | `full_holoassist_hardware.launch.py robot_ip:=<ip>` | Built — needs hardware test |

---

## 2. Full simulation stack

### Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

### What starts (with timing)

```
t=0s   robot_bringup      ros2_control_node (fake HW) + RSP + controller spawner
t=0s   moveit_stack       move_group + OMPL + SRDF
t=3s   workspace_tf       static base_link → workspace_frame TF
t=3s   workspace_scene    trolley mesh visual in RViz
t=4s   coordinate_listener
t=4s   pick_place_sequencer
t=5s   perception_stack   sim truth cubes + fake camera + visibility perception
t=6s   moveit_bridge      perceived cubes → MoveIt planning scene
t=6s   selected_cube_adapter   perception → MoveIt goal topics
t=6s   pick_place_service /holoassist/pick_cube_to_bin service
t=7s   RViz
```

### Trigger a pick

```bash
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

---

## 3. Frame model (simulation)

```
base_link
  └─ workspace_frame   [static TF from workspace_frame_tf node]
       └─ camera_link  [sim_cube_truth_node]
            └─ camera_color_frame → camera_color_optical_frame
       └─ apriltag_cube_1 ... apriltag_cube_4  [sim_cube_truth_node]
```

MoveIt planning frame: `base_link`

---

## 4. Parameter ownership — single source map

### A. Integrated sim config

**File:** `moveit_robot_control/config/full_holoassist_sim.yaml`

| Parameter | Value | Meaning |
|---|---|---|
| `holoassist_workspace_frame_tf.x_m` | 0.0 | Workspace centred on robot X axis |
| `holoassist_workspace_frame_tf.y_m` | -0.315 | 315 mm in front of robot |
| `holoassist_workspace_frame_tf.z_m` | 0.02 | 20 mm above base_link origin |
| `workspace_scene_manager.table_mesh_xyz` | [0.031, -0.210, -1.05] | Trolley mesh placement in base_link |
| `workspace_scene_manager.table_mesh_scale` | [1.0, 1.0, 1.0] | No scale adjustment |
| `holoassist_selected_cube_to_moveit_target.target_z_offset_m` | 0.10 | Hover 10 cm above cube centre |
| `holoassist_selected_cube_to_moveit_target.target_roll_rad` | π | Gripper facing down |

### B. Hardware config

**File:** `moveit_robot_control/config/full_holoassist_hw.yaml`

Same as sim config but without `holoassist_workspace_frame_tf` section (workspace TF is solved by the board node on hardware).

### C. Pick-place sequencer params (set in launch file)

| Parameter | Value | Notes |
|---|---|---|
| `pregrasp_z_offset` | 0.10 | Approach above cube centre |
| `grasp_z_offset` | 0.0 | Descend to cube centre (both sim and hardware report centres) |
| `place_above_z_offset` | 0.15 | Height above bin before descent |
| `place_z_offset` | 0.05 | Final place height |
| `place_descent_enabled` | true | Gripper descends before releasing |

**Z offset calibration note:** Both `sim_cube_truth_node` and `cube_pose_node` (hardware) report cube **centres** at Z ≈ +0.020 m above the workspace surface (half of a 4 cm cube). `grasp_z_offset = 0.0` places the gripper at cube centre height. If hardware consistently misses, adjust this parameter rather than the cube pose pipeline.

### D. Workspace board / cube geometry (hardware)

**File:** `holo_assist_depth_tracker/config/workspace.yaml`

- `board_width_m: 0.700`, `board_depth_m: 0.500`
- `board_tag_center_edge_offset_m: 0.016`
- `robot_x_m: 0.450`, `robot_y_m: 0.564`, `robot_z_m: -0.015`

### E. Sim scene geometry

**File:** `holo_assist_depth_tracker_sim/config/sim_scene.yaml`

Board and workspace dimensions used by sim nodes.

### F. Sim cubes

**File:** `holo_assist_depth_tracker_sim/config/sim_cubes.yaml`

Default cube spawn positions in workspace_frame.

### G. Fake camera

**File:** `holo_assist_depth_tracker_sim/config/sim_camera.yaml`

Camera default pose in workspace_frame (XYZ + RPY). Default: `[0.0, -0.55, 0.45, 0.0, 0.68, 1.575]`.

---

## 5. Key integration fixes (history)

These were addressed during the j0hn integration pass:

| Issue | Fix |
|---|---|
| `scaled_joint_trajectory_controller` not found in sim | Added `require_controller_check` param (default true; set false for sim) |
| Robot goes to wrong height (too high for grasp) | `grasp_z_offset: 0.0` — sim reports cube centres, not top face |
| Robot drops cube at bin without descending | `place_descent_enabled: true` |
| Robot appeared slightly left of workspace board in RViz | `x_m: 0.0` in workspace_frame_tf (was 0.0275) |
| pick_place_service_node hardcoded to sim truth topics | Added `cube_pose_topic_prefix` param; hardware default = `/holoassist/perception` |

---

## 6. Validation checklist

After launching sim:

```bash
# Topics present?
ros2 topic list | grep -E "workspace_scene/markers|moveit_robot_control|pick_place|holoassist/sim"

# Workspace TF?
ros2 run tf2_ros tf2_echo base_link workspace_frame

# Target topics receiving data?
ros2 topic echo /moveit_robot_control/target_pose --once
ros2 topic echo /moveit_robot_control/state --once

# Pick service working?
source install/setup.bash
ros2 service call /holoassist/pick_cube_to_bin \
  holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"

# Watch execution
ros2 topic echo /moveit_robot_control/state
```

After launching hardware XR:

```bash
# Board locked?
ros2 topic echo /holoassist/perception/workspace_mode --once

# Cube poses present?
ros2 topic echo /holoassist/perception/april_cube_1_pose --once

# rosbridge running?
ros2 topic list | grep rosbridge
```

---

## 7. Merge guide (`j0hn` → `main`)

### Prerequisites

- All three modes validated on target machine
- Hardware XR launch tested with real RealSense + board
- No uncommitted changes on `j0hn`

### Merge sequence

```bash
# Ensure j0hn is current
git checkout j0hn
git fetch origin
git pull --ff-only origin j0hn

# Open merge target
git checkout main
git pull --ff-only origin main

# Merge
git merge --no-ff j0hn

# Rebuild integration packages
colcon build --symlink-install \
  --packages-select moveit_robot_control holo_assist_depth_tracker holo_assist_depth_tracker_sim

# Verify
source install/setup.bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py

# Push
git push origin main
```

### Conflict hot spots

Watch these paths first:

- `holo_assist_depth_tracker_sim/launch/*.launch.py` — new pick-place nodes and sim topic prefix
- `moveit_robot_control/moveit_robot_control_node/moveit_robot_control.py` — `require_controller_check` param
- `moveit_robot_control/launch/coordinate_listener.launch.py` — `require_controller_check` arg
- `moveit_robot_control/launch/full_holoassist_moveit_sim.launch.py` — pick-place nodes, sim topic prefix
- `moveit_robot_control/setup.py` — new launch files (hardware, gazebo) must remain in explicit list

### Post-merge sanity checks

1. Build succeeds for all integration packages
2. `full_holoassist_moveit_sim.launch.py` starts without missing-node errors
3. RViz shows trolley + robot + workspace + camera + cubes
4. PickCubeToBin service responds and robot executes pick
5. `holoassist_full_xr.launch.py` starts without errors (rosbridge on port 9090)
6. `coordinate_listener` works when legacy `TargetRPY` package is unavailable
