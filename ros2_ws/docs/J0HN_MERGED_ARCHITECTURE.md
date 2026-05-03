# j0hn Branch — Merged Architecture Guide

This document is the top-level map of the `j0hn` branch, which merges John's perception side and Ollie's motion execution side and extends both with a pick-place service layer, a hardware launch, and an XR streaming path.

Drill into package-level docs from the links in Section 6.

---

## 1. Operational Modes

The system runs in one of three modes depending on what hardware is available:

### Mode A — Full simulation (development)

Everything runs on the laptop with no physical robot or camera. Fake hardware plugin, static workspace TF, geometry-based synthetic perception.

**Entry point:**
```bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

What starts:
- `ros2_control_node` with fake hardware plugin (joint_trajectory_controller)
- MoveIt (move_group with OMPL, fake hardware SRDF)
- Static `base_link → workspace_frame` TF broadcaster
- Workspace scene manager (trolley mesh visual)
- Sim truth cubes + fake camera + visibility-based perception
- MoveIt planning scene bridge (perceived cubes → collision objects)
- Selected-cube → MoveIt target adapter
- Pick-place sequencer (state machine: pregrasp → grasp → bin)
- Pick-place service (PickCubeToBin service → sequencer command)
- RViz

### Mode B — Hardware XR only (no robot required)

RealSense camera tracks the board and cubes; poses stream to Unity/HoloLens via rosbridge WebSocket. No robot control.

**Entry point:**
```bash
ros2 launch holo_assist_depth_tracker holoassist_full_xr.launch.py
```

What starts:
- RealSense camera (realsense2_camera)
- apriltag_ros (tag detection → TF)
- workspace_board_node (locks workspace_frame from board tags 0–3)
- cube_pose_node (tracks cubes 1–4 from face tags 10–33)
- rosbridge WebSocket server (port 9090)

Unity subscribes to cube pose topics via `ws://<host>:9090`.

### Mode C — Full hardware stack

Physical UR3e + OnRobot gripper + RealSense camera. Full pick-place automation and XR streaming.

**Entry point:**
```bash
ros2 launch moveit_robot_control full_holoassist_hardware.launch.py robot_ip:=<ip>
```

What starts:
- UR3e driver (ur_ros2_control_node, scaled_joint_trajectory_controller)
- OnRobot gripper (tool_communication → /tmp/ttyUR, finger_width_trajectory_controller)
- MoveIt (move_group, real hardware SRDF)
- Full perception pipeline (camera + apriltag + board solver + cube tracker)
- Workspace scene manager (trolley mesh visual)
- Coordinate listener (hardware mode: require_controller_check=true, require_robot_status=true)
- Pick-place sequencer
- Pick-place service
- rosbridge WebSocket server (port 9090)
- RViz

---

## 2. System split by responsibility

### Perception (`holo_assist_depth_tracker`)

Key nodes:
- `holoassist_workspace_board_node` — SVD/Kabsch solve on board corner tags, locks workspace_frame
- `holoassist_cube_pose_node` — 4-cube face-tag consensus estimation
- `holoassist_overlay_node` — AprilTag debug overlay on RGB image
- `holo_assist_depth_tracker_node` — RealSense depth blob tracker (optional)

Config files:
- `config/apriltag_all.yaml` — 34 tag IDs [0-3, 10-33]
- `config/workspace.yaml` — board geometry, robot position in workspace
- `config/cubes.yaml` — cube tag groups, face order, physical dimensions

### Simulation perception (`holo_assist_depth_tracker_sim`)

Key nodes:
- `sim_cube_truth_node` — draggable truth cubes + interactive markers
- `sim_cube_perception_node` — visibility-based synthetic perception
- `sim_cube_moveit_bridge_node` — perceived cubes → MoveIt planning scene
- `selected_cube_to_moveit_target_node` — TF + hover offset → MoveIt goal topics
- `pick_place_service_node` — PickCubeToBin service bridge

### Motion execution (`moveit_robot_control`)

Key nodes:
- `coordinate_listener` (`MoveItCoordinateTopicControl`) — goal topic consumer, Cartesian/pose planner, trajectory executor
- `pick_place_sequencer` — pick/place state machine, bin-aware sequencing
- `workspace_scene_manager` — MoveIt scene collision objects + trolley mesh visual
- `workspace_frame_tf` — static workspace TF broadcaster (sim only)

---

## 3. End-to-end topic flow

### Hardware perception path

```
RealSense camera
  → /camera/camera/color/image_raw
      → apriltag_ros → /detections_all + TF (tag36h11:N in camera_optical_frame)
          → workspace_board_node
              → TF: camera_optical_frame → workspace_frame  [locked]
              → TF: workspace_frame → ur3e_base_link0       [robot position]
          → cube_pose_node
              → /holoassist/perception/april_cube_[1-4]_pose  [PoseStamped in workspace_frame]
              → TF: workspace_frame → apriltag_cube_[1-4]
```

### Sim perception path

```
sim_cube_truth_node
  → /holoassist/sim/truth/april_cube_[1-4]_pose  [draggable truth cubes]
  → camera TF tree (workspace_frame → camera_link → ...)

sim_cube_perception_node
  → /holoassist/sim/perception/april_cube_[1-4]_pose  [visibility-filtered]

sim_cube_moveit_bridge_node
  → /planning_scene  [collision objects]
  → /holoassist/teleop/selected_cube_pose
```

### Pose handoff path (sim)

```
selected_cube_to_moveit_target_node
  subscribes: /holoassist/teleop/selected_cube_pose  [PoseStamped in workspace_frame]
  TF lookup:  workspace_frame → base_link
  publishes:  /moveit_robot_control/target_pose  [Pose in base_link + hover offset]
              /moveit_robot_control/target_point [Point in base_link + hover offset]
```

### Service-triggered pick path

```
ros2 service call /holoassist/pick_cube_to_bin  →  pick_place_service_node
  reads: /holoassist/sim/truth/april_cube_N_pose     (sim)
  reads: /holoassist/perception/april_cube_N_pose    (hardware)
  TF:    workspace_frame → base_link
  publishes: /pick_place/mode = "run"
             /pick_place/command = JSON {"block_id":..., "x":..., "y":..., "z":..., "bin_id":...}

pick_place_sequencer
  subscribes: /pick_place/command, /pick_place/mode
  publishes:  /moveit_robot_control/target_pose  (pregrasp, grasp, place, bin positions)
              gripper commands

coordinate_listener
  subscribes: /moveit_robot_control/target_pose, /moveit_robot_control/target_point
  executes:   Cartesian path → pose-goal fallback → trajectory publish → joint convergence
```

### XR streaming path (hardware)

```
cube_pose_node
  → /holoassist/perception/april_cube_[1-4]_pose  [PoseStamped in workspace_frame]

rosbridge_websocket (port 9090)
  ← Unity/HoloLens subscribes via WebSocket (roslibjs or Unity Robotics Hub)
```

---

## 4. Core frame model

| Frame | Owner | Notes |
|---|---|---|
| `base_link` | robot URDF / robot_state_publisher | MoveIt planning frame |
| `workspace_frame` | sim: workspace_frame_tf node; hardware: workspace_board_node | Perception anchor |
| `ur3e_base_link0` | workspace_board_node (hardware only) | Robot base in workspace_frame |
| `camera_color_optical_frame` | RealSense driver (hardware) / sim_cube_truth_node (sim) | AprilTag detection frame |
| `tag36h11:<id>` | apriltag_ros | Per-tag TF in camera frame |
| `apriltag_cube_[1-4]` | cube_pose_node | Cube centre TFs in workspace_frame |

**Sim workspace_frame** (static): `base_link → workspace_frame = (0.0, -0.315, +0.020)` metres, zero rotation.

**Hardware workspace_frame** (dynamic, locked once): origin at board corner tag 0 centre, +X along board width (700 mm), +Y along board depth (500 mm), +Z up from surface.

Hardware TF tree (perception side):
```
camera_color_optical_frame
  → workspace_frame
    → ur3e_base_link0
    → apriltag_cube_[1-4]
```

**Note:** On hardware, `ur3e_base_link0` and the robot URDF's `base_link` are the same physical point but different TF frame names. The two TF trees are intentionally disconnected in XR-only mode (no MoveIt needed). For future full hardware MoveIt integration, set `robot_tf_child_frame: base_link` in `workspace.yaml` to merge the trees.

---

## 5. Launch entry points

| Mode | Command | Robot | Camera | MoveIt | Rosbridge |
|---|---|---|---|---|---|
| Full sim | `ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py` | fake | fake | yes | no |
| Hardware XR only | `ros2 launch holo_assist_depth_tracker holoassist_full_xr.launch.py` | no | real | no | yes |
| Full hardware | `ros2 launch moveit_robot_control full_holoassist_hardware.launch.py robot_ip:=<ip>` | real | real | yes | yes |
| Perception only | `ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py` | no | real | no | no |

---

## 6. Interfaces that must remain stable

**MoveIt input topics:**
- `/moveit_robot_control/target_point` (`geometry_msgs/msg/Point`)
- `/moveit_robot_control/target_pose` (`geometry_msgs/msg/Pose`)
- `/moveit_robot_control/target` (`moveit_robot_control_msgs/msg/TargetRPY`, legacy)

**MoveIt output topics:**
- `/moveit_robot_control/state`, `/moveit_robot_control/status`, `/moveit_robot_control/debug`, `/moveit_robot_control/complete`

**Pick-place service:**
- `/holoassist/pick_cube_to_bin` (`holo_assist_depth_tracker_sim_interfaces/srv/PickCubeToBin`)

**Cube pose topics:**
- `/holoassist/perception/april_cube_[1-4]_pose` (hardware)
- `/holoassist/sim/truth/april_cube_[1-4]_pose` (sim)

**`workspace_frame`** semantics: board-corner origin, Z up from surface, cube poses expressed as centres relative to this frame.

---

## 7. Deeper reference docs

| Document | Contents |
|---|---|
| `docs/POSE_HANDOFF_CONTRACT.md` | Full perception→motion interface spec |
| `src/holo_assist_depth_tracker/docs/PERCEPTION_PIPELINE_REFERENCE.md` | Board solving, cube tracking, XR output, camera self-localisation |
| `src/moveit_robot_control/docs/MOTION_EXECUTION_REFERENCE.md` | Planning strategy, pick-place sequencer, hardware vs sim params |
| `src/moveit_robot_control/INTEGRATION_AND_MERGE.md` | Integration history, tuning reference, merge guide |
