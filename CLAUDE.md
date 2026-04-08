# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

HoloAssist is a ROS 2-based XR teleoperation and perception framework.  
This branch (`john`) currently centers on:

- Depth-based perception (`holo_assist_depth_tracker`)
- MoveIt integration helpers (`holoassist_manipulation`, `holoassist_servo_tools`)
- Unity bridge launch wrappers (`holoassist_unity_bridge`)
- A React evidence dashboard (`holoassist-dashboard`) that reads relay APIs

The default frontend entry point is the **Perception Monitor** page (`src/App.tsx -> PerceptionMonitorPage`), not the older mock robot dashboard page.

### Progress Status

- [x] `holo_assist_depth_tracker` publishes:
  - `/holo_assist_depth_tracker/debug_image`
  - `/holo_assist_depth_tracker/bbox`
  - `/holo_assist_depth_tracker/pointcloud`
  - `/holo_assist_depth_tracker/obstacle_marker`
- [x] Depth tracker includes morphology, connected components, temporal smoothing, and PointCloud2 projection
- [x] Dashboard relay node is implemented (`dashboard_relay_node.py`) with HTTP endpoints:
  - `GET /api/perception/status`
  - `GET /api/perception/debug.jpg`
- [x] Relay tracks perception + robot + Unity integration metrics (joint state freshness/rates, command rates, TF-based EEF pose, Unity TCP probe)
- [x] Dashboard defaults to `PerceptionMonitorPage` with tabs: Overview, Perception, Robot State, Unity Bridge
- [x] Rolling 60s evidence charts for bbox/obstacle publish rates and freshness/point-count context
- [x] Rosbag workflow docs and helper scripts are in place for perception evidence capture/replay
- [x] Click-to-plan node exists (`clicked_point_to_moveit`) and executes MoveIt plans from `/clicked_point`
- [x] Pose-to-twist bridge exists (`pose_to_twist_servo`) for `/servo_target_pose -> /servo_node/delta_twist_cmds`
- [x] Pointcloud-to-MoveIt collision object bridge exists (`pointcloud_to_moveit_obstacle`)
- [x] Combined Unity movement bringup launch exists (`unity_movement_bringup.launch.py`)
- [x] URSim + UR driver + MoveIt simulation runbook is documented (`ros2_ws/RS2_HOLOASSIST_SIM_BRINGUP.md`)
- [ ] Replace placeholder package metadata (`TODO` license/description) in:
  - `holoassist_servo_tools`
  - `holoassist_unity_bridge`
  - `holoassist_manipulation`
- [ ] Add robust target-frame transforms and orientation control in `pose_to_twist_servo.py` (currently position-only, assumes command frame)
- [ ] Improve clicked-point orientation and frame handling in `clicked_point_to_moveit.cpp` (currently identity orientation and planning-frame assumption)
- [ ] Full Unity headset/controller project is not stored in this branch; only ROS bridge-side packages are present

## Repository Layout

```text
HoloAssist/
  CLAUDE.md
  README.md
  ros2_ws/
    RS2_HOLOASSIST_SIM_BRINGUP.md
    PERCEPTION_ROSBAG_WORKFLOW.md
    PERCEPTION_PHASE4_VALIDATION.md
    scripts/
      perception_rosbag_record.sh
      perception_rosbag_play.sh
      rosbag_profiles/
    src/
      holo_assist_depth_tracker/
        config/
        launch/
        holo_assist_depth_tracker/
          depth_tracker_node.py
          dashboard_relay_node.py
          webcam_image_publisher_node.py
      holoassist_manipulation/
        launch/
        src/
          clicked_point_to_moveit.cpp
          pointcloud_to_moveit_obstacle.cpp
      holoassist_servo_tools/
        launch/
        holoassist_servo_tools/
          pose_to_twist_servo.py
      holoassist_unity_bridge/
        launch/
          tcp_endpoint.launch.py
          unity_movement_bringup.launch.py
  holoassist-dashboard/
    README.md
    src/
      App.tsx
      pages/
        PerceptionMonitorPage.tsx
        DashboardPage.tsx (legacy mock UI, not default)
```

## Development Environment

**Primary target environment:** Ubuntu 22.04 + ROS 2 Humble.  
**Frontend:** Node.js 18+ (`holoassist-dashboard`).

### Build ROS workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run dashboard frontend

```bash
cd holoassist-dashboard
npm install
npm run dev
```

Default URL: `http://localhost:5173`  
Default proxy: `/api/* -> http://127.0.0.1:8765`

Optional API override:

```bash
VITE_PERCEPTION_API_BASE=http://<relay-host>:8765 npm run dev
```

## Launch Sequences

### 1) Perception + RViz + Dashboard (RealSense path)

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch holo_assist_depth_tracker dashboard_bridge.launch.py bind_host:=0.0.0.0 bind_port:=8765
```

Terminal 3:

```bash
cd holoassist-dashboard
npm run dev
```

### 2) RGB fallback mode (no depth camera)

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch holo_assist_depth_tracker webcam_rgb_monitor.launch.py
```

This launches:

- `holo_assist_webcam_image_publisher`
- `holo_assist_depth_tracker_dashboard_relay` with RGB fallback enabled

### 3) URSim + MoveIt simulation flow

Follow `ros2_ws/RS2_HOLOASSIST_SIM_BRINGUP.md` for full runbook.  
Common command set:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# UR driver against URSim
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<URSIM_IP> launch_rviz:=false

# MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true

# Click-to-plan helper
ros2 launch holoassist_manipulation click_to_plan.launch.py move_group_name:=ur_manipulator planning_frame:=base_link tcp_frame:=tool0

# Pose -> Twist servo bridge
ros2 launch holoassist_servo_tools pose_to_twist_servo.launch.py command_frame:=base_link eef_frame:=tool0 target_topic:=/servo_target_pose twist_topic:=/servo_node/delta_twist_cmds
```

Optional obstacle bridge:

```bash
ros2 launch holoassist_manipulation pointcloud_to_moveit_obstacle.launch.py planning_frame:=base_link
```

Optional Unity-side ROS TCP endpoint bringup:

```bash
ros2 launch holoassist_unity_bridge unity_movement_bringup.launch.py
```

## Architecture

```text
Depth Camera (RealSense) or Webcam
  -> holo_assist_depth_tracker
      -> debug image, bbox, pointcloud, obstacle marker
          -> dashboard relay HTTP API
              -> holoassist-dashboard (Perception Monitor UI)

/clicked_point
  -> clicked_point_to_moveit
      -> MoveIt plan + execute

/servo_target_pose
  -> pose_to_twist_servo
      -> /servo_node/delta_twist_cmds
          -> MoveIt Servo / UR control path

/holo_assist_depth_tracker/pointcloud
  -> pointcloud_to_moveit_obstacle
      -> MoveIt planning scene collision object updates
```

## Key Topics and Endpoints

Perception:

- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`

Robot / command observability (relay subscriptions):

- `/joint_states`
- `/servo_target_pose`
- `/servo_node/delta_twist_cmds`
- `/clicked_point`
- TF (`command_frame` to `eef_frame`)
- `/unity/map_loaded`

Dashboard HTTP:

- `/api/perception/status`
- `/api/perception/debug.jpg`
- `/healthz`

## Known Issues and Constraints

- RealSense launch argument names can vary by `realsense2_camera` version; verify with:
  - `ros2 launch realsense2_camera rs_launch.py --show-args`
- `pose_to_twist_servo.py` currently:
  - assumes target pose is already in `command_frame`
  - controls linear velocity only (angular control intentionally left zero)
- `clicked_point_to_moveit.cpp` currently:
  - assumes clicked point is already in `planning_frame`
  - uses simple fixed orientation (identity quaternion)
- `pointcloud_to_moveit_obstacle` requires `move_group` availability to apply objects
- Dashboard relay `rgb_fallback` mode intentionally reports no depth capability (pointcloud/obstacle unsupported)
- `DashboardPage.tsx` is still mock-oriented; production-facing page is `PerceptionMonitorPage.tsx`
- Rosbag helper scripts are Bash scripts and are intended for Linux/WSL shells

## Working Conventions for Claude Code

- Prefer editing source packages in `ros2_ws/src/*` and avoid touching generated `build/`, `install/`, and `log/`.
- Keep topic names consistent with current defaults unless explicitly migrating both publishers and consumers.
- For UI work, preserve `PerceptionMonitorPage` as default unless requested otherwise.
- For evidence-related changes, keep bbox and obstacle marker rates as primary measured metrics (per `PERCEPTION_PHASE4_VALIDATION.md`).
- When changing launch defaults, update the corresponding runbooks in:
  - `ros2_ws/RS2_HOLOASSIST_SIM_BRINGUP.md`
  - `ros2_ws/PERCEPTION_ROSBAG_WORKFLOW.md`
  - `holoassist-dashboard/README.md`
