# RViz Configurations

Two RViz configs are used depending on launch mode.

---

## 1. Perception RViz
**File:** `ros2_ws/src/holoassist/perception/config/depth_tracker_visualization.rviz`  
**Used when:** `--perception` flag (no `--moveit`)

**Fixed frame:** `base`

| Display | Topic / Source | Notes |
|---|---|---|
| Grid | — | XY plane, 0.1 m cells |
| April Cube 1 Marker | `/holoassist/perception/april_cube_1_marker` | Red cube overlay |
| April Cube 2 Marker | `/holoassist/perception/april_cube_2_marker` | Green |
| April Cube 3 Marker | `/holoassist/perception/april_cube_3_marker` | Blue |
| April Cube 4 Marker | `/holoassist/perception/april_cube_4_marker` | Yellow |
| TF | all frames | Robot + camera + detected tags |
| Tracker RGB Overlay | `/holoassist/perception/debug_image` | Camera image with tag detections drawn |
| Camera Color Raw | `/camera/camera/color/image_raw` | Disabled by default |

**TF tree shown:**
```
world
  └─ base_link
       ├─ base_link_inertia → shoulder → upper_arm → forearm → wrist_1 → wrist_2 → wrist_3
       │    └─ flange → tool0 → onrobot_base_link → gripper_tcp, fingers, etc.
       └─ camera_link → camera_color_frame → camera_color_optical_frame
            ├─ apriltag_cube_1..4   (cube centre frames)
            └─ tag36h11:1, :10, :21, :22, :23, :28, :31   (individual tag frames)
```

---

## 2. Robot / MoveIt RViz
**File:** `ros2_ws/src/ur_onrobot/ur_onrobot_description/rviz/view_robot.rviz`  
**Used when:** `--moveit` flag or driver-only (no `--perception`)

Loads the full robot model with MotionPlanning plugin. Used by MoveIt to show:
- Robot model (UR3e + RG2)
- Planning scene (trolley mesh, collision objects)
- Interactive motion planning markers

---

## Switching Between Configs

`launch.py` automatically picks the right RViz instance:

| Flags | RViz owner | Config used |
|---|---|---|
| Neither `--moveit` nor `--perception` | UR driver | `view_robot.rviz` |
| `--perception` only | Perception launch | `depth_tracker_visualization.rviz` |
| `--moveit` only | MoveIt launch | `view_robot.rviz` |
| Both `--perception` and `--moveit` | Perception launch | `depth_tracker_visualization.rviz` |

Only one RViz window opens regardless of flags.

---

## Manual RViz Launch

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash

# Perception view
rviz2 -d ros2_ws/src/holoassist/perception/config/depth_tracker_visualization.rviz

# Robot/MoveIt view
rviz2 -d ros2_ws/install/ur_onrobot_description/share/ur_onrobot_description/rviz/view_robot.rviz
```
