# HoloAssist RViz Digital Twin Notes

## Assets Found In `worlds/`

Current assets discovered and reused:

- `holoassist_scene.urdf` (legacy, plain URDF)
- `holoassist_trolley_board.world` (Gazebo world)
- `UR3eTrolley(1).dae` (large trolley/assembly mesh)
- `TrolleyAssemAsAPartv2(1).STEP` (CAD source)
- `1cube.3mf.obj`, `2cube.3mf.obj`, `3cube.3mf.obj`, `4cube.3mf.obj`
- `tag36_11_00000.png`, `tag36_11_00001.png`, `tag36_11_00002.png`, `tag36_11_00003.png`

New organized sim assets:

- `worlds/urdf/holoassist_scene.urdf.xacro`
- `worlds/meshes/apriltags/tag_0.dae` ... `tag_3.dae`
- `worlds/textures/apriltags/tag36_11_00000.png` ... `tag36_11_00003.png`
- `worlds/rviz/holoassist_scene.rviz`
- `worlds/launch/view_holoassist_scene.launch.py`

## Textured Board And AprilTag Visual Plates

- Board size: `0.700 m x 0.500 m x 0.010 m`.
- Board color: black/dark grey.
- Tag visual size: `0.032 m x 0.032 m`.
- Tag center offset: `0.016 m` from each board edge (flush-corner layout).
- Tag plate height: `z=0.006 m` relative to `workspace_frame` (1 mm above board top at `z=0.005`).

Tag centres:

- `tag0_visual`: `x=-0.334 y=-0.234 z=0.006`
- `tag1_visual`: `x=+0.334 y=-0.234 z=0.006`
- `tag2_visual`: `x=-0.334 y=+0.234 z=0.006`
- `tag3_visual`: `x=+0.334 y=+0.234 z=0.006`

Important: these tags are RViz visuals only. They are cosmetic in RViz and are not camera detections by themselves.

## Frame Tree

The digital-twin URDF frame structure is:

- `world`
  - `workspace_frame`
    - `board_link`
    - `tag0_visual`
    - `tag1_visual`
    - `tag2_visual`
    - `tag3_visual`
    - `ur3e_base_link0`
    - `camera_link`
      - `camera_color_optical_frame`
    - optional `april_cube_1..4`

## UR3e Base Placement

For the scene, the base marker is centered between back tags:

- `x=0.000`
- `y=0.314`
- `z=-0.015`

Diameter visual: `0.128 m`.

## Camera Optical Frame Convention

`camera_color_optical_frame` uses ROS optical convention:

- `+Z` forward
- `+X` right
- `+Y` down

## Validation Commands

Generate and validate URDF:

```bash
source /opt/ros/humble/setup.bash
cd /home/john/git/RS2-HoloAssist/john/ros2_ws/src/holo_assist_depth_tracker
ros2 run xacro xacro worlds/urdf/holoassist_scene.urdf.xacro > /tmp/holoassist_scene.urdf
check_urdf /tmp/holoassist_scene.urdf
```

Launch from package launch (recommended):

```bash
cd /home/john/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holo_assist_depth_tracker --symlink-install
source install/setup.bash
ros2 launch holo_assist_depth_tracker view_holoassist_scene.launch.py
```

Launch directly from `worlds/` launch file:

```bash
cd /home/john/git/RS2-HoloAssist/john/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch /home/john/git/RS2-HoloAssist/john/ros2_ws/src/holo_assist_depth_tracker/worlds/launch/view_holoassist_scene.launch.py
```

TF spot checks:

```bash
ros2 run tf2_ros tf2_echo workspace_frame board_link
ros2 run tf2_ros tf2_echo workspace_frame tag0_visual
ros2 run tf2_ros tf2_echo workspace_frame tag1_visual
ros2 run tf2_ros tf2_echo workspace_frame tag2_visual
ros2 run tf2_ros tf2_echo workspace_frame tag3_visual
ros2 run tf2_ros tf2_echo workspace_frame ur3e_base_link0
```
