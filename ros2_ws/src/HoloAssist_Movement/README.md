# holoassist_movement

`holoassist_movement` is the HoloAssist motion-control package. It brings together the UR3e + OnRobot MoveIt stack, the workspace/trolley scene, fake-hardware simulation, and the pick-and-place sequencer.

Use this README for day-to-day running and debugging. Deeper implementation notes live in:

- [docs/MOTION_EXECUTION_REFERENCE.md](./docs/MOTION_EXECUTION_REFERENCE.md)
- [INTEGRATION_AND_MERGE.md](./INTEGRATION_AND_MERGE.md)
- [../../docs/POSE_HANDOFF_CONTRACT.md](../../docs/POSE_HANDOFF_CONTRACT.md)
- [../../docs/J0HN_MERGED_ARCHITECTURE.md](../../docs/J0HN_MERGED_ARCHITECTURE.md)

## Before Running

Always source ROS and this workspace from the workspace root:

```bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Only run one robot stack at a time. Do not run the fake sim and `ur_robot_driver` or `full_holoassist_hardware.launch.py` together; they share ROS names such as `/controller_manager` and will break each other.

If the graph is confused, stop old launches with `Ctrl+C`. If required:

```bash
pkill -f "full_holoassist_moveit_sim.launch.py"
pkill -f "full_holoassist_hardware.launch.py"
pkill -f "ros2 launch ur_robot_driver"
pkill -f "ros2_control_node"
pkill -f "ur_ros2_control_node"
pkill -f "move_group"
pkill -f "rviz2"
```

## Fake MoveIt Sim

This is the normal development command. It uses fake hardware and does not need a robot IP.

```bash
ros2 launch holoassist_movement full_holoassist_moveit_sim.launch.py \
  use_rviz:=true \
  start_pick_place:=true
```

This launches:

- fake UR3e + OnRobot `ros2_control`
- `joint_state_broadcaster`, `joint_trajectory_controller`, and `finger_width_trajectory_controller`
- MoveIt `move_group`
- workspace/trolley scene
- simulated cube truth/perception
- cube-to-MoveIt planning-scene bridge
- pick/place sequencer and `/holoassist/pick_cube_to_bin` service
- one RViz session

No `robot_ip` argument is used in this mode. The fake sim default is:

```bash
robot_base_yaw_rad:=0.0
```

The sim bench/workspace is already world-fixed in [config/full_holoassist_sim.yaml](./config/full_holoassist_sim.yaml).

## Pick Up a Cube in Sim

With the fake sim running, open a second terminal, source the workspace, then call:

```bash
ros2 service call /holoassist/pick_cube_to_bin \
  holoassist_perception/srv/PickCubeToBin \
  "{cube_name: 'april_cube_1', bin_id: 'bin_1'}"
```

Valid cubes:

```text
april_cube_1
april_cube_2
april_cube_3
april_cube_4
```

Valid bins:

```text
bin_1
bin_2
bin_3
bin_4
```

Watch progress with:

```bash
ros2 topic echo /pick_place/status
```

The sequence is:

1. move to the configured home joint pose
2. move above the cube
3. descend to the cube
4. close the gripper
5. lift
6. move above the bin
7. descend to place
8. open the gripper
9. return home

The current home pose is configured in `pick_place_sequencer.py`:

```text
Base      -75 deg
Shoulder  -90 deg
Elbow     -50 deg
Wrist 1  -120 deg
Wrist 2    90 deg
Wrist 3     0 deg
```

## URSim or Real Robot

For URSim Docker or the real robot, use the hardware launch. This mode needs `robot_ip`.

For the URSim started with:

```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

the robot IP is normally `192.168.56.101`:

```bash
ros2 launch holoassist_movement full_holoassist_hardware.launch.py \
  robot_ip:=192.168.56.101 \
  start_camera:=false
```

For the real robot, replace the IP and normally leave the camera enabled:

```bash
ros2 launch holoassist_movement full_holoassist_hardware.launch.py \
  robot_ip:=<robot_ip>
```

Hardware defaults:

```bash
robot_base_yaw_rad:=3.14159
velocity_scale:=0.05
use_rviz:=true
start_pick_place:=true
```

`robot_base_yaw_rad` rotates the robot mounting frame in the URDF. It does not change the real robot's joint values.

## URSim With World-Fixed Bench

For URSim testing without the real camera stack:

```bash
ros2 launch holoassist_movement full_holoassist_hardware.launch.py \
  robot_ip:=192.168.56.101 \
  robot_base_yaw_rad:=0.0 \
  hw_config:=/home/ollie/git/RS2/main/HoloAssist/ros2_ws/src/HoloAssist_Movement/config/full_holoassist_ursim.yaml \
  start_camera:=false \
  start_tracker:=false \
  start_overlay:=false \
  start_pick_place:=false \
  start_rosbridge:=false \
  use_calibrated_workspace:=false
```

## Build

Build this package after editing launch files, Python nodes, configs, or RViz files:

```bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select holoassist_movement --symlink-install
source install/setup.bash
```

If you edit the sim bridge in `holoassist_perception`, build that package too:

```bash
colcon build --packages-select holoassist_perception --symlink-install
source install/setup.bash
```

## Launch Files

Main launch files:

- `launch/full_holoassist_moveit_sim.launch.py`
  Fake-hardware MoveIt sim. Best for motion and pick/place development. No IP needed.

- `launch/full_holoassist_hardware.launch.py`
  UR driver based launch for URSim or the real robot. Requires `robot_ip`.

- `launch/full_holoassist_gazebo_sim.launch.py`
  Gazebo-based sim entry point.

- `launch/coordinate_listener.launch.py`
  Starts only the MoveIt topic controller.

- `launch/pick_place.launch.py`
  Starts only the pick/place sequencer.

- `launch/pick_place_system.launch.py`
  Starts workspace scene, coordinate listener, and pick/place sequencer, but not the robot driver or MoveIt.

- `launch/workspace_scene.launch.py`
  Starts only the workspace/trolley scene manager.

## Nodes And Topics

### Coordinate Listener

Executable:

```text
coordinate_listener
```

Node name:

```text
holoassist_movement
```

Inputs:

- `/holoassist/movement/target_point` - `geometry_msgs/msg/Point`
- `/holoassist/movement/target_pose` - `geometry_msgs/msg/Pose`
- `/holoassist/movement/target` - `holoassist_movement/msg/TargetRPY`
- `/holoassist/movement/target_joint_state` - `sensor_msgs/msg/JointState`

Outputs:

- `/holoassist/movement/status`
- `/holoassist/movement/state`
- `/holoassist/movement/debug`
- `/holoassist/movement/complete`

Important parameters:

- `move_group_name`
- `ee_link`
- `frame`
- `trajectory_topic`
- `require_robot_status`
- `require_controller_check`
- `velocity_scale`
- `orientation_mode`
- `allow_pose_goal_fallback`
- `avoid_flange_forearm_clamp`

### Pick-Place Sequencer

Executable:

```text
pick_place_sequencer
```

Inputs:

- `/pick_place/block_pose` - `geometry_msgs/msg/PoseStamped`
- `/pick_place/command` - JSON in `std_msgs/msg/String`
- `/pick_place/mode` - `run`, `stop`, or `pause`

Outputs:

- `/pick_place/status`
- `/holoassist/movement/target_point`
- `/holoassist/movement/target_pose`
- `/holoassist/movement/target_joint_state`
- `/finger_width_trajectory_controller/joint_trajectory`
- `/workspace_scene/command`

### Workspace Scene Manager

Executable:

```text
workspace_scene_manager
```

Purpose:

- publishes the trolley/table mesh for RViz
- optionally applies table collision to MoveIt
- adds/removes block collision objects and markers

Topics:

- `/workspace_scene/command`
- `/workspace_scene/spawn_block_pose`
- `/workspace_scene/markers`
- `/workspace_scene/status`

## Configuration Files

Useful files in this package:

- [config/full_holoassist_sim.yaml](./config/full_holoassist_sim.yaml)
  Fake sim workspace, trolley, and selected-cube target offsets.

- [config/full_holoassist_hw.yaml](./config/full_holoassist_hw.yaml)
  Hardware workspace/trolley settings.

- [config/full_holoassist_ursim.yaml](./config/full_holoassist_ursim.yaml)
  URSim-specific world-fixed bench settings.

- [config/bin_poses.json](./config/bin_poses.json)
  Default bin locations used by the pick/place sequencer.

- [config/sim_controllers.yaml](./config/sim_controllers.yaml)
  Controllers used by the fake-hardware sim.

- [rviz/holoassist_hw.rviz](./rviz/holoassist_hw.rviz)
  Hardware/URSim RViz config.

## Bin Configuration

Bin poses are in [config/bin_poses.json](./config/bin_poses.json):

```json
{
  "bin_1": {"xyz": [-0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_2": {"xyz": [-0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_3": {"xyz": [0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_4": {"xyz": [0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]}
}
```

To use another bin file:

```bash
ros2 launch holoassist_movement pick_place.launch.py \
  bin_config_path:=/full/path/to/bin_poses.json
```

## Quick Manual Tests

Move to one XYZ point:

```bash
ros2 topic pub --once /holoassist/movement/target_point \
  geometry_msgs/msg/Point "{x: 0.20, y: 0.30, z: 0.10}"
```

Move to a full pose:

```bash
ros2 topic pub --once /holoassist/movement/target_pose \
  geometry_msgs/msg/Pose \
  "{position: {x: 0.20, y: 0.30, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}"
```

Move to the pick/place home joint pose:

```bash
ros2 topic pub --once /holoassist/movement/target_joint_state \
  sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], position: [-1.308997, -1.570796, -0.872665, -2.094395, 1.570796, 0.0]}"
```

Send a JSON pick/place command directly:

```bash
ros2 topic pub --once /pick_place/command std_msgs/msg/String \
  "{data: '{\"block_id\":\"block_1\",\"frame_id\":\"base_link\",\"block_pose\":{\"x\":0.20,\"y\":0.30,\"z\":0.02},\"bin_id\":\"bin_3\"}'}"
```

Add a visual/collision block to the workspace scene:

```bash
ros2 topic pub --once /workspace_scene/command std_msgs/msg/String \
  "{data: '{\"action\":\"add_block\",\"id\":\"block_1\",\"frame_id\":\"base_link\",\"x\":0.20,\"y\":0.30,\"z\":0.10,\"size\":[0.05,0.05,0.05],\"z_mode\":\"bottom\"}'}"
```

Remove a block:

```bash
ros2 topic pub --once /workspace_scene/command std_msgs/msg/String \
  "{data: '{\"action\":\"remove_block\",\"id\":\"block_1\"}'}"
```

## Checks and Troubleshooting

Check that fake sim controllers are active:

```bash
ros2 control list_controllers
```

Expected active controllers in fake sim:

```text
joint_state_broadcaster
joint_trajectory_controller
finger_width_trajectory_controller
```

Check joint states:

```bash
ros2 topic echo /joint_states --once
```

Check robot TF:

```bash
ros2 run tf2_ros tf2_echo world base_link
```

Check pick/place progress:

```bash
ros2 topic echo /pick_place/status
```

Check MoveIt state:

```bash
ros2 topic echo /holoassist/movement/state
ros2 topic echo /holoassist/movement/status
ros2 topic echo /holoassist/movement/debug
```

Common issues:

- RViz does not open:
  Use `use_rviz:=true`. The sim intentionally disables nested RViz instances and launches only one top-level RViz.

- The robot is missing in RViz:
  Check `/joint_states` and `ros2 control list_controllers`. This usually means another UR driver stack is still running and clashing with `/controller_manager`.

- The launch says `use_rviz=false` even though you passed `true`:
  Rebuild `holoassist_movement`; launch files run from `install/`.

- A cube collision blocks the lift after grasp:
  Restart after rebuilding `holoassist_perception`. The sim bridge suppresses the carried cube when the sequencer removes it from the planning scene.

- The robot faces the wrong way:
  For fake sim, leave `robot_base_yaw_rad` at its default `0.0`. For hardware/URSim, the hardware launch defaults to `3.14159`. This changes the URDF mounting frame, not the real joint values.

- Nothing moves after sending a pick command:
  Make sure `/holoassist/pick_cube_to_bin` exists, `pick_place_sequencer` is running, and the mode is `run`.

```bash
ros2 service list | rg /holoassist/pick_cube_to_bin
ros2 topic pub --once /pick_place/mode std_msgs/msg/String "{data: run}"
```
