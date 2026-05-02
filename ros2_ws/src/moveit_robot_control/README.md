# moveit_robot_control

`moveit_robot_control` is the ROS 2 package in this workspace that sits between MoveIt and the higher-level pick-and-place workflow.

It provides three main pieces:

1. `coordinate_listener`
   Accepts coordinate goals on ROS topics, plans motion with MoveIt, and executes trajectories on the robot.
2. `pick_place_sequencer`
   Runs the pick-and-place sequence by publishing motion goals and gripper commands.
3. `workspace_scene_manager`
   Publishes table/block markers and optional MoveIt collision objects for the workspace.

## What each node does

### 1. Coordinate listener

Node name:

- `moveit_robot_control`

Executable:

- `coordinate_listener`

Purpose:

- Accepts target positions and poses from topics
- Plans a Cartesian path first when possible
- Falls back to MoveIt pose-goal planning when the straight-line route fails
- In `orientation_mode:=auto`, samples several wrist orientations and, if needed, does a final free-orientation position-only fallback so MoveIt can choose a reachable wrist orientation
- Rejects predicted UR flange-to-forearm clamp routes before execution when `avoid_flange_forearm_clamp:=true`
- Publishes human-readable status plus machine-readable state/debug topics

Input topics:

- `/moveit_robot_control/target_point` - `geometry_msgs/msg/Point`
- `/moveit_robot_control/target_pose` - `geometry_msgs/msg/Pose`
- `/moveit_robot_control/target` - `moveit_robot_control_msgs/msg/TargetRPY`

Output topics:

- `/moveit_robot_control/status` - plain text status
- `/moveit_robot_control/state` - simple lifecycle state such as `READY`, `PLANNING`, `EXECUTING`, `COMPLETE`, `FAILED`
- `/moveit_robot_control/debug` - JSON payload with detailed facts
- `/moveit_robot_control/complete` - completion message when a goal finishes

### 2. Pick-place sequencer

Node name:

- `pick_place_sequencer`

Executable:

- `pick_place_sequencer`

Purpose:

- Waits for a block pose or a JSON command
- Computes the approach, grasp, lift, place-above, and optional place-down poses
- Commands the gripper open/close actions
- Publishes high-level progress updates for each pick-place step
- Uses the bin pose configuration file as the source of truth for bin locations

Input topics:

- `/pick_place/block_pose` - `geometry_msgs/msg/PoseStamped`
- `/pick_place/command` - `std_msgs/msg/String` containing JSON
- `/pick_place/mode` - `std_msgs/msg/String` with `run` or `stop`

Output topics:

- `/pick_place/status` - JSON string describing the current step and target coordinates
- `/moveit_robot_control/target_point` or `/moveit_robot_control/target_pose` - motion goals sent to the coordinate listener
- `/finger_width_trajectory_controller/follow_joint_trajectory` - gripper command action
- `/workspace_scene/command` - optional block add/remove scene updates

### 3. Workspace scene manager

Node name:

- `workspace_scene_manager`

Executable:

- `workspace_scene_manager`

Purpose:

- Publishes the trolley/table mesh into RViz
- Optionally adds a table collision object to MoveIt
- Adds, removes, and clears block collision objects and matching markers

Input topics:

- `/workspace_scene/command` - `std_msgs/msg/String` containing JSON commands
- `/workspace_scene/spawn_block_pose` - `geometry_msgs/msg/PoseStamped`

Output topics:

- `/workspace_scene/markers`
- `/workspace_scene/status`

## Launch files in this package

- `launch/coordinate_listener.launch.py`
  Starts the coordinate listener node and exposes planning/execution parameters.

- `launch/pick_place.launch.py`
  Starts the pick-place sequencer and exposes placement/bin/gripper settings.

- `launch/pick_place_system.launch.py`
  Starts `workspace_scene_manager`, `coordinate_listener`, and `pick_place_sequencer` together.

- `launch/workspace_scene.launch.py`
  Starts the workspace scene manager.

For a UR + OnRobot setup like this workspace, the usual MoveIt bringup is:

- `ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2`

## Files used by the current workflow

If you want to send only the files used by the current UR + OnRobot pick-place flow to main, these are the ones to focus on:

- `README.md`
- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/moveit_robot_control`
- `config/bin_poses.json`
- `launch/coordinate_listener.launch.py`
- `launch/pick_place.launch.py`
- `launch/pick_place_system.launch.py`
- `launch/workspace_scene.launch.py`
- `moveit_robot_control_node/moveit_robot_control.py`
- `moveit_robot_control_node/pick_place_sequencer.py`
- `moveit_robot_control_node/workspace_scene_manager.py`
- `moveit_robot_control_node/__init__.py`
- `meshes/UR3eTrolley_decimated.dae`

## Archived files

Files not part of the current workflow are stored in:

- `old_files/`

Right now that archived set is:

- `old_files/launch/ur_moveit.launch.py`
  Legacy generic UR MoveIt launch for older or plain-UR workflows.

## Bin configuration

The bin positions now live in one place:

- [config/bin_poses.json](./config/bin_poses.json)

Edit that file to change the default bin locations used by the pick-place sequencer.

Current default bins:

```json
{
  "bin_1": {"xyz": [-0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_2": {"xyz": [-0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_3": {"xyz": [0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
  "bin_4": {"xyz": [0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]}
}
```

You can also point the sequencer at a different file with:

```bash
bin_config_path:=/full/path/to/bin_poses.json
```

## Build

From the workspace root, build this package with:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
colcon build --packages-up-to moveit_robot_control --symlink-install
source install/setup.bash
```

For the full UR + OnRobot run stack, build the robot control package, the MoveIt config package, and their local dependencies too:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
colcon build --packages-up-to moveit_robot_control ur_onrobot_control ur_onrobot_moveit_config --symlink-install
source install/setup.bash
```

ROS 2 launch commands only see packages that have been built into `install/` and then sourced. A package folder existing under `src/` is not enough by itself.

## Typical run order

The important thing to remember is that `pick_place.launch.py` is not the whole robot stack by itself. It expects the robot driver, controllers, MoveIt, and the coordinate listener to already be available.

Open a separate terminal for each long-running launch. Choose one Terminal 1 robot bringup mode, then continue with Terminal 2 onward.

### Terminal 1A - Real robot with UR + OnRobot

Use this when you are connected to the physical UR robot and the physical OnRobot gripper.

For the real gripper, the UR teach pendant must have the RS485/tool communication forwarder set up and running so the robot accepts connections on TCP port `54321`. The driver exposes that tool connection locally as `/tmp/ttyUR` for the OnRobot serial driver.

You can check the tool bridge with:

```bash
nc -vz <robot_ip> 54321
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=<robot_ip> \
  launch_tool_communication:=true \
  gripper_target_force:=5.0
```

### Terminal 1B - Sim robot or URSim with fake gripper

Use this when you are running against URSim/Polyscope simulation, or when the UR robot is reachable but the OnRobot RS485 tool bridge is not available.

In this mode the UR side still connects to `robot_ip`, but the OnRobot gripper is fake and tool communication is disabled. This avoids errors such as `Connection refused` on `192.168.56.101:54321` and `Cannot open serial port /tmp/ttyUR`.

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=192.168.56.101 \
  onrobot_use_fake_hardware:=true \
  launch_tool_communication:=false
```

### Terminal 1C - Pure fake hardware, no robot connection

Use this when you want a local ROS-only test with no physical robot and no URSim connection.

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  use_fake_hardware:=true \
  onrobot_use_fake_hardware:=true \
  launch_tool_communication:=false
```

### Terminal 2 - MoveIt

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2
```

### Terminal 3 - Optional workspace scene

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch moveit_robot_control workspace_scene.launch.py \
  frame_id:=base_link \
  publish_table_mesh:=true \
  apply_table_collision:=false
```

### Terminal 4 - Coordinate listener

Use the UR + OnRobot planning group and TCP:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch moveit_robot_control coordinate_listener.launch.py \
  move_group_name:=ur_onrobot_manipulator \
  ee_link:=gripper_tcp \
  frame:=base_link \
  allow_pose_goal_fallback:=true \
  orientation_mode:=auto \
  avoid_flange_forearm_clamp:=true
```

### Terminal 5 - Pick and place

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch moveit_robot_control pick_place.launch.py \
  frame_id:=base_link \
  initial_mode:=run \
  orientation_mode:=auto \
  block_id:=block_1
```

### One-command version of those three launches

If you want the workspace scene, coordinate listener, and pick-place sequencer together, use:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
source install/setup.bash

ros2 launch moveit_robot_control pick_place_system.launch.py \
  frame_id:=base_link \
  publish_table_mesh:=true \
  apply_table_collision:=false \
  move_group_name:=ur_onrobot_manipulator \
  ee_link:=gripper_tcp \
  frame:=base_link \
  allow_pose_goal_fallback:=true \
  orientation_mode:=auto \
  avoid_flange_forearm_clamp:=true \
  initial_mode:=run \
  block_id:=block_1
```

This combined launch does not start the robot driver or MoveIt. Those still need to be running already.

If you start the sequencer with `initial_mode:=stop`, it will queue the request but will not move until you publish:

```bash
ros2 topic pub --once /pick_place/mode std_msgs/msg/String "{data: run}"
```

### Coordinate listener for fake hardware

For pure fake hardware testing, start the coordinate listener with `require_robot_status:=false`:

```bash
ros2 launch moveit_robot_control coordinate_listener.launch.py \
  move_group_name:=ur_onrobot_manipulator \
  ee_link:=gripper_tcp \
  frame:=base_link \
  require_robot_status:=false \
  allow_pose_goal_fallback:=true \
  orientation_mode:=auto
```

## Quick tests

### Move the robot to a single XYZ point

```bash
ros2 topic pub --once /moveit_robot_control/target_point geometry_msgs/msg/Point \
"{x: 0.20, y: 0.30, z: 0.10}"
```

### Move the robot to a full pose

```bash
ros2 topic pub --once /moveit_robot_control/target_pose geometry_msgs/msg/Pose \
"{position: {x: 0.20, y: 0.30, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}"
```

### Send a pick-place request using a block pose topic

This uses the default destination from the sequencer configuration.

```bash
ros2 topic pub --once /pick_place/block_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.20, y: 0.30, z: 0.10}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Send a pick-place request to a specific bin

```bash
ros2 topic pub --once /pick_place/command std_msgs/msg/String \
"{data: '{\"block_id\":\"block_1\",\"frame_id\":\"base_link\",\"block_pose\":{\"x\":0.20,\"y\":0.30,\"z\":0.10},\"bin_id\":\"bin_3\"}'}"
```

### Spawn a block in the workspace scene

```bash
ros2 topic pub --once /workspace_scene/command std_msgs/msg/String \
"{data: '{\"action\":\"add_block\",\"id\":\"block_1\",\"frame_id\":\"base_link\",\"x\":0.20,\"y\":0.30,\"z\":0.10,\"size\":[0.05,0.05,0.05],\"z_mode\":\"bottom\"}'}"
```

### Remove a block from the workspace scene

```bash
ros2 topic pub --once /workspace_scene/command std_msgs/msg/String \
"{data: '{\"action\":\"remove_block\",\"id\":\"block_1\"}'}"
```

## Recommended monitoring topics

### High-level pick-place progress

```bash
ros2 topic echo /pick_place/status
```

This is the best topic to watch when you want to know:

- what step the sequencer is on
- which bin it is targeting
- which `x/y/z` it is moving toward at that step

### Motion planner/executor status

```bash
ros2 topic echo /moveit_robot_control/status
ros2 topic echo /moveit_robot_control/state
ros2 topic echo /moveit_robot_control/debug
```

## Important parameters

### Coordinate listener

- `move_group_name`
- `ee_link`
- `frame`
- `orientation_mode` - `auto`, `current`, or `fixed`
- `allow_pose_goal_fallback`
- `avoid_flange_forearm_clamp`
- `velocity_scale`
- `joint_goal_tolerance`

### Pick-place sequencer

- `bin_config_path`
- `default_bin_id`
- `item_bin_map`
- `pregrasp_z_offset`
- `grasp_z_offset`
- `place_above_z_offset`
- `place_z_offset`
- `place_descent_enabled`
- `initial_mode`
- `open_width`
- `close_width`
- `gripper_action_name`

### Workspace scene manager

- `publish_table_mesh`
- `apply_table_collision`
- `table_collision_xyz`
- `table_collision_size`

## How the pieces fit together

The normal flow is:

1. A block pose arrives on `/pick_place/block_pose` or a JSON command arrives on `/pick_place/command`
2. `pick_place_sequencer` chooses the destination pose or bin
3. The sequencer generates approach and place poses
4. The sequencer publishes motion goals to the coordinate listener
5. `coordinate_listener` plans with MoveIt and sends the final joint trajectory to the controller
6. The sequencer opens and closes the gripper at the right steps
7. The sequencer optionally removes and re-adds the block in the planning scene

## Troubleshooting

### `Package 'ur_onrobot_control' not found`

If the files are visible under `src/ur_onrobot/ur_onrobot_control` but `ros2 launch` cannot find the package, the package has not been built into the active ROS environment yet.

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/git/RS2/main/HoloAssist/ros2_ws
colcon build --packages-up-to ur_onrobot_control --symlink-install
source install/setup.bash
```

Then check:

```bash
ros2 pkg list | grep ur_onrobot_control
```

If the next launch reports `Package 'ur_onrobot_moveit_config' not found`, build the full UR + OnRobot stack from the Build section instead.

### `Cannot open serial port /tmp/ttyUR` or `Connection refused`

This means the OnRobot gripper serial bridge did not come up. The launch creates `/tmp/ttyUR` by connecting to TCP port `54321` on the UR robot.

For the real robot, check that the teach pendant has the RS485/tool communication forwarder installed and running, then check:

```bash
nc -vz <robot_ip> 54321
```

For URSim or any setup without the real OnRobot tool bridge, use the sim robot command from Terminal 1B:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=192.168.56.101 \
  onrobot_use_fake_hardware:=true \
  launch_tool_communication:=false
```

### Gripper commands run but the gripper does not move

The pick-place sequencer sends gripper commands through:

```bash
/finger_width_trajectory_controller/follow_joint_trajectory
```

Check that the gripper controller is active:

```bash
ros2 control list_controllers | grep finger_width
```

If `finger_width_trajectory_controller` is `inactive`, activate it:

```bash
ros2 control switch_controllers \
  --activate finger_width_trajectory_controller \
  --activate-asap
```

For sim/fake gripper runs, this changes the `finger_width` joint state/RViz model only. It will not move a physical gripper unless the real OnRobot hardware and tool bridge are connected.

### `pick_place.launch.py` starts but nothing moves

Check these first:

- Is the coordinate listener running?
- Is MoveIt running?
- Is the robot driver running?
- Did you launch the sequencer with `initial_mode:=stop`?

### The robot waits forever for MoveIt services

MoveIt is not up, or the wrong MoveIt stack is running.

For the UR + OnRobot setup, use:

```bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
```

### The sequencer goes to the wrong bin

Check:

- [config/bin_poses.json](./config/bin_poses.json)
- any custom `bin_config_path:=...`
- any `default_bin_id`
- any `item_bin_map`

### A target point is reachable in position but fails in orientation

Use:

- `orientation_mode:=auto`
- `allow_pose_goal_fallback:=true`

In the current code, `auto` will try sampled orientations first and then fall back to free-orientation position planning if needed.

### The robot trips the UR flange/forearm protective stop

Keep:

- `avoid_flange_forearm_clamp:=true`

This package now performs a planner-side clamp-zone check and tries to reject risky routes before execution, but it still depends on a reasonable robot model, valid TF, and safe target poses.
