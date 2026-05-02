# moveit_robot_control

`moveit_robot_control` is the ROS 2 package in this workspace that sits between MoveIt and the higher-level pick-and-place workflow.

Detailed integration + merge guide:

- [INTEGRATION_AND_MERGE.md](./INTEGRATION_AND_MERGE.md)

## Full Sim + MoveIt (single launch)

For the integrated fake-hardware stack (trolley + MoveIt robot + workspace + camera + truth/perceived cubes + target forwarding):

```bash
ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py
```

This launch starts:

- MoveIt bringup from `ur_onrobot_moveit_config`
- `workspace_scene_manager` (trolley mesh marker on `/workspace_scene/markers`)
- `holoassist_workspace_frame_tf` (`base_link -> workspace_frame` static TF)
- `coordinate_listener` with `require_robot_status:=false` by default
- sim truth/perception stack and selected-cube MoveIt target adapter
- one RViz session preconfigured for robot + TF + trolley + workspace/camera/cubes

Main scene-placement tuning file:

- `config/full_holoassist_sim.yaml`

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
- `/finger_width_trajectory_controller/joint_trajectory` - gripper commands
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

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
colcon build --packages-select moveit_robot_control --symlink-install
source install/setup.bash
```

## Typical run order

The important thing to remember is that `pick_place.launch.py` is not the whole robot stack by itself. It expects the robot driver, controllers, MoveIt, and the coordinate listener to already be available.

### Real robot with UR + OnRobot

Open a separate terminal for each long-running launch.

### Terminal 1 - Robot driver and gripper

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=<robot_ip> \
  gripper_target_force:=5.0
```

### Terminal 2 - MoveIt

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2
```

### Terminal 3 - Optional workspace scene

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
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
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash

ros2 launch moveit_robot_control_node coordinate_listener.launch.py \
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
cd /home/ollie/RS2_workspace/ros2_ws
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
cd /home/ollie/RS2_workspace/ros2_ws
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

### Fake hardware or simulation

For fake hardware testing, the key changes are:

- start the robot bringup with `use_fake_hardware:=true`
- start the coordinate listener with `require_robot_status:=false`

Example coordinate listener launch for fake hardware:

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
