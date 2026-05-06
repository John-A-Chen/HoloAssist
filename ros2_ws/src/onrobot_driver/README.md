# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 driver for OnRobot Grippers.

Note: If you are using it with a Universal Robot eSeries, check out the [UR_OnRobot_ROS2](https://github.com/tonydle/UR_OnRobot_ROS2) package instead.

## Features
- ROS 2 driver for OnRobot grippers controlled via Modbus TCP or Serial
- Currently supported grippers:
    - [RG2](https://onrobot.com/en/products/rg2-gripper)
    - [RG6](https://onrobot.com/en/products/rg6-gripper) 
- ROS 2 Control hardware interface plugin

## Dependencies (included in the installation steps below)

- [onrobot_description](https://github.com/tonydle/OnRobot_ROS2_Description)
- libnet1-dev (for Modbus TCP/Serial)
- [Modbus](https://github.com/Mazurel/Modbus) C++ library (included as a submodule)

## Installation

1. Navigate to your ROS 2 workspace and **clone the repository** into the `src` directory:
   ```sh
   git clone --recurse-submodules https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
   ```
2. Install git dependencies using `vcs`:
   ```sh
   vcs import src --input src/onrobot_driver/required.repos
   ```
3. Install libnet:
   ```sh
   sudo apt install libnet1-dev
   ```
4. Build using colcon with symlink install:
   ```sh
   colcon build --symlink-install
   ```
5. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Usage
### Launch the driver
Launch the driver with `onrobot_type` [`rg2`,`rg6`] and `connection_type` [`serial` (UR Tool I/O) or `tcp` (Control Box)] arguments.
   ```sh
   ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=serial
   ```
Other arguments:
- `use_fake_hardware` (default: `false`): Use mock hardware interface for testing
- `launch_rviz` (default: `true`): Launch RViz with the gripper model
- `launch_rsp` (default: `true`): Launch the Robot State Publisher node (publishes to `/tf`)
- `device` (default: `/tmp/ttyUR`): Virtual Serial device path (if using Modbus Serial)
- `ip_address` (default: `192.168.1.1`): IP address of the Compute Box (if using Modbus TCP)
- `port` (default: `502`): Port of the Compute Box (if using Modbus TCP)

### Get the `finger_width` joint state (metres)
   ```sh
   ros2 topic echo /onrobot/joint_states
   ```
### Control the gripper with `finger_width_controller`(JointGroupPositionController)
   ```sh
   ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
   ```

## TO DOs
### Setting target force
At the moment the target force is set to be half of the maximum force. This can be changed in the `RG` class, but it would be better to set it as a parameter, or a service call.

### Implementing other controllers
A [Gripper Action Controller](https://control.ros.org/humble/doc/ros2_controllers/gripper_controllers/doc/userdoc.html) can be implemented to control the gripper with a `gripper_action_interface` and `GripperCommand` action. This will allow for more advanced control of the gripper, such as opening and closing with a specified force and monitoring the action state.

### Adding support for other grippers
The driver can be extended to support other OnRobot grippers, such as the [RG2-FT](https://onrobot.com/en/products/rg2-ft-gripper) and [3FG15](https://onrobot.com/en/products/3fg15-three-finger-gripper) grippers.

## Author
[Tony Le](https://github.com/tonydle)

## License
This software is released under the MIT License, see [LICENSE](./LICENSE).