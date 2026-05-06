# OnRobot_ROS2_Description

This package provides a ROS2 URDF description for OnRobot grippers. It uses XACRO macros to generate the URDF and includes a sample launch file to visualise the gripper.

## Installation

1. Navigate to your workspace and **clone the repository** into the `src` directory:

   ```sh
   git clone https://github.com/tonydle/OnRobot_ROS2_Description.git src/onrobot_description
   ```
2. Build using colcon with symlink install:

   ```sh
   colcon build --symlink-install
   ```
3. Source the workspace:

   ```sh
   source install/setup.bash
   ```

## Quick Test
To quickly test the visualisation of the OnRobot gripper model, run the provided launch file:
```sh
ros2 launch onrobot_description view_onrobot.launch.py onrobot_type:=rg2
``` 
This launches RViz2 with the URDF loaded.

## How To Use onrobot_macro.xacro
The urdf/onrobot_macro.xacro builds the OnRobot model based on the chosen `onrobot_type`. You can integrate this macro into your own robot's URDF by including it and then attaching the `onrobot_base_link` where needed.

An example is given in urdf/onrobot.urdf.xacro where the macro is used to create an instance of the OnRobot gripper that is attached to world using a fixed joint:

```xml
<xacro:include filename="$(find onrobot_description)/urdf/onrobot_macro.xacro"/>
<xacro:onrobot onrobot_type="$(arg onrobot_type)" prefix="$(arg prefix)"/>
...
<link name="world" />
<joint name="$(arg prefix)onrobot_base_link_joint" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link="world"/>
  <child link="$(arg prefix)onrobot_base_link"/>
</joint>
```

## Author
[Tony Le](https://github.com/tonydle)

## License
This software is released under the MIT License, see [LICENSE](./LICENSE).

The original mesh files and xacros are derived from [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot), which is licensed under the MIT License.