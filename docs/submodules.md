# Current Submodules

This page replaces the old submodule conversion plan. The current checkout already has the active submodules recorded in `.gitmodules`.

## First-Party Packages

These packages are first-party HoloAssist code and should remain in the main repository:

| ROS package | Path | Role |
|---|---|---|
| `holoassist_perception` | `ros2_ws/src/HoloAssist_Perception` | Camera launch, AprilTag integration, debug overlay, trolley marker publisher, pick-place service bridge, selected-cube adapter, and custom perception interfaces. |
| `holoassist_movement` | `ros2_ws/src/HoloAssist_Movement` | MoveIt execution, workspace scene management, pick/place sequencing, and movement interface messages. |

## ROS Submodules

| Submodule path | Remote | Role |
|---|---|---|
| `ros2_ws/src/easy_handeye2` | `https://github.com/marcoesposito1988/easy_handeye2.git` | Eye-to-hand calibration GUI and message packages. |
| `ros2_ws/src/UR_OnRobot_ROS2` | `https://github.com/tonydle/UR_OnRobot_ROS2.git` | Combined UR + OnRobot launch, description, and MoveIt config. |
| `ros2_ws/src/ROS-TCP-Endpoint` | `https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git` | ROS side of Unity TCP bridge on port 10000. |
| `ros2_ws/src/onrobot_description` | `https://github.com/tonydle/OnRobot_ROS2_Description.git` | OnRobot RG2/RG6 descriptions. |
| `ros2_ws/src/onrobot_driver` | `https://github.com/tonydle/OnRobot_ROS2_Driver.git` | OnRobot gripper hardware driver. |

Do not document `Universal_Robots_ROS2_Driver` as a direct submodule in this checkout. The UR driver stack is consumed through apt/system dependencies and the `UR_OnRobot_ROS2` integration layer.

## Unity / ThirdParty Submodules

| Submodule path | Remote | Role |
|---|---|---|
| `ThirdParty/ROS-TCP-Connector` | `https://github.com/Unity-Technologies/ROS-TCP-Connector.git` | Unity-side ROS TCP connector and visualization packages. |
| `ThirdParty/URDF-Importer` | `https://github.com/Unity-Technologies/URDF-Importer.git` | Unity URDF importer package. |
| `ThirdParty/Q3toROS` | `https://github.com/tonydle/Q3toROS.git` | Quest-to-ROS helper code or examples. |

Unity consumes the ROS TCP Connector and URDF Importer through file dependencies in `Unity/My project/Packages/manifest.json`.

## Local Patch Status

`ros2_ws/src/UR_OnRobot_ROS2` is dirty by design in the current checkout. Local patches add `use_fake_gripper_hardware` support so the UR arm can connect through RTDE while the OnRobot gripper serial hardware is skipped.

Long term, commit those changes to a fork or store them as an explicit patch. Until then, clean clones or submodule resets need the fake-gripper patch reapplied.

## Apt / System Dependencies

The core ROS stack still comes from system packages:

- ROS 2 Humble
- MoveIt 2
- `apriltag_ros`
- `realsense2_camera`
- `rviz2`
- `tf2_ros`
- Universal Robots driver packages such as `ur_robot_driver`, `ur_controllers`, `ur_description`, and `ur_dashboard_msgs`
