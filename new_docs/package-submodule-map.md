# Package and Submodule Map

This file separates first-party HoloAssist code from upstream submodules and apt/system dependencies.

## First-Party ROS Packages

| ROS package | Path | Current role | Website replacement |
|---|---|---|---|
| `holoassist_perception` | `ros2_ws/src/HoloAssist_Perception` | Camera launch, AprilTag integration, debug overlay, trolley marker publisher, pick-place service bridge, selected-cube adapter, custom perception interfaces. | Replaces `holo_assist_depth_tracker`, `holo_assist_depth_tracker_sim`, and `holo_assist_depth_tracker_sim_interfaces` references where those old names refer to current code. |
| `holoassist_movement` | `ros2_ws/src/HoloAssist_Movement` | MoveIt execution, workspace scene, pick/place sequencing, movement interface messages. | Replaces `moveit_robot_control` and `moveit_robot_control_msgs`. |

Current first-party package proof:

- `holoassist_perception`: `ros2_ws/src/HoloAssist_Perception/package.xml:3`
- `holoassist_movement`: `ros2_ws/src/HoloAssist_Movement/package.xml:4`

## Current ROS Submodules

From `.gitmodules` and the parent gitlink table:

| Submodule path | Remote | Pinned commit | Role |
|---|---|---:|---|
| `ros2_ws/src/easy_handeye2` | `https://github.com/marcoesposito1988/easy_handeye2.git` | `b42cae604b5c01dbd650fcdac40dbf334cb098f4` | Eye-to-hand calibration GUI and message packages. |
| `ros2_ws/src/UR_OnRobot_ROS2` | `https://github.com/tonydle/UR_OnRobot_ROS2.git` | `9b5f7f80f3fb0761fb97ca4894f00443b6d07b32` | Combined UR + OnRobot launch, description, and MoveIt config. Dirty locally due `use_fake_gripper_hardware` patch. |
| `ros2_ws/src/ROS-TCP-Endpoint` | `https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git` | `54c1a64b6d5ef6ffa0a0431570bb74329b79b15b` | ROS side of Unity TCP bridge on port 10000. |
| `ros2_ws/src/onrobot_description` | `https://github.com/tonydle/OnRobot_ROS2_Description.git` | `29180b3fa9cba6555f3e515e789b8ccd34252fab` | OnRobot RG2/RG6 descriptions. |
| `ros2_ws/src/onrobot_driver` | `https://github.com/tonydle/OnRobot_ROS2_Driver.git` | `b99abaccfbbe90f2096feff833f4c0849757a587` | OnRobot gripper hardware driver. |

References:

- `.gitmodules` current ROS entries: `.gitmodules:1`, `.gitmodules:4`, `.gitmodules:11`, `.gitmodules:15`, `.gitmodules:19`.
- Gitlink commit table from `git ls-files -s`: parent repository records all above as mode `160000`.

## Current Unity / ThirdParty Submodules

| Submodule path | Remote | Pinned commit | Role |
|---|---|---:|---|
| `ThirdParty/ROS-TCP-Connector` | `https://github.com/Unity-Technologies/ROS-TCP-Connector.git` | `c27f00c6cf750d2d0564349b3039d19aa3925e7c` | Unity-side ROS TCP connector and visualizations packages. |
| `ThirdParty/URDF-Importer` | `https://github.com/Unity-Technologies/URDF-Importer.git` | `90f353e4352aae4df52fa2c05e49b804631d2a63` | Unity URDF importer package. |
| `ThirdParty/Q3toROS` | `https://github.com/tonydle/Q3toROS.git` | `427d7f68ef8e5f0f1982e18a14ec2d71394ae7ad` | Quest-to-ROS helper code or examples. |

Unity consumes the first two directly through file dependencies in `Unity/My project/Packages/manifest.json:13`.

## Packages Provided by `UR_OnRobot_ROS2`

| ROS package | Path | Role |
|---|---|---|
| `ur_onrobot_control` | `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_control` | Launches UR + OnRobot driver/control stack. |
| `ur_onrobot_description` | `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_description` | Combined UR arm plus OnRobot gripper xacro/URDF. |
| `ur_onrobot_moveit_config` | `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_moveit_config` | MoveIt config used by `holoassist_movement`. |

References:

- `ur_onrobot_control` depends on `ur_robot_driver`, `onrobot_driver`, and controller packages: `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_control/package.xml:16`.
- `ur_onrobot_description` depends on `ur_description` and `onrobot_description`: `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_description/package.xml:22`.
- `ur_onrobot_moveit_config` depends on MoveIt, `ur_onrobot_control`, and `ur_onrobot_description`: `ros2_ws/src/UR_OnRobot_ROS2/ur_onrobot_moveit_config/package.xml:24`.

## Packages Provided by `easy_handeye2`

| ROS package | Path | Role |
|---|---|---|
| `easy_handeye2` | `ros2_ws/src/easy_handeye2/easy_handeye2` | Calibration GUI and calibration logic. |
| `easy_handeye2_msgs` | `ros2_ws/src/easy_handeye2/easy_handeye2_msgs` | Calibration message/service definitions. |

References:

- `easy_handeye2` package name and version: `ros2_ws/src/easy_handeye2/easy_handeye2/package.xml:6`.
- `easy_handeye2_msgs` package name and version: `ros2_ws/src/easy_handeye2/easy_handeye2_msgs/package.xml:4`.

## Apt / System Dependencies

The current repo still relies on apt-installed packages for the core ROS stack:

- ROS 2 Humble
- MoveIt 2
- `apriltag_ros`
- `realsense2_camera`
- `rviz2`
- `tf2_ros`
- Universal Robots driver packages such as `ur_robot_driver`, `ur_controllers`, `ur_description`, and `ur_dashboard_msgs`

Do not document `Universal_Robots_ROS2_Driver` as a direct submodule in this checkout. The website currently does this in several places, but current `.gitmodules` has no such entry.

## Local-Patch Status

`ros2_ws/src/UR_OnRobot_ROS2` is a submodule and is currently dirty by design. The three modified tracked files add `use_fake_gripper_hardware` support:

- `ur_onrobot_control/launch/start_robot.launch.py`
- `ur_onrobot_description/urdf/ur_onrobot.urdf.xacro`
- `ur_onrobot_description/urdf/ur_onrobot_macro.xacro`

The local patch is documented at `docs/ur_onrobot_ros2_local_patches.md:1`.

Best long-term website note:

> `UR_OnRobot_ROS2` should be forked or patched explicitly because HoloAssist currently carries a local fake-gripper integration patch inside the submodule. Until that patch is committed to a fork, clean clones or submodule resets need the patch reapplied.

