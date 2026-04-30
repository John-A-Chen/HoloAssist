# HoloAssist Quick Start

All commands run from the repo root (`/home/sebastian/git/rs2/HoloAssist`).

## Virtual Testing (Fake Hardware)

Open 5 separate terminals. Copy-paste each block:

**Terminal 1 — UR driver (simulated)**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=xxx use_fake_hardware:=true launch_rviz:=false
```

Wait until you see `[controller_manager]: ...` logs, then open the remaining terminals.

**Terminal 2 — Unity-ROS bridge**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Terminal 3 — TF marker bridge**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run holoassist_manager tf_marker_bridge
```

**Terminal 4 — Activate velocity controller (run once, then close)**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 2}"
```
You should see `ok=True`. This terminal can be closed after.

**Terminal 5 — RViz**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
rviz2 -d holoassist.rviz
```

Then deploy APK to Quest and launch the app.

## Real Hardware (UR3e at 192.168.0.194)

Same as above but replace Terminal 1 with:

**Terminal 1 — UR driver (real robot)**
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=false
```

Then on teach pendant: load and run **External Control** program (Host IP: `192.168.0.100`).

Terminals 2-5 are identical to virtual testing.

## Quest Deployment

**Before building:**
- Delete `Assets/XR/Temp/` folder in Unity if it exists
- Check ROS IP in Unity: **Robotics > ROS Settings**
  - From Unity Editor: `127.0.0.1`
  - From Quest headset: laptop's WiFi IP (run `hostname -I` to check)
- Port: `10000`

**Build and install:**
1. In Unity: **File > Build** (builds the APK)
2. Then **File > Build And Run** (installs and launches on Quest)
3. Or manually install:
```bash
adb install -r "Unity/My project/Builds/HoloAssist.apk"
```

**Restart app on Quest:**
```bash
adb shell am force-stop com.DefaultCompany.Myproject && adb shell monkey -p com.DefaultCompany.Myproject -c android.intent.category.LAUNCHER 1
```

Or: press Meta button > close app > reopen from App Library > Unknown Sources.

## Quest Controls

| Button | Direct Joint Mode | Servo Mode |
|--------|------------------|------------|
| **Menu** (left) | Switch to Servo | Switch to Direct Joint |
| **A** (right) | Next joint | -- |
| **B** (right) | Previous joint | -- |
| **X** (left) | Toggle TF axes | Toggle TF axes |
| **Right stick Y** | Jog joint | End-effector Z |
| **Right stick X** | -- | End-effector yaw |
| **Left stick Y** | -- | End-effector X |
| **Left stick X** | -- | End-effector Y |

## Troubleshooting

- **ros_tcp_endpoint crash on reconnect:** Restart Terminal 2 after every Unity Play/Stop cycle
- **No connection from Quest:** Check WiFi IP with `hostname -I`, update in Unity Robotics > ROS Settings, rebuild APK
- **Build error "Unable to query OpenGL":** Do File > Build first, then File > Build And Run
- **Build error XR/Temp conflict:** Delete `Assets/XR/Temp/` folder before building
- **Build error "PPtr cast failed":** In Unity do Assets > Reimport All, then rebuild
- **Arm not moving:** Make sure velocity controller is activated (Terminal 4). It resets every time you restart Terminal 1
- **End effector pose shows 0s:** Make sure `base_link` is assigned to Robot Base field and `tool0` to End Effector Transform on RobotDataPanel
