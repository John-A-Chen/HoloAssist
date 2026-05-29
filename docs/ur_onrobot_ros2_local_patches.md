# UR_OnRobot_ROS2 Local Patches

These changes are applied to the `ros2_ws/src/UR_OnRobot_ROS2` submodule but left
uncommitted.  Re-apply them after any submodule reset or fresh clone.

**Why:** `use_fake_hardware:=true` fakes both the UR arm and the gripper (uses a
generic `control_node`, no RTDE).  We need a separate flag so the real UR arm can
connect to URSim via RTDE while only the OnRobot serial port is skipped — this gives
a code path identical to real hardware for sim-to-real validation.

---

## File 1 — `ur_onrobot_description/urdf/ur_onrobot.urdf.xacro`

Add one `<xacro:arg>` line after the existing `use_fake_hardware` arg (line 9):

```xml
    <xacro:arg name="use_fake_hardware" default="false"/>
+   <xacro:arg name="use_fake_gripper_hardware" default="false"/>
```

Pass it into the macro call (after the existing `use_fake_hardware` attribute, ~line 28):

```xml
        use_fake_hardware="$(arg use_fake_hardware)"
+       use_fake_gripper_hardware="$(arg use_fake_gripper_hardware)"
```

---

## File 2 — `ur_onrobot_description/urdf/ur_onrobot_macro.xacro`

Add the parameter to the macro signature (after `use_fake_hardware:=false`, ~line 14):

```xml
        use_fake_hardware:=false
+       use_fake_gripper_hardware:=false
```

Change the `<xacro:onrobot>` call so either flag fakes the gripper (~line 47):

```xml
-       use_fake_hardware="${use_fake_hardware}"
+       use_fake_hardware="${use_fake_hardware or use_fake_gripper_hardware}"
```

---

## File 3 — `ur_onrobot_control/launch/start_robot.launch.py`

**Import:** Add `OrSubstitution` to the existing substitutions import block:

```python
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
+   OrSubstitution,
    PathJoinSubstitution,
)
```

**Read the config value** (after the `use_fake_hardware` line in `launch_setup`, ~line 25):

```python
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
+   use_fake_gripper_hardware = LaunchConfiguration("use_fake_gripper_hardware")
```

**Pass to xacro** (after the `use_fake_hardware:=` block in the Command list, ~line 57):

```python
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
+           "use_fake_gripper_hardware:=",
+           use_fake_gripper_hardware,
+           " ",
```

**Skip serial port** when either flag is set — change the `tool_communication_node`
condition (~line 136):

```python
-       condition=UnlessCondition(use_fake_hardware),
+       condition=UnlessCondition(OrSubstitution(use_fake_hardware, use_fake_gripper_hardware)),
```

**Declare the argument** in `generate_launch_description()`, after the
`use_fake_hardware` argument block (~line 299):

```python
+   declared_arguments.append(
+       DeclareLaunchArgument(
+           "use_fake_gripper_hardware",
+           default_value="false",
+           description="Use mock gripper hardware (skips serial port). Useful with the URSim simulator.",
+       )
+   )
```

---

## After re-applying, rebuild

```bash
cd ros2_ws
colcon build --packages-select ur_onrobot_description ur_onrobot_control
```

## Usage

```bash
# Real robot, fake gripper (simulator or gripper not attached)
./scripts/launch.sh --robot-ip 192.168.56.101 --fake-gripper

# Real robot, real gripper (lab)
./scripts/launch.sh --robot-ip 192.168.0.194
```
