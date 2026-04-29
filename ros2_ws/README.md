# UR3e / UR3 ROS 2 Simulation Startup

This workspace is set up for the Universal Robots UR3e / UR3 using ROS 2 Humble, the UR ROS 2 driver, URSim, RViz, and MoveIt.

URSim is the simulated Polyscope teach pendant. It is useful for practising robot connection and External Control, but it is not a Gazebo-style physics simulation.

## Robot Type Cheatsheet

Use one of these consistently in commands:

| Robot | `ur_type` | Docker image |
| --- | --- | --- |
| UR3e, e-Series | `ur3e` | `universalrobots/ursim_e-series` |
| UR3, CB3 series | `ur3` | `universalrobots/ursim_cb3` |

The examples below use `ur3e`.

## One-Time Setup

Install the UR driver and MoveIt packages if they are not already installed:

```bash
sudo apt update
sudo apt install ros-humble-ur ros-humble-moveit*
```

Make sure Docker works without `sudo`:

```bash
docker run hello-world
```

If that only works with `sudo`, follow Docker's Linux post-install steps, then log out and back in.

Pull the URSim image:

```bash
docker pull universalrobots/ursim_e-series
```

For a CB3 UR3 instead:

```bash
docker pull universalrobots/ursim_cb3
```

## Daily Startup

Open separate terminals for each step.

### Terminal 1: Start URSim

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
ros2 run ur_client_library start_ursim.sh -m ur3e
```

For a CB3 UR3:

```bash
ros2 run ur_client_library start_ursim.sh -m ur3
```

If you are using WSL, start URSim with explicit port forwarding:

```bash
ros2 run ur_client_library start_ursim.sh \
  -m ur3e \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"
```

For WSL with a CB3 UR3:

```bash
ros2 run ur_client_library start_ursim.sh \
  -m ur3 \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"
```

Open Polyscope in your browser:

```text
http://192.168.56.101:6080/vnc.html
```

On WSL, use:

```text
http://localhost:6080/vnc.html
```

### Terminal 2: Start the UR ROS Driver

Check that the simulator is reachable:

```bash
ping 192.168.56.101
```

Then launch the driver:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true
```

For a CB3 UR3, change `ur_type`:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true
```

In the Polyscope browser window, load or select the **External Control** program and press **Play**. The ROS driver will connect, but the robot will not accept trajectory commands until External Control is running.

### Terminal 3: Start MoveIt

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
ros2 launch ./ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true launch_servo:=false
```

For a CB3 UR3:

```bash
ros2 launch ./ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true launch_servo:=false
```

You should now have RViz with the robot model and MoveIt planning available.

## Running Movement Package

Use this flow when you want to send target coordinates over a ROS topic and
listen for movement state, debug information, and completion.

Open separate terminals and run them in this order.

### Terminal 1: URSim

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
ros2 run ur_client_library start_ursim.sh -m ur3e
```

### Terminal 2: UR Driver

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true
```

In the Polyscope browser window, load or select the **External Control** program
and press **Play**.

### Terminal 3: MoveIt

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 launch ./ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true launch_servo:=false
```

### Terminal 4: Coordinate Listener

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
colcon build --packages-select moveit_robot_control --symlink-install
source install/setup.bash
ros2 launch moveit_robot_control coordinate_listener.launch.py
```

The listener subscribes to `/moveit_robot_control/target`
(`geometry_msgs/msg/Point`) and publishes `complete` on
`/moveit_robot_control/complete` (`std_msgs/msg/String`) after the robot reaches
each coordinate.

### Terminal 5: Send A Coordinate

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 topic pub --once /moveit_robot_control/target geometry_msgs/msg/Point "{x: 0.30, y: 0.10, z: 0.25}"
```

### Optional Debug Terminals

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 topic echo /moveit_robot_control/state
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 topic echo /moveit_robot_control/status
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 topic echo /moveit_robot_control/debug
```

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
source install/setup.bash
ros2 topic echo /moveit_robot_control/complete
```

The state topic publishes values such as `READY`, `QUEUED`, `PLANNING`,
`PLANNED`, `EXECUTING`, `COMPLETE`, `FAILED`, and `REJECTED`. The debug topic is
a JSON string with details like the goal id, target coordinate, queue length,
planning time, trajectory point count, execution time, and failure reason.

## Move The Robot

Use the existing joint-position MoveIt script:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
python3 moveit_robot_control.py
```

Or move the end effector to a coordinate point using the coordinate-control script:

```bash
source /opt/ros/humble/setup.bash
cd /home/ollie/RS2_workspace/ros2_ws
./moveit_coordinate_control.py 0.30 0.10 0.25 --dry-run
```

The coordinate script expects `x y z` in meters, in the `base_link` frame. The default end-effector link is `tool0`.

When the dry run succeeds, remove `--dry-run` to actually execute:

```bash
./moveit_coordinate_control.py 0.30 0.10 0.25
```

If planning fails because the current tool orientation is too restrictive, let MoveIt choose the orientation:

```bash
./moveit_coordinate_control.py 0.30 0.10 0.25 --position-only
```

Or give the target orientation in degrees:

```bash
./moveit_coordinate_control.py 0.30 0.10 0.25 --roll-deg 180 --pitch-deg 0 --yaw-deg 0
```

The coordinate script also checks for a folded-arm path before publishing the trajectory. If the tool gets too close to the arm, it refuses to execute because that can trigger UR protective stop `C403A0`.

By default it plans several candidate routes, rejects routes that are too close to the arm, too long, or require a large joint swing, and then selects the safest remaining route. You can test route selection without moving:

```bash
./moveit_coordinate_control.py 0.25 -0.20 0.25 --dry-run --route-candidates 5
```

Look for output like:

```text
Candidate 1/5: minimum tool0 clearance 0.031 m duration 20.40 s, max joint motion 2.300 rad (too close)
Candidate 2/5: minimum tool0 clearance 0.126 m duration 18.10 s, max joint motion 1.900 rad (safe)
Selected safest route from 5 successful candidate(s): minimum tool0 clearance 0.126 m
```

Current conservative defaults:

```text
minimum tool-to-arm clearance: 0.12 m
maximum route duration: 35 s
maximum joint motion from start: 6 rad
```

For risky points, try a higher `z`, use `--position-only`, or move through an intermediate waypoint:

```bash
./moveit_coordinate_control.py 0.30 0.00 0.35 --velocity-scale 0.05 --acceleration-scale 0.05
./moveit_coordinate_control.py 0.25 -0.20 0.30 --position-only --velocity-scale 0.05 --acceleration-scale 0.05
```

To spend more time searching for a safer path:

```bash
./moveit_coordinate_control.py 0.25 -0.20 0.25 --route-candidates 10 --velocity-scale 0.05 --acceleration-scale 0.05
```

To make the guard even stricter:

```bash
./moveit_coordinate_control.py 0.25 -0.20 0.25 --min-tool-arm-distance 0.15 --max-route-duration 25 --route-candidates 10 --velocity-scale 0.05 --acceleration-scale 0.05
```

Do not disable the guard unless you are deliberately testing in simulation:

```bash
./moveit_coordinate_control.py 0.25 -0.20 0.25 --allow-folded-arm-path --dry-run
```

## Useful Checks

List running ROS nodes:

```bash
ros2 node list
```

Check joint states are publishing:

```bash
ros2 topic echo /joint_states --once
```

Check the trajectory controller exists:

```bash
ros2 control list_controllers
```

Check MoveIt planning service exists:

```bash
ros2 service list | grep plan_kinematic_path
```

Check Docker containers:

```bash
docker ps
```

## Common Problems

### `ros2 run ur_client_library start_ursim.sh` is not found

Install the UR packages:

```bash
sudo apt install ros-humble-ur
```

Then open a new terminal and source ROS again:

```bash
source /opt/ros/humble/setup.bash
```

### Docker permission denied

Your user is probably not in the `docker` group yet. Follow Docker's Linux post-install steps, then log out and back in. Test with:

```bash
docker run hello-world
```

### The driver launches but the robot does not move

Check Polyscope and make sure **External Control** is loaded and playing.

Also check the scaled trajectory controller:

```bash
ros2 control list_controllers
```

You want `scaled_joint_trajectory_controller` to be active.

### MoveIt reports a wrist joint tolerance error near `-6.283`

The wrist 3 joint is continuous. Sometimes the robot reports `-360 deg` instead of `0 deg`.

In Polyscope, move wrist 3 back near `0 deg`, then start **External Control** again.

### Polyscope shows protective stop `C403A0`

This means the tool flange came too close to the lower arm. Reset the stop in Polyscope, move the arm to a more open pose, then press **Play** on External Control again.

Avoid repeating the same move. Try:

```bash
./moveit_coordinate_control.py 0.30 0.00 0.35 --position-only --velocity-scale 0.05 --acceleration-scale 0.05
```

The coordinate script now checks planned paths and refuses ones that look likely to trigger this folded-arm stop.

### Cannot ping the simulator

Try:

```bash
ping 192.168.56.101
```

If using WSL, make sure you started URSim with the `-f` port-forwarding command above, then use the localhost VNC page:

```text
http://localhost:6080/vnc.html
```

## Real Robot Notes

For the real MX Lab robot, use the same driver command but change:

```bash
robot_ip:=<real_robot_ip>
```

Use the correct robot type:

```bash
ur_type:=ur3e
```

or:

```bash
ur_type:=ur3
```

Always confirm the robot is clear to move before running scripts that publish trajectories.
