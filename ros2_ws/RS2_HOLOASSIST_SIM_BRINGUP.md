# RS2 HoloAssist Bringup (Simulation Only, URSim + ROS 2 Humble)

This runbook validates the UR3e software stack end-to-end without a physical robot:

- URSim (PolyScope in Docker)
- External Control URCap
- `ur_robot_driver`
- `ros2_control` controllers
- MoveIt 2 + RViz
- MoveIt Servo
- HoloAssist click-to-plan + pose-to-servo bridge nodes

## 0. Architecture (Data Flow)

```text
RViz / HoloAssist nodes / Unity (later)
        |                     |
        | Pose / Click / Twist|
        v                     v
   MoveIt move_group      MoveIt Servo
        | (plan+execute)     | (real-time jogging)
        v                    v
   ros2_control controllers (JTC / scaled JTC / velocity controller)
        |
        v
   ur_robot_driver (ROS 2 driver + RTDE + dashboard + script interfaces)
        |
        v
   URSim (External Control URCap program running)
```

MoveIt does not need a custom bridge to the UR driver. The integration point is `ros2_control` + controller actions/topics.

## 1. Networking and Middleware (Container-Safe)

Use Cyclone DDS in all ROS terminals/containers:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Recommended Docker networking for URSim on Linux:

- Easiest: `--network host` (lowest friction for UR driver and URCap reverse connection)
- More container-safe/reproducible: user-defined bridge network with fixed IPs for URSim and ROS container

Important UR driver/URCap connectivity detail:

- The External Control URCap opens a reverse socket back to the ROS driver host (`reverse_ip` / `script_sender` path).
- If using Docker bridge mode, the IP entered in URCap must be reachable from URSim container (usually the ROS container IP on the same Docker network).

## 2. URSim + External Control Bringup

Example URSim (e-Series) launch in Docker (adjust image/tag as installed):

```bash
docker run --rm -it \
  --name ursim \
  --network host \
  universalrobots/ursim_e-series
```

URCap setup (inside PolyScope in URSim):

1. Install/enable External Control URCap (if not already in image).
2. Create a UR3e program with an External Control node.
3. Enter the ROS driver host IP and port (usually reverse port default from `ur_robot_driver` launch).
4. Put robot in Remote Control mode (URSim UI).
5. Press Play on the program (External Control must be running before motion commands work).

## 3. Driver Bringup (`ur_robot_driver`)

Launch the UR driver against URSim:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=<URSIM_IP> \
  launch_rviz:=false
```

Notes:

- `use_mock_hardware:=false` for URSim + External Control (you are using the real networked driver path, not fake hardware).
- Some installs require extra args (`initial_joint_controller`, calibration file, etc.). Use `ros2 launch ... --show-args`.

Validate before proceeding:

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 action list | rg scaled_joint_trajectory_controller
```

Expected:

- `/joint_states` publishes continuously
- `joint_state_broadcaster` active
- `scaled_joint_trajectory_controller` present (typically active)

Quick execution smoke test (trajectory action):

- Send a small joint goal to `/scaled_joint_trajectory_controller/follow_joint_trajectory`
- Confirm URSim robot moves

## 4. MoveIt 2 + RViz Integration (UR3e)

Launch UR MoveIt config (from `ur_moveit_config`):

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true
```

Validate:

```bash
ros2 node list | rg move_group
ros2 param list /move_group | rg robot_description
```

In RViz:

1. Add/start MotionPlanning panel.
2. Confirm planning scene and TF render correctly.
3. Set target pose/joint goal.
4. Click Plan (should succeed).
5. Click Execute (URSim should move through the driver).

### MoveIt -> UR driver data flow (no custom bridge)

```text
MoveIt move_group
  -> TrajectoryExecutionManager
  -> FollowJointTrajectory action (ros2_control controller)
  -> /scaled_joint_trajectory_controller
  -> ur_robot_driver hardware interface
  -> URScript + RTDE / External Control
  -> URSim
```

No extra bridge is required between MoveIt and `ur_robot_driver` if the UR controller is correctly exposed through `ros2_control`.

## 5. Click-to-Plan (Simulation)

Your repo already includes a click-to-plan node:

- `ros2_ws/src/holoassist_manipulation/src/clicked_point_to_moveit.cpp`

Launch it:

```bash
ros2 launch holoassist_manipulation click_to_plan.launch.py \
  move_group_name:=ur_manipulator \
  planning_frame:=base_link \
  tcp_frame:=tool0
```

RViz test:

1. Use the `Publish Point` tool (publishes `/clicked_point`)
2. Click reachable points in front of the robot
3. Node converts point -> `PoseStamped`, plans, then executes

Current implementation assumptions:

- Clicked point is already in `planning_frame`
- Orientation is fixed (identity quaternion)

You will likely tune orientation and add TF transforms next.

## 6. MoveIt Servo (UR3e, Simulation via URSim)

Yes, Servo can run against URSim through the normal UR driver path.

### Controller choice for Servo

Servo does not require a UR-specific bridge, but it does require a controller/output type that matches its command stream.

Two workable options:

- `scaled_joint_trajectory_controller` (joint trajectory topic): easiest to share with MoveIt execution
- Velocity controller (e.g. `forward_velocity_controller` / `JointGroupVelocityController`): lower latency feel for teleop, but requires controller switching and different Servo output config

Pragmatic recommendation for today:

- Start with `scaled_joint_trajectory_controller`
- Use Servo output type `trajectory_msgs/JointTrajectory`
- Publish to `/scaled_joint_trajectory_controller/joint_trajectory`

### UR-specific Servo parameters to verify

Check/update in your Servo YAML:

- `move_group_name: ur_manipulator`
- `planning_frame: base_link`
- `ee_frame_name`: usually `tool0` (or your TCP frame)
- `robot_link_command_frame`: `base_link` (for twist in base frame)
- `joint_topic: /joint_states`
- `command_out_topic: /scaled_joint_trajectory_controller/joint_trajectory`
- `command_out_type: trajectory_msgs/JointTrajectory`
- `publish_joint_velocities` / `publish_joint_positions` consistent with selected controller
- `incoming_command_timeout` small enough to stop on command loss
- collision checking rate tuned so it does not starve loop rate

### Servo validation sequence

1. Start driver + MoveIt + Servo node.
2. Confirm Servo status topic/logs show initialized (not halted due to collisions/singularity by default).
3. Stream `TwistStamped` to Servo input (often `/servo_node/delta_twist_cmds`).
4. Confirm continuous motion in URSim.
5. Stop publishing -> robot stops (timeout / zero twist).

Example one-shot command (replace frame if needed):

```bash
ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  -r 50
```

## 7. HoloAssist Servo Bridge (Pose -> Twist)

Your repo includes a pose-to-servo bridge:

- `ros2_ws/src/holoassist_servo_tools/holoassist_servo_tools/pose_to_twist_servo.py`

Launch it:

```bash
ros2 launch holoassist_servo_tools pose_to_twist_servo.launch.py \
  command_frame:=base_link \
  eef_frame:=tool0 \
  target_topic:=/servo_target_pose \
  twist_topic:=/servo_node/delta_twist_cmds
```

This lets you test future Unity/XR pose targets immediately in simulation by publishing `PoseStamped` to `/servo_target_pose`.

## 8. Suggested Bringup Order (Today)

Use this exact order and validate each stage before advancing:

1. URSim Docker up, External Control program loaded (not yet running)
2. `ur_robot_driver` launch
3. Start External Control program in URSim
4. Validate `/joint_states` + controllers
5. MoveIt (`ur_moveit_config`) + RViz
6. Plan/Execute in RViz
7. Click-to-plan node (`holoassist_manipulation`)
8. MoveIt Servo node (UR config)
9. `pose_to_twist_servo` bridge
10. Unity TCP endpoint (architecture only / dry run)

## 9. Common Failure Points (Highest Probability)

1. External Control URCap not running in URSim
- Symptom: driver connects but motion commands do nothing / controller errors

2. Wrong IP entered in URCap (reverse connection issue)
- Symptom: driver starts, but program never transitions to active control

3. Controller mismatch (MoveIt/Servo output vs active controller)
- Symptom: planning succeeds but execution fails, or Servo accepts commands but no motion

4. Wrong frame names (`tool0`, `base`, `base_link`, custom TCP)
- Symptom: Servo motion goes wrong direction or MoveIt target is infeasible

5. MoveIt controller config not pointing at `scaled_joint_trajectory_controller`
- Symptom: plan succeeds, execute instantly fails with missing action server

6. Servo safety halting (collision/singularity/joint limit)
- Symptom: commands received but status remains halted or velocities clamped to zero

7. DDS/network isolation between containers
- Symptom: nodes run but topics/actions invisible across containers

## 10. Unity / XR Bridge Architecture (Today: Design Only)

Target data paths:

- Unity hand pose -> ROS `PoseStamped`
- ROS pose -> MoveIt Servo (`TwistStamped`) via bridge node
- ROS feedback -> Unity (`/joint_states`, tool pose, robot status)

### Option comparison

`rosbridge_websocket`

- Pros: easy, widely used, browser-friendly
- Cons: JSON serialization overhead, higher latency/jitter
- Verdict: good for UI/debug, not ideal for tight VR teleop loop

`ROS-TCP-Endpoint` (Unity Robotics)

- Pros: practical Unity integration, typed messages, straightforward setup
- Cons: extra TCP serialization layer, can add latency under load
- Verdict: best near-term choice for HoloAssist prototyping

`Direct DDS` (Cyclone DDS / RTPS in Unity)

- Pros: lowest latency path, native pub/sub model, fewer bridges
- Cons: significantly more integration complexity in Unity, portability/tooling cost
- Verdict: best long-term performance path if teleop latency is critical and team can absorb integration work

### Recommendation for low-latency VR teleoperation

For today / MVP:

- Use `ROS-TCP-Endpoint` for Unity integration
- Keep the real-time servo loop on ROS side (`PoseStamped` -> `pose_to_twist_servo` -> Servo)
- Send only high-level pose targets from Unity, not per-joint commands

For production/latency-critical iteration:

- Evaluate direct DDS or a native plugin transport after you baseline end-to-end latency with ROS-TCP

## 11. What to Validate at Each Stage (Gate Checklist)

Driver gate:

- `/joint_states` live
- controller list sane
- small trajectory action moves URSim

MoveIt gate:

- `move_group` running
- plan works in RViz
- execute moves URSim

Click-to-plan gate:

- `/clicked_point` received
- plan generated
- execute succeeds for reachable points

Servo gate:

- Servo initialized, not halted
- 50 Hz `TwistStamped` produces continuous motion
- stop on zero/timeout works

XR bridge gate:

- Unity can publish `PoseStamped`
- ROS can return `joint_states` + tool pose
- measured RTT/jitter acceptable for your target UX

