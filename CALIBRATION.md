# Camera Calibration Guide

Eye-to-hand calibration for the Intel RealSense D435I and UR3e robot. Uses `easy_handeye2` with the Park solver to compute the static TF between `base_link` and `camera_color_optical_frame`.

## Prerequisites

- ROS 2 Humble on Ubuntu 22.04
- ROS workspace built: `cd ros2_ws && colcon build --symlink-install`
- `easy_handeye2` cloned and built (see Setup below)
- AprilTag (36h11 family, 32mm+) printed and taped flat onto the gripper
- Intel RealSense D435I plugged in via USB
- UR3e robot reachable at `192.168.0.194`

## Setup (one-time)

### Install easy_handeye2

```bash
cd ros2_ws/src
git clone git@github.com:marcoesposito1988/easy_handeye2.git
cd ../
colcon build --symlink-install --packages-select easy_handeye2_msgs easy_handeye2
pip install -e src/easy_handeye2/easy_handeye2
```

### Install dependencies

```bash
sudo apt install -y ros-humble-rqt-image-view ros-humble-rqt-gui ros-humble-rqt-gui-py ros-humble-tf-transformations
```

### Symlink ur_onrobot sub-packages (if not already done)

```bash
cd ros2_ws/src
ln -s ur_onrobot/ur_onrobot_description .
ln -s ur_onrobot/ur_onrobot_control .
ln -s ur_onrobot/ur_onrobot_moveit_config .
cd ../ && colcon build --symlink-install
```

### Configure the solver

The default solver in `easy_handeye2` is Tsai-Lenz which gives bad results for this setup. Change it to Park:

In `ros2_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py`, line 62:
```python
# Change from:
algorithm = 'Tsai-Lenz'
# To:
algorithm = 'Park'
```

No rebuild needed (symlink install).

### Prepare the AprilTag

- Print an AprilTag from the **36h11** family (any ID works, default config uses ID 0)
- Size: 32mm minimum, **larger is better** (64mm+ recommended for accuracy)
- Tape it **flat and rigid** onto the gripper — any wobble ruins calibration
- Note the tag size in metres (e.g. 0.032 for 32mm) — you'll need it for the apriltag_ros command
- If using a different tag ID, update the `tracking_marker_frame` argument (e.g. `tag36h11:5` for ID 5)

## Calibration Procedure

### Terminal 1 — Robot driver
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194 launch_rviz:=false
```
Then run **External Control** on the teach pendant (host IP: `192.168.0.100`).

### Terminal 2 — Camera
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch holo_assist_depth_tracker camera_only.launch.py
```

### Terminal 3 — AprilTag detector
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args -p "families:=36h11" -p "size:=0.032" -p "publish_tf:=true" --remap image_rect:=/camera/camera/color/image_raw --remap camera_info:=/camera/camera/color/camera_info
```
Replace `0.032` with your actual tag size in metres.

### Terminal 4 — easy_handeye2 calibration server
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py name:=holoassist_calibration calibration_type:=eye_on_base tracking_base_frame:=camera_color_optical_frame tracking_marker_frame:=tag36h11:0 robot_base_frame:=base_link robot_effector_frame:=tool0
```
The rqt GUI may or may not open. Either way, the server works.

### Terminal 5 — Verify tag detection before capturing
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 topic echo /detections --once
```
Hold the tag in the camera's view. You must see a detection before capturing. Optionally view the camera feed:
```bash
ros2 run rqt_image_view rqt_image_view
```

### Collect samples (~20-25 poses)

Move the robot to a pose where the camera can see the tag, then:
```bash
ros2 service call /handeye_server/take_sample easy_handeye2_msgs/srv/TakeSample
```

Repeat 20-25 times. **Important:**
- Vary both **position and orientation** — tilt and rotate the gripper, don't just translate
- Spread across the full workspace
- Make sure the tag is fully visible at each pose (no partial occlusion)
- Wait for the tag to be stable before capturing (no motion blur)

### Save
```bash
ros2 service call /handeye_server/save_calibration std_srvs/srv/Trigger
```

Result saved to `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`.

## Using the Calibration at Runtime

Publish the saved calibration as a static TF:
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch easy_handeye2 publish.launch.py name:=holoassist_calibration
```

Run this in a terminal alongside the robot driver and camera. Any objects detected by the camera will be correctly transformed to `base_link` coordinates.

## Verifying the Calibration

With the calibration publisher, robot driver, and camera all running:

```bash
ros2 launch holo_assist_depth_tracker visualize_depth_tracker.launch.py
```

In RViz, set **Fixed Frame** to `base_link`. You should see the robot model, camera pointcloud, and detected objects all aligned correctly.

Check the TF directly:
```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## Troubleshooting

| Problem | Fix |
|---|---|
| No tag detections | Check `ros2 topic echo /detections --once` with tag in view. Ensure `publish_tf:=true` is set on apriltag_ros. |
| TF flickering between two values | Two nodes publishing the same static TF. Kill any leftover `calibrate.launch.py` or dummy publishers. |
| Calibration result way off | Use Park solver (not Tsai-Lenz). Use a bigger tag. Vary orientation more between samples. |
| `rqt_calibrator.py` crashes | Known issue. Use service calls instead of the GUI. |
| Git HTTPS clone fails | Use SSH: `git clone git@github.com:...` |
| `easy_handeye2_msgs` not found | Build both: `colcon build --packages-select easy_handeye2_msgs easy_handeye2` |
| `ur_onrobot_description` build fails | Symlink sub-packages to top-level `src/` (see Setup). |

## Network

```
Robot's Router (192.168.0.x):
  UR3e robot:       192.168.0.194
  Laptop Ethernet:  192.168.0.100  (teach pendant External Control host IP)
  Laptop WiFi:      192.168.0.102  (for Quest 3)
```

## Reference

- Calibration file: `~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib`
- easy_handeye2 repo: github.com/marcoesposito1988/easy_handeye2
- Solver comparison (from sim testing): Park >> Tsai/Horaud/Daniilidis
