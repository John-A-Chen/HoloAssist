# Eye-to-Hand Calibration — Working Notes

## Goal

Find the static transform between the Intel RealSense depth camera (fixed in the world) and the UR3e robot base (`camera_color_optical_frame` <-> `base_link`). Once we have this, any object the camera detects (e.g. an AprilTag cube) can be expressed in `base_link` coordinates and the robot can move to pick it up.

## The Problem

The camera publishes object poses in `camera_color_optical_frame`. MoveIt plans in `base_link`. We need a transform chain to convert between them. The existing codebase used a workspace board as an intermediary (`camera → workspace_frame → base_link`), but this wasn't working reliably. We decided to start fresh with a direct eye-to-hand calibration.

## Approach: Direct Eye-to-Hand Calibration

Classic robotics approach — no intermediary frame needed:

1. Stick an AprilTag on the end effector (gripper/tool0)
2. Move the robot to ~15-20 different poses where the camera can see the tag
3. At each pose, record:
   - **EE pose in `base_link`** — from robot FK via `/tf`
   - **Tag pose in `camera_color_optical_frame`** — from `apriltag_ros` detection
4. Solve using OpenCV `cv2.calibrateHandEye()` — gives `camera → base_link` transform
5. Publish as a static TF — done

This is ~100 lines of math. No workspace board, no SVD board fitting, no intermediary frame.

## What Was Built

### Files Created

1. **`ros2_ws/src/holo_assist_depth_tracker/holo_assist_depth_tracker/nodes/eye_hand_calibration_node.py`**
   - Full ROS 2 calibration node
   - Two modes: `sim` (synthetic observations, validates solver) and `hardware` (real camera + robot)
   - Sim mode: moves fake robot to 20 random joint configs, computes what camera "would see" (ground truth + Gaussian noise), solves
   - Hardware mode: interactive — user moves robot, calls `~/capture` service at each pose, calls `~/solve` when done
   - Runs all 4 OpenCV solver methods (Tsai, Park, Horaud, Daniilidis) and compares
   - Publishes result as static TF, saves to YAML
   - RViz markers: green spheres at sampled EE positions, blue cube for solved camera

2. **`ros2_ws/src/holo_assist_depth_tracker/config/eye_hand_calibration_params.yaml`**
   - All tunable parameters: frames, tag offset, sim camera pose, noise levels, solver method, output path

3. **`ros2_ws/src/holo_assist_depth_tracker/config/eye_hand_calibration.rviz`**
   - RViz config with RobotModel, TF frames (world, base_link, tool0, camera), calibration markers

4. **`ros2_ws/src/holo_assist_depth_tracker/launch/eye_hand_calibration.launch.py`**
   - Sim mode: starts fake UR3e + MoveIt + calibration node + RViz (self-contained)
   - Hardware mode: starts only the calibration node (assumes robot/camera already running)

### Entry Point

Registered in `setup.py` as `holoassist_eye_hand_calibration`.

### Dependencies Symlinked

The `ur_onrobot` packages weren't on the `main` branch. Symlinked from `/home/nic/git/src/`:
- `ur_onrobot_description`, `ur_onrobot_control`, `ur_onrobot_moveit_config`
- `onrobot_description`, `onrobot_driver`

All 11 packages build successfully.

## What Was Tested

### Sim Mode

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch holo_assist_depth_tracker eye_hand_calibration.launch.py
```

- Fake robot + MoveIt starts, robot moves to 20 random poses
- All 4 solver methods run. Results:

| Method | Position Error | Rotation Error | Verdict |
|---|---|---|---|
| **Park** | **0.4 mm** | **0.03 deg** | **Near perfect** |
| Tsai | 1053 mm | 119 deg | Garbage |
| Horaud | Large | Large | Bad |
| Daniilidis | Large | Large | Bad |

- Default solver changed from `tsai` to `park` after seeing these results
- Added `auto` mode that picks the most consistent method across all solvers
- Calibration saves to `~/.holoassist/calibration/eye_hand_calibration_latest.yaml`

### RViz

- First attempt: RViz opened blank (no config file specified)
- Fixed: created `eye_hand_calibration.rviz` with RobotModel, TF, MarkerArray displays
- Launch file now loads this config automatically

## What We Considered But Didn't Do

### Using a real camera with fake robot (cube as pretend EE)

Idea: move a physical AprilTag cube to 20 positions in front of the real camera, pretend it's on the end effector.

**Why it doesn't work:** Eye-to-hand calibration needs paired data at each pose:
- Tag pose in camera frame — yes, the real camera can provide this
- EE pose in base_link — **no**, there's no real robot providing FK

Without the robot side, we only have half the equation. The camera can see the tag but we don't know where the "end effector" is in robot coordinates.

### Camera-only test mode

Could add a `camera_test` mode that:
- User moves cube to positions in front of real camera
- Verifies apriltag_ros detects the tag and publishes good TF
- Logs all detections to confirm the pipeline works
- Doesn't try to solve (no robot data)

This would validate the camera side before going to the lab. Not implemented yet.

### Using the workspace board instead

The existing `workspace_board_node` already solves `camera → workspace_frame` from the 4 corner AprilTags on the workspace board. If we know where the robot sits relative to the board (from `board_calibration_node` or manual measurement), we get the full chain:

```
camera → workspace_frame → base_link
```

This is what the existing system was designed to do, but it wasn't working reliably (the reason we started this exercise).

### workspace_align_camera_tf_node (already exists)

File: `ros2_ws/src/holo_assist_depth_tracker_sim/holo_assist_depth_tracker_sim/workspace_align_camera_tf_node.py`

Automatically computes `world → camera_link` by snapping the camera-detected workspace_frame onto the known `base_link → workspace_frame` position. Replaces hand-tuned static camera mount TF. Requires both the board calibration and workspace_board_node to be running.

## What To Do Next

### When you have the real robot + camera in the lab:

1. **Print and tape AprilTag ID 50** (36h11 family, 32mm) onto the gripper/tool0
   - Make sure the tag is flat and rigidly attached — any wobble ruins calibration
   - Note the offset from tool0 origin to tag center and update `tag_offset_x/y/z` in the config

2. **Start the robot + camera + apriltag_ros:**
   ```bash
   # Terminal 1: robot driver
   ros2 launch ur_onrobot_control start_robot.launch.py \
       ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.192 launch_rviz:=false
   # Then run External Control on teach pendant

   # Terminal 2: camera + apriltag
   ros2 launch holo_assist_depth_tracker camera_only.launch.py
   ros2 run apriltag_ros apriltag_node --ros-args \
       -p "families:=36h11" -p "size:=0.032" \
       --remap image_rect:=/camera/camera/color/image_raw \
       --remap camera_info:=/camera/camera/color/camera_info
   ```

3. **Run calibration in hardware mode:**
   ```bash
   # Terminal 3:
   ros2 launch holo_assist_depth_tracker eye_hand_calibration.launch.py mode:=hardware
   ```

4. **Collect samples** — move robot to ~20 diverse poses (vary position AND orientation):
   ```bash
   # At each pose where the camera can see the tag:
   ros2 service call /eye_hand_calibration/capture std_srvs/srv/Trigger
   ```

5. **Solve:**
   ```bash
   ros2 service call /eye_hand_calibration/solve std_srvs/srv/Trigger
   ```

6. **Result** saved to `~/.holoassist/calibration/eye_hand_calibration_latest.yaml`

### Tips for good calibration accuracy:

- Use at least 15-20 poses (more is better)
- Cover the full workspace — don't cluster poses in one area
- Vary the EE orientation significantly (tilt, rotate) — not just position
- Make sure the tag is fully visible in the camera at each pose (no partial occlusion)
- Keep the tag rigidly mounted — if it moves relative to tool0 between captures, results degrade
- The Park solver method consistently outperforms Tsai for this setup

### After calibration:

The saved YAML contains `world → camera_link`. To use it at runtime, either:
- Keep the calibration node running (it holds the static TF)
- Or publish it from a static transform publisher loading the YAML

Then any object detected by the camera (cube poses from `cube_pose_node`, etc.) can be transformed to `base_link` via TF for MoveIt planning.

### Things to investigate if accuracy is poor:

- **Camera intrinsics** — is the RealSense factory calibration accurate? Can recalibrate with Intel's tool
- **Tag detection noise** — larger tags (64mm instead of 32mm) give more stable pose estimates
- **Tag planarity** — if the tag isn't perfectly flat, pose estimation suffers
- **Lighting** — harsh reflections or low light degrade tag detection
- **Robot calibration** — the UR3e kinematic calibration was extracted (see `may6.md`); make sure the calibrated YAML is being used in the driver launch

## Existing Calibration Infrastructure (for reference)

The codebase already has two other calibration paths:

| Approach | Node | TF Published | How |
|---|---|---|---|
| Board calibration (FK) | `board_calibration_node` | `base_link → workspace_frame` | Robot visits board corners, SVD solve |
| Board detection (camera) | `workspace_board_node` | `camera_optical → workspace_frame` | Camera sees 4 corner AprilTags, SVD solve |
| **Eye-to-hand (new)** | `eye_hand_calibration_node` | `world → camera_link` | Camera sees tag on EE at multiple poses, OpenCV solve |

The eye-to-hand approach is the most direct — it gives `camera → base_link` without any intermediary frame. The board-based approaches require both calibration steps and the board to be visible.
