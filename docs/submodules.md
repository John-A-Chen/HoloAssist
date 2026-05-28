# Repo Structure & Submodule Plan

Goal: `git clone --recursive git@github.com:John-A-Chen/HoloAssist.git` gives a fully
buildable workspace with no manual cloning steps.

---

## Current State

### Packages that already live in the main repo (stay here)

These are fully custom — no upstream to track:

| Package | Path |
|---|---|
| `holoassist_perception` | `ros2_ws/src/HoloAssist_Perception/` |
| `holoassist_movement` | `ros2_ws/src/HoloAssist_Movement/` |
| `holoassist_unity_bridge` | `ros2_ws/src/holoassist_unity_bridge/` |

### Packages installed via apt (no vendoring needed)

These are standard ROS Humble debs — `rosdep` handles them:

| Package | apt name |
|---|---|
| AprilTag detection node | `ros-humble-apriltag-ros` |
| AprilTag C++ lib | `ros-humble-apriltag` |
| MoveIt 2 | `ros-humble-moveit` |
| RealSense driver | `ros-humble-realsense2-camera` |
| RViz2 | `ros-humble-rviz2` |
| tf2 | `ros-humble-tf2-ros` |

---

## Packages That Need Submodule Conversion

### 1. `easy_handeye2` — simple, just needs `.gitmodules`

**Status:** Git already tracks this as a gitlink (`160000 b42cae6`). Working tree is clean.
The only missing piece is a `.gitmodules` entry so `git clone --recursive` resolves it.

**Action:**
```bash
# Run from repo root
git submodule add --name easy_handeye2 \
  https://github.com/marcoesposito1988/easy_handeye2.git \
  ros2_ws/src/easy_handeye2
git submodule set-branch --branch master ros2_ws/src/easy_handeye2
```

**Pinned commit:** `b42cae6`

---

### 2. `Universal_Robots_ROS2_Driver` — straight submodule against upstream

**Status:** Has its own `.git`, untracked by parent. Only local change = deleted markdown
files, no code changes. Safe to submodule directly against upstream.

**Upstream:** `git@github.com:UniversalRobots/Universal_Robots_ROS2_Driver.git`  
**Pinned commit:** `7658387` (describes as `2.12.0-14-g7658387`)

> **Check for updates:** Upstream UR ROS2 driver is actively maintained. Before upgrading,
> verify `ur_onrobot_control` launch files still work with the new driver API — the controller
> names and launch arguments can change between minor versions.

**Action:**
```bash
# Remove the bare .git dir first, then add as submodule
rm -rf ros2_ws/src/Universal_Robots_ROS2_Driver/.git
git submodule add --name Universal_Robots_ROS2_Driver \
  git@github.com:UniversalRobots/Universal_Robots_ROS2_Driver.git \
  ros2_ws/src/Universal_Robots_ROS2_Driver
# Then pin to our tested commit
cd ros2_ws/src/Universal_Robots_ROS2_Driver && git checkout 7658387
```

---

### 3. `ROS-TCP-Endpoint` — needs a fork first

**Status:** Has its own `.git`, untracked by parent. Has **one real local fix** that upstream
has not merged: `setup.cfg` uses hyphens (`script-dir`, `install-scripts`) which Python's
setuptools on our Ubuntu 22.04 / Python 3.10 does not accept — `ros2 run` can't find the
endpoint executable. Our fix changes them to underscores (`script_dir`, `install_scripts`).

The exact diff:
```diff
 [develop]
-script-dir=$base/lib/ros_tcp_endpoint
+script_dir=$base/lib/ros_tcp_endpoint
 [install]
-install-scripts=$base/lib/ros_tcp_endpoint
+install_scripts=$base/lib/ros_tcp_endpoint
```

**Upstream:** `git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git`  
**Our version:** `v0.7.0` (commit `54c1a64`)

> **Check for updates:** Unity has published newer ROS-TCP-Endpoint versions under a different
> branch structure. Check `main-ros2` branch at the upstream for newer releases. The fix above
> may already be in a newer version, which would simplify us back to a plain submodule.

**Action — requires manual GitHub step first:**

1. Fork `Unity-Technologies/ROS-TCP-Endpoint` to `John-A-Chen/ROS-TCP-Endpoint` on GitHub
2. On the fork, commit the `setup.cfg` fix to a branch (e.g. `ros2-setupcfg-fix`)
3. Then:
```bash
rm -rf ros2_ws/src/ROS-TCP-Endpoint/.git
git submodule add --name ROS-TCP-Endpoint \
  git@github.com:John-A-Chen/ROS-TCP-Endpoint.git \
  ros2_ws/src/ROS-TCP-Endpoint
```

---

### 4. `onrobot_description` and `onrobot_driver` — submodule against Tony Le's upstream

**Status:** Source files committed directly into this repo (no `.git` dir, not a submodule).
No local modifications — these are verbatim copies of Tony Le's packages.

**Upstreams:**
- `https://github.com/tonydle/OnRobot_ROS2_Description.git` (onrobot_description)
- `https://github.com/tonydle/OnRobot_ROS2_Driver.git` (onrobot_driver)

> **Check for updates:** Tony Le actively maintains these. Check for gripper model or
> driver fixes before pinning.

**Action:**
```bash
git rm -r ros2_ws/src/onrobot_description ros2_ws/src/onrobot_driver
git submodule add --name onrobot_description \
  https://github.com/tonydle/OnRobot_ROS2_Description.git \
  ros2_ws/src/onrobot_description
git submodule add --name onrobot_driver \
  https://github.com/tonydle/OnRobot_ROS2_Driver.git \
  ros2_ws/src/onrobot_driver
```

---

### 5. `ur_onrobot` — fork needed (we have integration changes)

**Status:** Source files committed directly into this repo. Contains three packages:
`ur_onrobot_description`, `ur_onrobot_control`, `ur_onrobot_moveit_config`. These are Tony
Le's UR+OnRobot integration layer from `github.com/Tony-Leu/UR_OnRobot_ROS2`.

**Local changes (~429 lines diff):** Primarily in `view_robot.rviz` — updated cube marker
namespaces, added apriltag frame display, adjusted panel sizing to match our perception
pipeline RViz setup. These are integration-level config changes, not code changes.

**Recommended approach:** Fork `Tony-Leu/UR_OnRobot_ROS2`, commit our RViz + config
changes to the fork, then submodule against the fork.

> If Tony Le's upstream diverges significantly (new gripper models, new controller layout),
> we can rebase our fork on top. Our changes are isolated to `.rviz` config so rebases are
> low-risk.

---

## How We Integrate External Packages

### apriltag_ros (apt, not vendored)

We use `ros-humble-apriltag-ros` as-is. Our `CubePoseNode` subscribes to
`/detections_all` (`apriltag_msgs/AprilTagDetectionArray`) and does all multi-face cube
centre fusion internally. We don't modify `apriltag_ros` itself — we built on top of its
TF-publishing output.

Relevant config: `config/apriltag_all.yaml` sets `family: 36h11`, `size: 0.032`, and lists
all tag IDs `[0..3, 10..33]` that the detector should publish TF for.

### MoveIt 2 (apt, not vendored)

We use `ros-humble-moveit` as-is. Our custom packages are:
- `holoassist_movement` — wraps `MoveGroupInterface` with our Cartesian-first/pose-goal-fallback
  pattern, manages the workspace collision scene, runs the pick/place state machine, and defines
  the typed command/status messages for our control loop
- `ur_onrobot_moveit_config` — SRDF and MoveIt config for the UR3e+RG2 arm group

We don't patch MoveIt itself. If a future MoveIt Humble update breaks the `MoveGroupInterface`
API, only `HoloAssist_Movement/holoassist_movement_nodes/holoassist_movement.py` needs updating.

### ROS-TCP-Endpoint (fork)

We use Unity's `ros_tcp_endpoint` package to bridge ROS topics/services to Unity over TCP
port 10000. Our only change is the `setup.cfg` fix described above — the package logic is
unchanged. `beacon.py` (our addition) runs alongside it to broadcast the laptop IP over UDP
so Quest headsets can auto-discover the endpoint.

### easy_handeye2 (upstream submodule)

We use the upstream package without modification. It provides the GUI for eye-on-base
calibration and saves the result to `~/.ros2/easy_handeye2/calibrations/`. Our `launch.py`
reads the saved `.calib` YAML directly and publishes the `base_link → camera_link` static TF
— so `easy_handeye2` is only needed at calibration time, not during normal operation.

---

## Target `.gitmodules`

After all conversions, `.gitmodules` should contain:

```ini
[submodule "easy_handeye2"]
    path = ros2_ws/src/easy_handeye2
    url = https://github.com/marcoesposito1988/easy_handeye2.git
    branch = master

[submodule "Universal_Robots_ROS2_Driver"]
    path = ros2_ws/src/Universal_Robots_ROS2_Driver
    url = git@github.com:UniversalRobots/Universal_Robots_ROS2_Driver.git

[submodule "ROS-TCP-Endpoint"]
    path = ros2_ws/src/ROS-TCP-Endpoint
    url = git@github.com:John-A-Chen/ROS-TCP-Endpoint.git

[submodule "onrobot_description"]
    path = ros2_ws/src/onrobot_description
    url = https://github.com/tonydle/OnRobot_ROS2_Description.git

[submodule "onrobot_driver"]
    path = ros2_ws/src/onrobot_driver
    url = https://github.com/tonydle/OnRobot_ROS2_Driver.git

[submodule "ur_onrobot"]
    path = ros2_ws/src/ur_onrobot
    url = git@github.com:John-A-Chen/UR_OnRobot_ROS2.git
```

---

## Pending Before This Is Fully Set Up

- [ ] Fork `Unity-Technologies/ROS-TCP-Endpoint` → `John-A-Chen/ROS-TCP-Endpoint`, commit `setup.cfg` fix
- [ ] Fork `Tony-Leu/UR_OnRobot_ROS2` → `John-A-Chen/UR_OnRobot_ROS2`, commit our RViz changes
- [ ] Run the submodule conversion commands above
- [ ] Verify `git clone --recursive` then `colcon build` succeeds on a clean machine
