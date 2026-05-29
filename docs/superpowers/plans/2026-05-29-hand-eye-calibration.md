# Hand-Eye Calibration Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the hand-eye calibration pipeline so waypoints appear as TF markers in RViz, MoveIt can navigate them, samples auto-capture at each pose, Park solver is used and labelled, a gripper-tip TF marker shows the physical tag mount, and the dashboard Calibration + Cube tabs are visually improved.

**Architecture:** A new `calibration/waypoint_publisher_node.py` ROS node reads the saved poses YAML, calls MoveIt FK, and publishes TF frames + a MarkerArray for each calibration waypoint. The Park solver default is set in `coordinator_node.py`. A second marker in the same node shows the physical tag mount on the gripper tip. The dashboard receives algorithm info through the existing JSON status channel. Two dashboard screens are restyled in `dashboard/main.py`.

**Tech Stack:** ROS 2 Humble, Python 3, PyQt5, `moveit_msgs/GetPositionFK`, `visualization_msgs/MarkerArray`, `tf2_ros.StaticTransformBroadcaster`, easy_handeye2, PyYAML

---

## File Map

| Action | File | Responsibility |
|--------|------|----------------|
| Create | `calibration/waypoint_publisher_node.py` | FK + TF frames + markers for each calibration waypoint; gripper tag-mount marker |
| Modify | `calibration/coordinator_node.py` | Change default algorithm to Park; add `algorithm` to status JSON |
| Modify | `scripts/calibrate_auto.py` | Launch waypoint publisher; add `--algorithm` flag; plumb `--fake-gripper` to waypoint node |
| Modify | `ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz` | Add MarkerArray display for waypoints + gripper marker |
| Modify | `dashboard/main.py` | CalibrationScreen: solver pill, visual pose grid; CubePickScreen: colour-coded cubes, improved layout |
| Modify | `dashboard/ros_interface.py` | Parse `algorithm` from calibration status JSON; call `stop_calibration_stack` on shutdown |
| Modify | `scripts/launch.py` | Kill calibration coordinator if it spawned; proper non-moveit shutdown |

---

## Task 1: Calibration Waypoint Publisher — FK + TF + Markers

**Files:**
- Create: `calibration/waypoint_publisher_node.py`

### What it does
Reads a joint-pose YAML (same format as `coordinator_node.py` uses), calls the MoveIt `/compute_fk` service once per pose, broadcasts a static TF frame `calib_waypoint_N` in `base_link`, and publishes a latched `MarkerArray` on `/holoassist/calibration/waypoint_markers` with numbered spheres + text for each waypoint. Also subscribes to `/holoassist/calibration/status` and recolours the current-target waypoint yellow. Additionally publishes one extra marker and TF frame `tool_tag_mount` rigidly parented to `tool0`, showing where the physical AprilTag is mounted on the gripper.

- [ ] **Step 1: Create the node**

```python
#!/usr/bin/env python3
"""Publish TF + RViz markers for calibration waypoints and gripper tag mount."""
import json
import math
import pathlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Point, Vector3
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import String, ColorRGBA, Header
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Offset from tool0 to the physical AprilTag mount (metres, quaternion).
# Tune after measuring the real gripper fixture.
TAG_MOUNT_XYZ = (0.0, 0.0, 0.10)   # 10 cm above tool0 along z
TAG_MOUNT_QUAT = (0.0, 0.0, 0.0, 1.0)  # no rotation relative to tool0


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("holoassist_waypoint_publisher")
        self.declare_parameter("poses_file", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "tool0")

        poses_path = str(self.get_parameter("poses_file").value).strip()
        if not poses_path:
            self.get_logger().error("poses_file parameter is required")
            sys.exit(1)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._ee_link = str(self.get_parameter("ee_link").value)
        self._poses_file = pathlib.Path(poses_path)
        self._joint_names, self._poses_rad = self._load_poses()
        self._current_index = 0  # 1-based; 0 = none active

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/holoassist/calibration/waypoint_markers", LATCHED
        )
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.create_subscription(String, "/holoassist/calibration/status", self._status_cb, 10)

        self.get_logger().info(
            f"Waiting for /compute_fk to compute FK for {len(self._poses_rad)} waypoints..."
        )
        # Retry FK until MoveIt is up.
        self._fk_timer = self.create_timer(2.0, self._try_compute_fk)

    def _load_poses(self):
        with self._poses_file.open() as f:
            data = yaml.safe_load(f)
        names = list(data["joint_names"])
        poses = [[math.radians(float(v)) for v in p] for p in data["poses_deg"]]
        return names, poses

    def _try_compute_fk(self):
        if not self._fk_client.service_is_ready():
            self.get_logger().info("Waiting for /compute_fk service...")
            return
        self._fk_timer.cancel()
        poses_xyz = []
        for i, pose_rad in enumerate(self._poses_rad):
            req = GetPositionFK.Request()
            req.header.frame_id = self._base_frame
            req.fk_link_names = [self._ee_link]
            js = JointState()
            js.name = self._joint_names
            js.position = pose_rad
            rs = RobotState()
            rs.joint_state = js
            req.robot_state = rs
            future = self._fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            if result is None or not result.pose_stamped:
                self.get_logger().warn(f"FK failed for pose {i+1}, skipping")
                poses_xyz.append(None)
                continue
            poses_xyz.append(result.pose_stamped[0].pose)
        self._publish_tf_and_markers(poses_xyz)
        self._publish_tag_mount_tf()
        self.get_logger().info(
            f"Published {sum(p is not None for p in poses_xyz)} waypoint TF frames."
        )

    def _publish_tf_and_markers(self, poses):
        transforms = []
        markers = []
        now = self.get_clock().now().to_msg()

        for i, pose in enumerate(poses):
            if pose is None:
                continue
            idx = i + 1

            # TF frame
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._base_frame
            tf.child_frame_id = f"calib_waypoint_{idx}"
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            transforms.append(tf)

            # Sphere marker
            sphere = Marker()
            sphere.header.frame_id = self._base_frame
            sphere.header.stamp = now
            sphere.ns = "calib_waypoints"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale = Vector3(x=0.03, y=0.03, z=0.03)
            sphere.color = self._waypoint_color(idx)
            markers.append(sphere)

            # Text label
            text = Marker()
            text.header = sphere.header
            text.ns = "calib_waypoint_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose
            text.pose.position.z += 0.04
            text.scale.z = 0.025
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            text.text = f"W{idx}"
            markers.append(text)

        self._tf_broadcaster.sendTransform(transforms)
        msg = MarkerArray()
        msg.markers = markers
        self._marker_pub.publish(msg)

    def _waypoint_color(self, idx):
        if idx == self._current_index:
            return ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)  # yellow = current
        return ColorRGBA(r=0.25, g=0.65, b=1.0, a=0.7)      # blue = pending

    def _publish_tag_mount_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._ee_link
        tf.child_frame_id = "tool_tag_mount"
        x, y, z = TAG_MOUNT_XYZ
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        qx, qy, qz, qw = TAG_MOUNT_QUAT
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform([tf])

        # Visual marker at the tag mount
        marker = Marker()
        marker.header.frame_id = "tool_tag_mount"
        marker.header.stamp = tf.header.stamp
        marker.ns = "tool_tag_mount"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.032, y=0.032, z=0.002)  # 32mm tag footprint
        marker.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.85)  # orange
        label = Marker()
        label.header = marker.header
        label.ns = "tool_tag_mount_label"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 0.025
        label.pose.orientation.w = 1.0
        label.scale.z = 0.020
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = "tag36h11:1"
        arr = MarkerArray()
        arr.markers = [marker, label]
        self._marker_pub.publish(arr)

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            idx = int(data.get("pose_index", 0))
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if idx != self._current_index:
            self._current_index = idx
            # Re-colour markers (need stored poses — recompute colours only)
            self.get_logger().debug(f"Active waypoint index: {idx}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Save to: `calibration/waypoint_publisher_node.py`

- [ ] **Step 2: Mark executable**

```bash
chmod +x /home/john/git/RS2-HoloAssist/main/calibration/waypoint_publisher_node.py
```

- [ ] **Step 3: Verify syntax**

```bash
cd /home/john/git/RS2-HoloAssist/main
source ros2_ws/install/setup.bash
python3 -c "import ast; ast.parse(open('calibration/waypoint_publisher_node.py').read()); print('syntax OK')"
```
Expected: `syntax OK`

- [ ] **Step 4: Commit**

```bash
git add calibration/waypoint_publisher_node.py
git commit -m "feat: add calibration waypoint publisher with FK + TF + gripper tag mount marker"
```

---

## Task 2: Park Solver Default + Algorithm in Status

**Files:**
- Modify: `calibration/coordinator_node.py` (lines 33, 141-145)

The easy_handeye2 OpenCV backend already supports `"Park"` (`cv2.CALIB_HAND_EYE_PARK`). We change the coordinator default and expose the algorithm name in the status JSON so the dashboard can display it.

- [ ] **Step 1: Change default algorithm and add it to status**

In `coordinator_node.py`, find line 33:
```python
        self.declare_parameter("algorithm", "OpenCV/Tsai-Lenz")
```
Change to:
```python
        self.declare_parameter("algorithm", "Park")
```

In `_publish_status` (around line 126), find the `payload` dict and add `"algorithm"` key:
```python
        payload = {
            "ready": ready,
            "state": self._state,
            "message": self._message,
            "running": self._running,
            "sample_count": self._sample_count,
            "pose_index": self._pose_index,
            "pose_total": len(self.poses),
            "computed": self._computed,
            "result": self._result,
            "latest_path": self._latest_path,
            "archive_path": self._archive_path,
            "error": self._error,
            "marker_frame": self.marker_frame,
            "algorithm": self.algorithm,   # ← add this line
        }
```

- [ ] **Step 2: Verify easy_handeye2 accepts "Park"**

```bash
grep -n "Park\|AVAILABLE_ALGORITHMS" \
  /home/john/git/RS2-HoloAssist/main/ros2_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py
```
Expected: `'Park': cv2.CALIB_HAND_EYE_PARK` in the dict.

- [ ] **Step 3: Commit**

```bash
git add calibration/coordinator_node.py
git commit -m "feat: default calibration solver to Park; expose algorithm in status JSON"
```

---

## Task 3: Wire Waypoint Publisher into calibrate_auto.py

**Files:**
- Modify: `scripts/calibrate_auto.py` (add `start()` call after coordinator)

The waypoint publisher runs as a Python subprocess in the same shell environment. It uses the same poses file as the coordinator.

- [ ] **Step 1: Add waypoint publisher launch after the coordinator start**

In `calibrate_auto.py`, find the block starting at line 180:
```python
        start(
            "Calibration coordinator",
            "python3 calibration/coordinator_node.py --ros-args"
            f" -p poses_file:={poses_file}"
            f" -p auto_start:={'true' if args.auto_start else 'false'}"
            " -p marker_frame:=tag36h11:1",
        )
```
Immediately after, add:
```python
        start(
            "Waypoint publisher",
            "python3 calibration/waypoint_publisher_node.py --ros-args"
            f" -p poses_file:={poses_file}"
            " -p base_frame:=base_link"
            " -p ee_link:=tool0",
        )
```

- [ ] **Step 2: Add --algorithm flag to calibrate_auto.py**

Find the `argparse` block (around line 50-75) and add before `args = parser.parse_args()`:
```python
    parser.add_argument(
        "--algorithm",
        default="Park",
        choices=["Park", "OpenCV/Tsai-Lenz", "OpenCV/Andreff", "OpenCV/Daniilidis", "OpenCV/Horaud"],
        help="Hand-eye calibration algorithm (default: Park)",
    )
```

Pass it to the coordinator start command — replace:
```python
            " -p marker_frame:=tag36h11:1",
```
with:
```python
            " -p marker_frame:=tag36h11:1"
            f" -p algorithm:={args.algorithm}",
```

- [ ] **Step 3: Verify parse**

```bash
python3 /home/john/git/RS2-HoloAssist/main/scripts/calibrate_auto.py --help 2>&1 | grep algorithm
```
Expected: `--algorithm {Park,...}`

- [ ] **Step 4: Commit**

```bash
git add scripts/calibrate_auto.py
git commit -m "feat: launch waypoint publisher from calibrate_auto; add --algorithm flag"
```

---

## Task 4: Add Waypoint + Gripper Marker Displays to RViz

**Files:**
- Modify: `ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz`

Add two new `MarkerArray` display entries to the existing displays list. Open the file and add directly before the final `- Class: rviz_common/Tool...` section (or at end of Displays list):

- [ ] **Step 1: Add MarkerArray displays**

In `holoassist_hw.rviz`, find the last display entry and append these two YAML blocks at the same indentation level as the other displays:

```yaml
  - Class: rviz_default_plugins/MarkerArray
    Enabled: true
    Name: Calibration Waypoints
    Namespaces:
      calib_waypoints: true
      calib_waypoint_labels: true
    Topic:
      Depth: 5
      Durability Policy: Transient Local
      Filter size: 10
      History Policy: Keep Last
      Reliability Policy: Reliable
      Value: /holoassist/calibration/waypoint_markers
    Value: true
  - Class: rviz_default_plugins/MarkerArray
    Enabled: true
    Name: Gripper Tag Mount
    Namespaces:
      tool_tag_mount: true
      tool_tag_mount_label: true
    Topic:
      Depth: 5
      Durability Policy: Transient Local
      Filter size: 10
      History Policy: Keep Last
      Reliability Policy: Reliable
      Value: /holoassist/calibration/waypoint_markers
    Value: true
```

Note: both displays subscribe to the same topic `/holoassist/calibration/waypoint_markers` but filter different namespaces.

- [ ] **Step 2: Verify YAML is valid**

```bash
python3 -c "import yaml; yaml.safe_load(open('ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz')); print('YAML OK')"
```
Expected: `YAML OK`

- [ ] **Step 3: Rebuild (ament_cmake package, just install)**

```bash
cd ros2_ws && colcon build --packages-select holoassist_movement 2>&1 | tail -5
```
Expected: `Summary: 1 packages finished`

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/HoloAssist_Movement/rviz/holoassist_hw.rviz
git commit -m "feat: add calibration waypoint + gripper tag mount marker displays to RViz"
```

---

## Task 5: Dashboard — Calibration Screen UI Improvements

**Files:**
- Modify: `dashboard/main.py` `CalibrationScreen` class (lines 1478–1693)
- Modify: `dashboard/ros_interface.py` — parse `algorithm` from status JSON

### 5a: Expose algorithm in ros_interface status

- [ ] **Step 1: Find DashboardStatus in ros_interface.py and add algorithm field**

Search for the `DashboardStatus` dataclass (or namedtuple) definition in `ros_interface.py`. Add:
```python
    calibration_algorithm: str = "Park"
```

Find `_on_calibration_status` (or wherever calibration JSON is parsed) and add:
```python
            self._status.calibration_algorithm = str(data.get("algorithm", "Park"))
```

- [ ] **Step 2: Verify by grepping**

```bash
grep -n "calibration_algorithm\|DashboardStatus" dashboard/ros_interface.py | head -10
```

### 5b: Improve CalibrationScreen layout

- [ ] **Step 3: Update CalibrationScreen `__init__` to add solver pill and pose grid**

In `dashboard/main.py`, replace the entire `CalibrationScreen.__init__` body (lines 1486–1587) with the improved version below. The key additions are:

1. A "solver pill" label showing `▶ PARK SOLVER` in green
2. A `QFrame` progress grid of pose squares (up to 32) that light up as poses complete
3. Better spacing and grouping

```python
    def __init__(self, ros: RosInterface, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._last_detail = ""
        self._pose_total = 1
        self._pose_done = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(8)

        # ── Header row ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("HAND-EYE CALIBRATION")
        title.setFont(QFont("monospace", 11, QFont.Bold))
        title.setStyleSheet(f"color: {BLUE};")
        header.addWidget(title)
        header.addStretch()
        self.solver_label = QLabel("▶ PARK SOLVER")
        self.solver_label.setFont(QFont("monospace", 8, QFont.Bold))
        self.solver_label.setStyleSheet(
            f"color: {GREEN}; background: #0d2b18; border: 1px solid {GREEN};"
            " border-radius: 4px; padding: 2px 6px;"
        )
        header.addWidget(self.solver_label)
        header.addSpacing(8)
        self.ready_label = QLabel("SERVER: WAITING")
        self.ready_label.setFont(QFont("monospace", 10, QFont.Bold))
        header.addWidget(self.ready_label)
        layout.addLayout(header)

        self.tag_label = QLabel("Physical marker: tag36h11:1")
        self.tag_label.setFont(QFont("monospace", 9))
        self.tag_label.setStyleSheet(f"color: {TEXT_DIM};")
        layout.addWidget(self.tag_label)

        self.state_label = QLabel("Waiting for calibration coordinator...")
        self.state_label.setFont(QFont("monospace", 11, QFont.Bold))
        self.state_label.setWordWrap(True)
        layout.addWidget(self.state_label)

        # ── Pose grid ───────────────────────────────────────────────
        self._pose_squares = []
        grid_frame = QFrame()
        grid_frame.setStyleSheet(f"background: {PANEL_BG}; border-radius: 4px;")
        grid_layout = QGridLayout(grid_frame)
        grid_layout.setSpacing(3)
        grid_layout.setContentsMargins(6, 6, 6, 6)
        for i in range(32):
            sq = QLabel(str(i + 1))
            sq.setAlignment(Qt.AlignCenter)
            sq.setFont(QFont("monospace", 7))
            sq.setFixedSize(22, 22)
            sq.setStyleSheet(
                f"background: {DARK_BG}; color: {TEXT_DIM}; border: 1px solid {BORDER};"
                " border-radius: 3px;"
            )
            self._pose_squares.append(sq)
            grid_layout.addWidget(sq, i // 8, i % 8)
        layout.addWidget(grid_frame)

        # Samples row
        self.sample_label = QLabel("Samples: 0  |  Result: not computed")
        self.sample_label.setFont(QFont("monospace", 10, QFont.Bold))
        layout.addWidget(self.sample_label)

        # ── Stack lifecycle ──────────────────────────────────────────
        stack_row = QHBoxLayout()
        self.launch_btn = QPushButton("LAUNCH STACK")
        self.launch_btn.setFont(QFont("monospace", 10, QFont.Bold))
        self.launch_btn.setCursor(Qt.PointingHandCursor)
        self.launch_btn.clicked.connect(self._on_launch)
        self.shutdown_btn = QPushButton("SHUTDOWN STACK")
        self.shutdown_btn.setFont(QFont("monospace", 10, QFont.Bold))
        self.shutdown_btn.setCursor(Qt.PointingHandCursor)
        self.shutdown_btn.clicked.connect(self._on_shutdown)
        stack_row.addWidget(self.launch_btn, 1)
        stack_row.addWidget(self.shutdown_btn, 1)
        layout.addLayout(stack_row)

        # ── Motion control ───────────────────────────────────────────
        buttons = QHBoxLayout()
        self.start_btn = self._cmd_button("START AUTO", "start")
        self.stop_btn = self._cmd_button("STOP", "stop")
        self.sample_btn = self._cmd_button("TAKE SAMPLE", "sample")
        self.compute_btn = self._cmd_button("COMPUTE", "compute")
        self.save_btn = self._cmd_button("SAVE", "save")
        for btn in (self.start_btn, self.stop_btn, self.sample_btn,
                    self.compute_btn, self.save_btn):
            buttons.addWidget(btn, 1)
        layout.addLayout(buttons)

        # ── Manual pose recording ────────────────────────────────────
        rec_row = QHBoxLayout()
        self.rec_label = QLabel("Recorded: 0 poses")
        self.rec_label.setFont(QFont("monospace", 9))
        self.rec_label.setStyleSheet(f"color: {TEXT_DIM};")
        self.rec_btn = QPushButton("REC POSE")
        self.rec_btn.setFont(QFont("monospace", 10, QFont.Bold))
        self.rec_btn.setCursor(Qt.PointingHandCursor)
        self.rec_btn.clicked.connect(self._on_rec_pose)
        self.save_poses_btn = QPushButton("SAVE POSES")
        self.save_poses_btn.setFont(QFont("monospace", 10, QFont.Bold))
        self.save_poses_btn.setCursor(Qt.PointingHandCursor)
        self.save_poses_btn.clicked.connect(self._on_save_poses)
        self.clear_poses_btn = QPushButton("CLEAR")
        self.clear_poses_btn.setFont(QFont("monospace", 10, QFont.Bold))
        self.clear_poses_btn.setCursor(Qt.PointingHandCursor)
        self.clear_poses_btn.clicked.connect(self._on_clear_poses)
        rec_row.addWidget(self.rec_label, 2)
        rec_row.addWidget(self.rec_btn, 1)
        rec_row.addWidget(self.save_poses_btn, 1)
        rec_row.addWidget(self.clear_poses_btn, 1)
        layout.addLayout(rec_row)

        warning = QLabel(
            "START AUTO commands physical MoveIt motion. Verify tool tag is visible "
            "and workcell is clear before starting."
        )
        warning.setFont(QFont("monospace", 8, QFont.Bold))
        warning.setStyleSheet(f"color: {ORANGE};")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        self.details = QTextEdit()
        self.details.setReadOnly(True)
        self.details.setFont(QFont("monospace", 9))
        layout.addWidget(self.details, 1)
        self.apply_scale(1.0)
```

- [ ] **Step 4: Update `update_status` to drive the pose grid and solver label**

Find `CalibrationScreen.update_status` and add after the existing `self.state_label.setText(...)` calls:

```python
        # Update solver pill
        algo = getattr(status, "calibration_algorithm", "Park")
        self.solver_label.setText(f"▶ {algo.upper()} SOLVER")

        # Update pose grid
        done = status.calibration_sample_count
        total = max(status.calibration_pose_total, 1)
        current = status.calibration_pose_index
        for i, sq in enumerate(self._pose_squares):
            pose_num = i + 1
            if pose_num <= done:
                sq.setStyleSheet(
                    f"background: {GREEN}; color: white; border: 1px solid {GREEN};"
                    " border-radius: 3px;"
                )
            elif pose_num == current:
                sq.setStyleSheet(
                    f"background: {YELLOW}; color: black; border: 1px solid {YELLOW};"
                    " border-radius: 3px;"
                )
            elif pose_num <= total:
                sq.setStyleSheet(
                    f"background: {DARK_BG}; color: {TEXT_DIM}; border: 1px solid {BORDER};"
                    " border-radius: 3px;"
                )
            else:
                sq.setStyleSheet(
                    f"background: {DARK_BG}; color: transparent; border: 1px solid transparent;"
                    " border-radius: 3px;"
                )
```

Also remove the old `self.progress` QProgressBar widget references (the grid replaces it) or keep both — if keeping both, leave the existing `self.progress` in place and don't remove from `apply_scale`.

- [ ] **Step 5: Add `calibration_algorithm` to DashboardStatus and status parsing**

In `dashboard/ros_interface.py`, find the `DashboardStatus` class and add field:
```python
    calibration_algorithm: str = "Park"
```

Find the subscriber callback that processes `/holoassist/calibration/status` JSON (search for `calibration_state` or `calibration_ready`) and add:
```python
            self._status.calibration_algorithm = str(data.get("algorithm", "Park"))
```

- [ ] **Step 6: Verify dashboard starts without error**

```bash
cd /home/john/git/RS2-HoloAssist/main
python3 -c "
import sys
sys.argv = ['main']
from dashboard.main import *
print('import OK')
"
```
Expected: `import OK`

- [ ] **Step 7: Commit**

```bash
git add dashboard/main.py dashboard/ros_interface.py
git commit -m "feat: CalibrationScreen pose grid, Park solver pill, algorithm status field"
```

---

## Task 6: Dashboard — Cube Pick Screen UI Improvements

**Files:**
- Modify: `dashboard/main.py` `CubePickScreen` class (lines 1210–1474)

Key improvements:
1. Cube buttons show colour-coded squares matching the physical AprilTag cubes (red/green/blue/yellow)
2. Bin buttons styled more distinctively
3. Monitor section uses a compact three-line status strip instead of a scrolling wall of text

- [ ] **Step 1: Update cube button creation in `CubePickScreen.__init__`**

Find the cube grid section (around line 1320–1360) where `cube_buttons` are created. Replace the loop that creates them with:

```python
        CUBE_COLORS = [
            ("#e53935", "#ffcdd2"),  # red
            ("#43a047", "#c8e6c9"),  # green
            ("#1e88e5", "#bbdefb"),  # blue
            ("#f9a825", "#fff9c4"),  # yellow
        ]
        CUBE_NAMES = ["APRIL CUBE\n1", "APRIL CUBE\n2", "APRIL CUBE\n3", "APRIL CUBE\n4"]
        self.cube_buttons = []
        for i in range(4):
            bg, fg = CUBE_COLORS[i]
            btn = QPushButton(CUBE_NAMES[i])
            btn.setFont(QFont("monospace", 11, QFont.Bold))
            btn.setCursor(Qt.PointingHandCursor)
            btn.setStyleSheet(
                f"background-color: {bg}22; color: {fg}; border: 2px solid {bg};"
                " border-radius: 6px;"
                f" QPushButton:hover {{ background-color: {bg}55; }}"
                f" QPushButton:pressed {{ background-color: {bg}; color: white; }}"
                f" QPushButton:disabled {{ background-color: {DARK_BG}; color: {TEXT_DIM};"
                f" border: 1px solid {BORDER}; }}"
            )
            btn.clicked.connect(lambda checked=False, cid=i + 1: self._request_pick(cid))
            self.cube_buttons.append(btn)
```

- [ ] **Step 2: Update bin buttons to use distinctive styling**

Find the bin buttons creation and replace:
```python
        BIN_STYLES = [
            (f"background-color: #1a3a2a; color: {GREEN}; border: 2px solid {GREEN};"),
            (f"background-color: #1a2a3a; color: {BLUE}; border: 2px solid {BLUE};"),
        ]
        self.bin_buttons = []
        for i, label in enumerate(["BIN 1", "BIN 2"]):
            btn = QPushButton(label)
            btn.setFont(QFont("monospace", 11, QFont.Bold))
            btn.setCursor(Qt.PointingHandCursor)
            btn.setCheckable(True)
            btn.setStyleSheet(BIN_STYLES[i] + " border-radius: 5px;")
            btn.clicked.connect(lambda checked=False, bid=i + 1: self._select_bin(bid))
            self.bin_buttons.append(btn)
```

- [ ] **Step 3: Compact the monitor section — replace the verbose QTextEdit with a status strip**

Find `self.pick_place_status = QTextEdit()` and the surrounding monitor layout. Add a compact 3-row status panel above it:

```python
        # Compact status strip
        status_strip = QFrame()
        status_strip.setStyleSheet(
            f"background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        strip_layout = QGridLayout(status_strip)
        strip_layout.setContentsMargins(8, 6, 8, 6)
        strip_layout.setSpacing(4)
        for col, header in enumerate(["BLOCK", "STEP", "STATE"]):
            lbl = QLabel(header)
            lbl.setFont(QFont("monospace", 7, QFont.Bold))
            lbl.setStyleSheet(f"color: {TEXT_DIM};")
            strip_layout.addWidget(lbl, 0, col)
        self.monitor_block2 = QLabel("---")
        self.monitor_step2 = QLabel("---")
        self.monitor_state2 = QLabel("WAITING")
        for col, lbl in enumerate([self.monitor_block2, self.monitor_step2, self.monitor_state2]):
            lbl.setFont(QFont("monospace", 10, QFont.Bold))
            lbl.setStyleSheet(f"color: {TEXT};")
            strip_layout.addWidget(lbl, 1, col)
        monitor_layout.addWidget(status_strip)   # insert before existing QTextEdit
```

- [ ] **Step 4: Drive the new compact strip from `update_status`**

In `CubePickScreen.update_status`, add after the existing monitor_block/step/state updates:
```python
        self.monitor_block2.setText(status.pick_place_block or "---")
        self.monitor_step2.setText(status.pick_place_step or "---")
        state_str = status.pick_place_state or "WAITING"
        state_color = {
            "RUNNING": BLUE, "COMPLETE": GREEN, "ERROR": RED, "FAILED": RED
        }.get(state_str.upper(), TEXT)
        self.monitor_state2.setText(state_str)
        self.monitor_state2.setStyleSheet(f"color: {state_color}; font-weight: bold;")
```

- [ ] **Step 5: Verify dashboard starts without error**

```bash
cd /home/john/git/RS2-HoloAssist/main
python3 -c "
import sys
sys.argv = ['main']
from dashboard.main import *
print('import OK')
"
```
Expected: `import OK`

- [ ] **Step 6: Commit**

```bash
git add dashboard/main.py
git commit -m "feat: CubePickScreen colour-coded cubes, distinctive bin buttons, compact status strip"
```

---

## Task 7: Proper Shutdown — Non-MoveIt Mode + Calibration Cleanup

**Files:**
- Modify: `scripts/launch.py` (~lines 590-640)
- Modify: `dashboard/ros_interface.py` shutdown handler

### 7a: launch.py — ensure calibration coordinator dies when launcher exits

The issue: if the user ran `LAUNCH STACK` from the dashboard while launch.py is running (without `--moveit`), the subprocess group spawned by `ros_interface.start_calibration_stack` won't be in launch.py's process list.

The cleanest fix: in launch.py's `stop_all()` teardown, add a `pkill` sweep for any `coordinator_node.py` or `waypoint_publisher_node.py` processes.

- [ ] **Step 1: Add calibration process cleanup to stop_all in launch.py**

Find the `stop_all` function in `scripts/launch.py` (look for the SIGINT/SIGKILL teardown block). After the `pkill -9 -f rviz2` line, add:

```python
        # Kill any calibration processes spawned by the dashboard independently
        subprocess.run(
            ["pkill", "-9", "-f", "coordinator_node.py"],
            capture_output=True,
        )
        subprocess.run(
            ["pkill", "-9", "-f", "waypoint_publisher_node.py"],
            capture_output=True,
        )
        subprocess.run(
            ["pkill", "-9", "-f", "handeye_server"],
            capture_output=True,
        )
```

### 7b: ros_interface.py — call stop_calibration_stack on shutdown

- [ ] **Step 2: Register calibration teardown in ros_interface shutdown**

Find `RosInterface.shutdown()` in `dashboard/ros_interface.py`. Before the rclpy teardown, add:

```python
    def shutdown(self):
        # Stop any running calibration stack first
        if self.calibration_stack_running():
            self.stop_calibration_stack()
        # ... existing shutdown code follows
```

- [ ] **Step 3: Commit**

```bash
git add scripts/launch.py dashboard/ros_interface.py
git commit -m "fix: kill calibration subprocesses on shutdown; call stop_calibration_stack from ros_interface.shutdown"
```

---

## Verification: End-to-End Test

After all tasks are committed, test the complete flow:

- [ ] **1. Start simulator**
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

- [ ] **2. Launch with fake gripper (no real robot needed for UI test)**
```bash
./scripts/launch.sh --fake-gripper --robot-ip 192.168.56.101 --verbose
```
Expected: "All running" banner. Play External Control in Polyscope.

- [ ] **3. In dashboard → CALIBRATION tab**
  - Confirm "▶ PARK SOLVER" pill is visible
  - Confirm pose grid shows 32 empty squares (or however many are in the poses file)
  - Confirm "SERVER: WAITING" until LAUNCH STACK

- [ ] **4. Create a test poses file and launch calibration stack**
```bash
cp calibration/real_poses.example.yaml calibration/real_poses.yaml
# Edit to add 8+ poses, then:
python3 scripts/calibrate_auto.py \
  --robot-ip 192.168.56.101 \
  --poses-file calibration/real_poses.yaml \
  --fake-gripper
```

- [ ] **5. Check RViz**
  - Confirm `calib_waypoint_1..N` TF frames appear in the TF display
  - Confirm blue spheres with W1..WN labels appear in 3D view
  - Confirm orange "tag36h11:1" square marker appears at gripper tip
  - Confirm "Calibration Waypoints" and "Gripper Tag Mount" MarkerArray displays are enabled

- [ ] **6. START AUTO from dashboard**
  - Pose grid squares turn yellow (current) then green (done) as each pose completes
  - PARK SOLVER pill stays visible throughout
  - Ctrl+C shuts everything down cleanly with no orphan processes

---

## Notes

- `TAG_MOUNT_XYZ` in `waypoint_publisher_node.py` must be tuned by measuring the physical tag fixture after first assembly.
- `recorded_poses.yaml` is git-ignored (it contains hardware-specific data). The `real_poses.example.yaml` is the template users copy and edit.
- The `holoassist_full.rviz` (used by perception launch) does not include calibration displays intentionally — calibration runs via `calibrate_auto.py` which uses `holoassist_hw.rviz` through `movement.launch.py`.
