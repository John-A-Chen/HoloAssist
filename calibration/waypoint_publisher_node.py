#!/usr/bin/env python3
"""Publish TF + RViz markers for calibration waypoints and gripper tag mount.

Topic: /holoassist/scene/calibration_poses  (latched MarkerArray, same pattern as
       /holoassist/scene/trolley)

Launched automatically by launch.py alongside the main stack.  If no poses file is
found the node still publishes the gripper tag-mount marker so operators can always
see where the AprilTag should be mounted.

Default poses-file search order (first found wins):
  1. calibration/recorded_poses.yaml   (saved via dashboard REC POSE → SAVE POSES)
"""
import json
import math
import pathlib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Vector3, Pose
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import String, ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml

TOPIC = "/holoassist/scene/calibration_poses"

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Offset from tool0 → physical AprilTag mount (metres, quaternion xyzw).
# Tune after measuring the real gripper fixture.
TAG_MOUNT_XYZ = (0.0, 0.0, 0.10)    # 10 cm above tool0 along z

_DEFAULT_SEARCH = [
    "calibration/recorded_poses.yaml",
]

# Time(0) for all marker stamps — tells RViz "use latest available TF"
# so no staleness-based flicker even when joint state updates briefly stall.
_TIME_ZERO = rclpy.time.Time().to_msg()


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("holoassist_calibration_poses_publisher")
        self.declare_parameter("poses_file", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "tool0")

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._ee_link = str(self.get_parameter("ee_link").value)

        # Resolve poses file: explicit param → default search paths → None
        poses_path = str(self.get_parameter("poses_file").value).strip()
        self._poses_file = self._resolve_poses_file(poses_path)

        self._joint_names = []
        self._poses_rad = []
        self._poses_cart = []   # list of Pose (base_link frame) or None
        if self._poses_file is not None:
            self._joint_names, self._poses_rad, self._poses_cart = self._load_poses(self._poses_file)
            self.get_logger().info(
                f"Loaded {len(self._poses_rad)} calibration poses from {self._poses_file}"
            )
        else:
            self.get_logger().info(
                "No poses file found — publishing gripper tag-mount marker only. "
                "Save poses with the dashboard REC POSE → SAVE POSES buttons."
            )

        self._current_index = 0
        # _cached_poses: list of Pose (base_link frame) or None, one per waypoint
        self._cached_poses = []

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._marker_pub = self.create_publisher(MarkerArray, TOPIC, LATCHED)
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.create_subscription(
            String, "/holoassist/calibration/status", self._status_cb, 10
        )

        has_saved_cart = any(c is not None for c in self._poses_cart)
        if has_saved_cart:
            # Cartesian positions saved at record time — use them directly
            self._cached_poses = self._poses_cart
            self._publish_all()
            good = sum(c is not None for c in self._cached_poses)
            self.get_logger().info(
                f"Using saved Cartesian poses: {good}/{len(self._cached_poses)} waypoints"
            )
        elif self._poses_rad:
            # Joint-only file — need FK to resolve positions
            self.get_logger().info("No Cartesian poses in file — waiting for /compute_fk…")
            self._fk_timer = self.create_timer(2.0, self._try_compute_fk)
            self._publish_all()   # publishes tag mount even before FK resolves
        else:
            self._publish_all()   # tag mount only
            self.get_logger().info(f"Calibration poses visible at topic: {TOPIC}")

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _resolve_poses_file(self, explicit: str) -> pathlib.Path | None:
        if explicit:
            p = pathlib.Path(explicit)
            return p if p.exists() else None
        root = pathlib.Path(__file__).resolve().parent.parent
        for rel in _DEFAULT_SEARCH:
            p = root / rel
            if p.exists():
                return p
        return None

    def _load_poses(self, path: pathlib.Path):
        with path.open() as f:
            data = yaml.safe_load(f)
        names = list(data["joint_names"])
        poses_rad = [[math.radians(float(v)) for v in row] for row in data["poses_deg"]]
        raw_cart = data.get("poses_cart", [])
        poses_cart = []
        for i, c in enumerate(raw_cart):
            if c is None:
                poses_cart.append(None)
            else:
                pose = Pose()
                pose.position.x = float(c["x"])
                pose.position.y = float(c["y"])
                pose.position.z = float(c["z"])
                pose.orientation.x = float(c["qx"])
                pose.orientation.y = float(c["qy"])
                pose.orientation.z = float(c["qz"])
                pose.orientation.w = float(c["qw"])
                poses_cart.append(pose)
        # Pad to match length of poses_rad
        while len(poses_cart) < len(poses_rad):
            poses_cart.append(None)
        return names, poses_rad, poses_cart

    # ── FK computation (fallback for joint-only files) ───────────────────────

    def _try_compute_fk(self):
        if not self._fk_client.service_is_ready():
            self.get_logger().debug("Waiting for /compute_fk…")
            return
        self._fk_timer.cancel()
        poses_xyz = []
        for i, pose_rad in enumerate(self._poses_rad):
            req = GetPositionFK.Request()
            req.header.frame_id = self._base_frame
            req.fk_link_names = [self._ee_link]
            js = JointState()
            js.name = self._joint_names
            js.position = list(pose_rad)
            rs = RobotState()
            rs.joint_state = js
            req.robot_state = rs
            future = self._fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            if result is None or not result.pose_stamped:
                self.get_logger().warn(f"FK failed for pose {i + 1}, skipping")
                poses_xyz.append(None)
            else:
                poses_xyz.append(result.pose_stamped[0].pose)

        self._cached_poses = poses_xyz
        self._publish_all()
        good = sum(p is not None for p in poses_xyz)
        self.get_logger().info(
            f"Calibration poses ready: {good}/{len(poses_xyz)} waypoints on {TOPIC}"
        )

    # ── Marker publishing ────────────────────────────────────────────────────

    def _publish_all(self):
        """Publish waypoint markers + tag-mount marker in a single MarkerArray.

        Publishing everything together on the LATCHED topic ensures that even
        when the active waypoint index changes and triggers a re-publish, the
        tag-mount marker is never erased from the retained message.
        All markers use Time(0) stamps so RViz uses the latest available TF
        transform — no staleness flicker when joint-state updates briefly stall.
        """
        transforms = []
        markers = []

        for i, pose in enumerate(self._cached_poses):
            if pose is None:
                continue
            idx = i + 1

            tf = TransformStamped()
            tf.header.stamp = _TIME_ZERO
            tf.header.frame_id = self._base_frame
            tf.child_frame_id = f"calib_waypoint_{idx}"
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            transforms.append(tf)

            sphere = Marker()
            sphere.header.frame_id = self._base_frame
            sphere.header.stamp = _TIME_ZERO
            sphere.ns = "calib_waypoints"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale = Vector3(x=0.03, y=0.03, z=0.03)
            sphere.color = self._waypoint_color(idx)
            markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "calib_waypoint_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.04
            text.pose.orientation.w = 1.0
            text.scale.z = 0.025
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            text.text = f"W{idx}"
            markers.append(text)

        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

        # Tag-mount marker — always included so it survives re-publishes.
        # Published in ee_link (tool0) frame. Time(0) avoids staleness flicker.
        x, y, z = TAG_MOUNT_XYZ
        cube_m = Marker()
        cube_m.header.frame_id = self._ee_link
        cube_m.header.stamp = _TIME_ZERO
        cube_m.ns = "tool_tag_mount"
        cube_m.id = 0
        cube_m.type = Marker.CUBE
        cube_m.action = Marker.ADD
        cube_m.pose.position.x = x
        cube_m.pose.position.y = y
        cube_m.pose.position.z = z
        cube_m.pose.orientation.w = 1.0
        cube_m.scale = Vector3(x=0.032, y=0.032, z=0.002)
        cube_m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.85)
        markers.append(cube_m)

        label_m = Marker()
        label_m.header.frame_id = self._ee_link
        label_m.header.stamp = _TIME_ZERO
        label_m.ns = "tool_tag_mount_label"
        label_m.id = 1
        label_m.type = Marker.TEXT_VIEW_FACING
        label_m.action = Marker.ADD
        label_m.pose.position.x = x
        label_m.pose.position.y = y
        label_m.pose.position.z = z + 0.025
        label_m.pose.orientation.w = 1.0
        label_m.scale.z = 0.020
        label_m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label_m.text = "tag36h11:1"
        markers.append(label_m)

        msg = MarkerArray()
        msg.markers = markers
        self._marker_pub.publish(msg)

    def _waypoint_color(self, idx):
        if idx == self._current_index:
            return ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)
        return ColorRGBA(r=0.25, g=0.65, b=1.0, a=0.7)

    # ── Status subscriber ────────────────────────────────────────────────────

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            idx = int(data.get("pose_index", 0))
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if idx != self._current_index:
            self._current_index = idx
            self._publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
