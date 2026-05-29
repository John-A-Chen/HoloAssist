#!/usr/bin/env python3
"""Publish TF + RViz markers for calibration waypoints and gripper tag mount."""
import json
import math
import pathlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Vector3
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import String, ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Offset from tool0 to the physical AprilTag mount (metres, quaternion xyzw).
# Tune after measuring the real gripper fixture.
TAG_MOUNT_XYZ = (0.0, 0.0, 0.10)    # 10 cm above tool0 along z
TAG_MOUNT_QUAT = (0.0, 0.0, 0.0, 1.0)


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
        self._current_index = 0   # 1-based; 0 = none active
        self._cached_poses = []   # list of geometry_msgs/Pose or None per waypoint

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/holoassist/calibration/waypoint_markers", LATCHED
        )
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.create_subscription(
            String, "/holoassist/calibration/status", self._status_cb, 10
        )

        self.get_logger().info(
            f"Waiting for /compute_fk to compute FK for {len(self._poses_rad)} waypoints..."
        )
        self._fk_timer = self.create_timer(2.0, self._try_compute_fk)

    # ── Pose loading ────────────────────────────────────────────────────────

    def _load_poses(self):
        with self._poses_file.open() as f:
            data = yaml.safe_load(f)
        names = list(data["joint_names"])
        poses = [[math.radians(float(v)) for v in p] for p in data["poses_deg"]]
        return names, poses

    # ── FK computation ──────────────────────────────────────────────────────

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
                continue
            poses_xyz.append(result.pose_stamped[0].pose)

        self._cached_poses = poses_xyz
        self._publish_tf_and_markers()
        self._publish_tag_mount_tf()
        good = sum(p is not None for p in poses_xyz)
        self.get_logger().info(f"Published {good}/{len(poses_xyz)} waypoint TF frames.")

    # ── TF + marker publishing ──────────────────────────────────────────────

    def _publish_tf_and_markers(self):
        transforms = []
        markers = []
        now = self.get_clock().now().to_msg()

        for i, pose in enumerate(self._cached_poses):
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
        msg = MarkerArray()
        msg.markers = markers
        self._marker_pub.publish(msg)

    def _waypoint_color(self, idx):
        if idx == self._current_index:
            return ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)   # yellow = current
        return ColorRGBA(r=0.25, g=0.65, b=1.0, a=0.7)       # blue = pending

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

        now = tf.header.stamp

        # Orange square representing the 32 mm tag footprint
        cube_m = Marker()
        cube_m.header.frame_id = "tool_tag_mount"
        cube_m.header.stamp = now
        cube_m.ns = "tool_tag_mount"
        cube_m.id = 0
        cube_m.type = Marker.CUBE
        cube_m.action = Marker.ADD
        cube_m.pose.orientation.w = 1.0
        cube_m.scale = Vector3(x=0.032, y=0.032, z=0.002)
        cube_m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.85)

        label_m = Marker()
        label_m.header.frame_id = "tool_tag_mount"
        label_m.header.stamp = now
        label_m.ns = "tool_tag_mount_label"
        label_m.id = 1
        label_m.type = Marker.TEXT_VIEW_FACING
        label_m.action = Marker.ADD
        label_m.pose.position.z = 0.025
        label_m.pose.orientation.w = 1.0
        label_m.scale.z = 0.020
        label_m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label_m.text = "tag36h11:1"

        arr = MarkerArray()
        arr.markers = [cube_m, label_m]
        self._marker_pub.publish(arr)

    # ── Status subscriber ───────────────────────────────────────────────────

    def _status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            idx = int(data.get("pose_index", 0))
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if idx != self._current_index:
            self._current_index = idx
            if self._cached_poses:
                self._publish_tf_and_markers()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
