"""
Publishes the HoloAssist trolley mesh as a latched Marker in the `world` frame.

Mesh: UR3eTrolley(1).dae (Blender export, Z_UP, unit=meter, internal scale 0.01
      = mesh vertices are in cm). OGRE applies the COLLADA node transform so
      scale=1.0 on the marker gives the correct physical size.

Published at z=0 in the world frame. Adjust _Z_OFFSET if the mesh needs
vertical repositioning relative to the robot.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Duration


_MESH_URI = "package://holo_assist_depth_tracker/worlds/UR3eTrolley(1).dae"

# Adjust these to reposition/rotate the trolley in the world frame.
_X_OFFSET = 0.0
_Y_OFFSET = 0.20
_Z_OFFSET = -0.05
_YAW_DEG  = 180.0    # rotation around Z axis (degrees)


def _deleteall_marker(stamp) -> Marker:
    m = Marker()
    m.header.frame_id = "world"
    m.header.stamp = stamp
    m.ns = "trolley"
    m.id = 999
    m.action = Marker.DELETEALL
    return m


def _build_trolley_marker(stamp) -> Marker:
    m = Marker()
    m.header.frame_id = "world"
    m.header.stamp = stamp
    m.ns = "trolley"
    m.id = 0
    m.type = Marker.MESH_RESOURCE
    m.action = Marker.ADD
    m.mesh_resource = _MESH_URI
    m.mesh_use_embedded_materials = True
    m.pose.position.x = _X_OFFSET
    m.pose.position.y = _Y_OFFSET
    m.pose.position.z = _Z_OFFSET
    half = math.radians(_YAW_DEG) / 2.0
    m.pose.orientation.z = math.sin(half)
    m.pose.orientation.w = math.cos(half)
    m.scale = Vector3(x=1.0, y=1.0, z=1.0)
    m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
    m.lifetime = Duration(sec=0, nanosec=0)
    return m


class TrolleyScenePublisher(Node):
    def __init__(self):
        super().__init__("trolley_scene_publisher")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, "/holoassist/scene/trolley", qos)

        stamp = self.get_clock().now().to_msg()
        # DELETEALL first to clear any previously persisted markers (e.g. old box primitives)
        msg = MarkerArray()
        msg.markers = [_deleteall_marker(stamp), _build_trolley_marker(stamp)]

        self._pub.publish(msg)
        self.get_logger().info(
            f"Published trolley mesh marker on /holoassist/scene/trolley ({_MESH_URI})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrolleyScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
