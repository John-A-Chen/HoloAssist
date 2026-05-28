#!/usr/bin/env python3
"""
Publishes the HoloAssist trolley in two forms:
  1. Latched visualization Marker (world frame) — for RViz display
  2. MoveIt PlanningScene CollisionObject (convex hull) — for collision avoidance

The marker stays alive via TRANSIENT_LOCAL so late-joining RViz instances
receive it. The MoveIt planning scene is applied once move_group is ready.
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point, Pose
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle

_MESH_URI = "package://holoassist_perception/worlds/UR3eTrolley_decimated.dae"
_X, _Y, _Z = 0.0, 0.20, -1.06
_YAW_DEG = 180.0


def _yaw_pose(x, y, z, yaw_deg) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    half = math.radians(yaw_deg) / 2.0
    p.orientation.z = math.sin(half)
    p.orientation.w = math.cos(half)
    return p


def _load_collision_mesh(dae_path: str) -> Mesh:
    """Load DAE, compute convex hull, return as ROS Mesh (keeps collision checking fast)."""
    import trimesh
    mesh = trimesh.load(dae_path, force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
    hull = mesh.convex_hull
    ros_mesh = Mesh()
    for v in hull.vertices:
        pt = Point()
        pt.x, pt.y, pt.z = float(v[0]), float(v[1]), float(v[2])
        ros_mesh.vertices.append(pt)
    for f in hull.faces:
        tri = MeshTriangle()
        tri.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
        ros_mesh.triangles.append(tri)
    return ros_mesh


class TrolleyScenePublisher(Node):
    def __init__(self):
        super().__init__("trolley_scene_publisher")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, "/holoassist/scene/trolley", qos)
        self._apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._mesh: Mesh | None = None

        self._publish_marker()
        self._timer = self.create_timer(1.0, self._try_apply)

    def _publish_marker(self):
        stamp = self.get_clock().now().to_msg()

        del_m = Marker()
        del_m.header.frame_id = "world"
        del_m.header.stamp = stamp
        del_m.ns = "trolley"
        del_m.id = 999
        del_m.action = Marker.DELETEALL

        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = stamp
        m.ns = "trolley"
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = _MESH_URI
        m.mesh_use_embedded_materials = True
        m.pose = _yaw_pose(_X, _Y, _Z, _YAW_DEG)
        m.scale = Vector3(x=1.0, y=1.0, z=1.0)
        m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
        m.lifetime = Duration(sec=0, nanosec=0)

        msg = MarkerArray()
        msg.markers = [del_m, m]
        self._pub.publish(msg)
        self.get_logger().info("Published trolley marker on /holoassist/scene/trolley")

    def _try_apply(self):
        if not self._apply_client.service_is_ready():
            return

        self._timer.cancel()

        if self._mesh is None:
            try:
                import ament_index_python.packages
                pkg = ament_index_python.packages.get_package_share_directory("holoassist_perception")
                dae_path = os.path.join(pkg, "worlds", "UR3eTrolley_decimated.dae")
                self._mesh = _load_collision_mesh(dae_path)
                self.get_logger().info(
                    f"Loaded trolley convex hull ({len(self._mesh.vertices)} verts, "
                    f"{len(self._mesh.triangles)} tris)"
                )
            except Exception as e:
                self.get_logger().error(f"Mesh load failed: {e}")
                return

        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = "trolley"
        obj.operation = CollisionObject.ADD
        obj.meshes = [self._mesh]
        obj.mesh_poses = [_yaw_pose(_X, _Y, _Z, _YAW_DEG)]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._apply_client.call_async(req).add_done_callback(self._on_applied)

    def _on_applied(self, future):
        try:
            if future.result() and future.result().success:
                self.get_logger().info("Trolley added to MoveIt planning scene")
            else:
                self.get_logger().warn("apply_planning_scene failed — retrying in 5s")
                self.create_timer(5.0, self._try_apply)
        except Exception as e:
            self.get_logger().error(f"apply_planning_scene error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrolleyScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
