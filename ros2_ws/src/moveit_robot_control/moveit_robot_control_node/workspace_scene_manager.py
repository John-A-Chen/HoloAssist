#!/usr/bin/env python3
import json
import math
import zlib
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


DEFAULT_FRAME_ID = "base_link"
DEFAULT_MARKER_TOPIC = "/workspace_scene/markers"
DEFAULT_COMMAND_TOPIC = "/workspace_scene/command"
DEFAULT_BLOCK_POSE_TOPIC = "/workspace_scene/spawn_block_pose"
DEFAULT_STATUS_TOPIC = "/workspace_scene/status"
DEFAULT_APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
DEFAULT_TABLE_MESH_RESOURCE = (
    "package://moveit_robot_control/meshes/UR3eTrolley_decimated.dae"
)
TABLE_MARKER_NS = "workspace_table"
BLOCK_MARKER_NS = "workspace_blocks"


@dataclass
class BlockSpec:
    object_id: str
    frame_id: str
    pose: Pose
    size: list[float]
    color: list[float]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def pose_from_xyz_rpy(xyz: Sequence[float], rpy_deg: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation = quaternion_from_rpy(
        math.radians(float(rpy_deg[0])),
        math.radians(float(rpy_deg[1])),
        math.radians(float(rpy_deg[2])),
    )
    return pose


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def finite_values(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class WorkspaceSceneManager(Node):
    """Publishes visible RViz markers and MoveIt collision objects for the cell."""

    def __init__(self) -> None:
        super().__init__("workspace_scene_manager")

        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("marker_topic", DEFAULT_MARKER_TOPIC)
        self.declare_parameter("command_topic", DEFAULT_COMMAND_TOPIC)
        self.declare_parameter("block_pose_topic", DEFAULT_BLOCK_POSE_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter(
            "apply_planning_scene_service", DEFAULT_APPLY_PLANNING_SCENE_SERVICE
        )
        self.declare_parameter("apply_timeout_sec", 5.0)
        self.declare_parameter("marker_publish_period_sec", 1.0)

        self.declare_parameter("publish_table_mesh", True)
        self.declare_parameter("table_mesh_resource", DEFAULT_TABLE_MESH_RESOURCE)
        self.declare_parameter("table_mesh_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("table_mesh_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("table_mesh_scale", [1.0, 1.0, 1.0])

        self.declare_parameter("apply_table_collision", False)
        self.declare_parameter("table_collision_id", "workspace_table")
        self.declare_parameter("table_collision_xyz", [0.0, 0.0, 1.04])
        self.declare_parameter("table_collision_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("table_collision_size", [0.80, 0.70, 0.04])

        self.declare_parameter("default_block_size", [0.05, 0.05, 0.05])
        self.declare_parameter("default_block_color", [0.1, 0.45, 0.95, 1.0])

        self.frame_id = str(self.get_parameter("frame_id").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        block_pose_topic = str(self.get_parameter("block_pose_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        apply_service = str(self.get_parameter("apply_planning_scene_service").value)
        self.apply_timeout_sec = float(self.get_parameter("apply_timeout_sec").value)
        marker_period = float(
            self.get_parameter("marker_publish_period_sec").value
        )

        self.publish_table_mesh = bool(self.get_parameter("publish_table_mesh").value)
        self.table_mesh_resource = str(
            self.get_parameter("table_mesh_resource").value
        )
        self.table_mesh_pose = pose_from_xyz_rpy(
            self.get_vector_parameter("table_mesh_xyz", 3),
            self.get_vector_parameter("table_mesh_rpy_deg", 3),
        )
        self.table_mesh_scale = self.get_vector_parameter("table_mesh_scale", 3)

        self.apply_table_collision = bool(
            self.get_parameter("apply_table_collision").value
        )
        self.table_collision_id = str(
            self.get_parameter("table_collision_id").value
        )
        self.table_collision_pose = pose_from_xyz_rpy(
            self.get_vector_parameter("table_collision_xyz", 3),
            self.get_vector_parameter("table_collision_rpy_deg", 3),
        )
        self.table_collision_size = self.get_vector_parameter(
            "table_collision_size", 3
        )

        self.default_block_size = self.get_vector_parameter("default_block_size", 3)
        self.default_block_color = self.get_vector_parameter(
            "default_block_color", 4
        )

        self.blocks: Dict[str, BlockSpec] = {}
        self.pose_spawn_count = 0

        transient_qos = QoSProfile(depth=10)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, transient_qos)
        self.status_pub = self.create_publisher(String, status_topic, transient_qos)
        self.create_subscription(String, command_topic, self.command_cb, 10)
        self.create_subscription(PoseStamped, block_pose_topic, self.block_pose_cb, 10)
        self.apply_client = self.create_client(ApplyPlanningScene, apply_service)

        self.create_timer(max(0.2, marker_period), self.publish_markers)
        self.startup_timer = self.create_timer(0.5, self.startup_cb)
        self.startup_done = False

        self.publish_status(
            "Workspace scene manager ready",
            marker_topic=marker_topic,
            command_topic=command_topic,
            block_pose_topic=block_pose_topic,
            frame_id=self.frame_id,
        )

    def get_vector_parameter(self, name: str, length: int) -> list[float]:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            text = value.strip()
            if text.startswith("["):
                parsed = json.loads(text)
            else:
                parsed = [part for part in text.replace(",", " ").split() if part]
        else:
            parsed = value

        try:
            vector = [float(item) for item in parsed]
        except TypeError as exc:
            raise ValueError(f"Parameter {name} must be a vector") from exc

        if len(vector) != length:
            raise ValueError(f"Parameter {name} must contain {length} values")
        if not finite_values(vector):
            raise ValueError(f"Parameter {name} contains a non-finite value")
        return vector

    def publish_status(self, message: str, **fields: Any) -> None:
        payload = {"message": message, **fields}
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def startup_cb(self) -> None:
        if self.startup_done:
            return
        self.startup_done = True
        self.startup_timer.cancel()

        self.publish_markers()
        if self.apply_table_collision:
            collision_object = self.make_box_collision_object(
                self.table_collision_id,
                self.frame_id,
                self.table_collision_pose,
                self.table_collision_size,
                CollisionObject.ADD,
            )
            self.apply_collision_objects([collision_object], "table collision object")
        else:
            self.get_logger().info(
                "Table mesh is visual only; set apply_table_collision:=true after "
                "tuning table_collision_xyz/size if you want MoveIt to avoid it"
            )

    def command_cb(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError("command JSON must be an object")

            action = str(command.get("action", "add_block")).lower()
            if action in {"add", "add_block", "spawn", "spawn_block"}:
                self.add_block_from_command(command)
            elif action in {"remove", "remove_block", "delete"}:
                object_id = str(command.get("id", "")).strip()
                if not object_id:
                    raise ValueError("remove command needs an id")
                self.remove_block(object_id)
            elif action in {"clear", "clear_blocks", "remove_all"}:
                self.clear_blocks()
            elif action in {"add_table_collision", "apply_table_collision"}:
                collision_object = self.make_box_collision_object(
                    self.table_collision_id,
                    self.frame_id,
                    self.table_collision_pose,
                    self.table_collision_size,
                    CollisionObject.ADD,
                )
                self.apply_collision_objects(
                    [collision_object], "table collision object"
                )
            else:
                raise ValueError(f"unknown action {action!r}")
        except Exception as exc:
            self.publish_status(f"Ignoring workspace scene command: {exc}")

    def block_pose_cb(self, msg: PoseStamped) -> None:
        self.pose_spawn_count += 1
        object_id = f"block_{self.pose_spawn_count:03d}"
        frame_id = msg.header.frame_id or self.frame_id
        self.add_block(
            object_id,
            frame_id,
            copy_pose(msg.pose),
            list(self.default_block_size),
            list(self.default_block_color),
        )

    def add_block_from_command(self, command: dict[str, Any]) -> None:
        object_id = str(command.get("id", "")).strip()
        if not object_id:
            self.pose_spawn_count += 1
            object_id = f"block_{self.pose_spawn_count:03d}"

        frame_id = str(command.get("frame_id", self.frame_id))
        size = self.parse_command_vector(
            command.get("size", command.get("dimensions", self.default_block_size)),
            3,
            "size",
        )
        color = self.parse_command_vector(
            command.get("color", self.default_block_color),
            4,
            "color",
        )
        pose = self.parse_command_pose(command)

        z_mode = str(command.get("z_mode", "center")).lower()
        if bool(command.get("z_is_bottom", False)) or z_mode in {"bottom", "surface"}:
            pose.position.z += size[2] / 2.0

        self.add_block(object_id, frame_id, pose, size, color)

    def parse_command_pose(self, command: dict[str, Any]) -> Pose:
        pose = Pose()

        if "position" in command:
            position = command["position"]
            if isinstance(position, dict):
                xyz = [position["x"], position["y"], position["z"]]
            else:
                xyz = self.parse_command_vector(position, 3, "position")
        else:
            xyz = [command["x"], command["y"], command["z"]]

        if not finite_values(xyz):
            raise ValueError("position contains a non-finite value")
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])

        if "orientation" in command:
            orientation = command["orientation"]
            pose.orientation.x = float(orientation.get("x", 0.0))
            pose.orientation.y = float(orientation.get("y", 0.0))
            pose.orientation.z = float(orientation.get("z", 0.0))
            pose.orientation.w = float(orientation.get("w", 1.0))
        elif "rpy_deg" in command:
            rpy_deg = self.parse_command_vector(command["rpy_deg"], 3, "rpy_deg")
            pose.orientation = pose_from_xyz_rpy([0.0, 0.0, 0.0], rpy_deg).orientation
        else:
            pose.orientation.w = 1.0

        orientation_values = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not finite_values(orientation_values):
            raise ValueError("orientation contains a non-finite value")
        return pose

    def parse_command_vector(
        self, value: Any, length: int, field_name: str
    ) -> list[float]:
        if isinstance(value, str):
            text = value.strip()
            if text.startswith("["):
                parsed = json.loads(text)
            else:
                parsed = [part for part in text.replace(",", " ").split() if part]
        else:
            parsed = value

        vector = [float(item) for item in parsed]
        if len(vector) != length:
            raise ValueError(f"{field_name} must contain {length} values")
        if not finite_values(vector):
            raise ValueError(f"{field_name} contains a non-finite value")
        return vector

    def add_block(
        self,
        object_id: str,
        frame_id: str,
        pose: Pose,
        size: list[float],
        color: list[float],
    ) -> None:
        if any(dimension <= 0.0 for dimension in size):
            raise ValueError("block dimensions must be greater than zero")

        block = BlockSpec(object_id, frame_id, pose, size, color)
        self.blocks[object_id] = block

        collision_object = self.make_box_collision_object(
            object_id,
            frame_id,
            pose,
            size,
            CollisionObject.ADD,
        )
        self.apply_collision_objects([collision_object], f"block {object_id}")
        self.publish_markers()
        self.publish_status(
            f"Added block {object_id}",
            id=object_id,
            frame_id=frame_id,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            size=size,
        )

    def remove_block(self, object_id: str) -> None:
        self.blocks.pop(object_id, None)
        collision_object = CollisionObject()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = self.frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        self.apply_collision_objects([collision_object], f"remove block {object_id}")
        self.publish_markers([self.make_delete_marker(object_id)])
        self.publish_status(f"Removed block {object_id}", id=object_id)

    def clear_blocks(self) -> None:
        if not self.blocks:
            self.publish_status("No workspace blocks to clear")
            return

        delete_markers = [
            self.make_delete_marker(object_id) for object_id in self.blocks.keys()
        ]
        collision_objects = []
        for object_id in self.blocks.keys():
            collision_object = CollisionObject()
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.header.frame_id = self.frame_id
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            collision_objects.append(collision_object)

        count = len(self.blocks)
        self.blocks.clear()
        self.apply_collision_objects(collision_objects, "clear workspace blocks")
        self.publish_markers(delete_markers)
        self.publish_status(f"Cleared {count} workspace block(s)")

    def make_box_collision_object(
        self,
        object_id: str,
        frame_id: str,
        pose: Pose,
        size: Sequence[float],
        operation: int,
    ) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(copy_pose(pose))
        collision_object.operation = operation
        return collision_object

    def apply_collision_objects(
        self, collision_objects: Sequence[CollisionObject], description: str
    ) -> bool:
        if not collision_objects:
            return True

        if not self.apply_client.service_is_ready():
            if not self.apply_client.wait_for_service(
                timeout_sec=self.apply_timeout_sec
            ):
                self.publish_status(
                    "MoveIt apply planning scene service is not available",
                    description=description,
                )
                return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_client.call_async(request)
        future.add_done_callback(
            lambda done_future: self.apply_result_cb(done_future, description)
        )
        return True

    def apply_result_cb(self, future: Any, description: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.publish_status(
                f"MoveIt rejected scene update for {description}: {exc}"
            )
            return

        if response is None or not response.success:
            self.publish_status(f"MoveIt did not apply scene update for {description}")
            return

        self.publish_status(f"MoveIt scene updated for {description}")

    def publish_markers(
        self, extra_markers: Optional[Sequence[Marker]] = None
    ) -> None:
        marker_array = MarkerArray()
        if self.publish_table_mesh:
            marker_array.markers.append(self.make_table_marker())
        for block in self.blocks.values():
            marker_array.markers.append(self.make_block_marker(block))
        if extra_markers:
            marker_array.markers.extend(extra_markers)
        self.marker_pub.publish(marker_array)

    def make_table_marker(self) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = TABLE_MARKER_NS
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose = copy_pose(self.table_mesh_pose)
        marker.scale.x = self.table_mesh_scale[0]
        marker.scale.y = self.table_mesh_scale[1]
        marker.scale.z = self.table_mesh_scale[2]
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.mesh_resource = self.table_mesh_resource
        marker.mesh_use_embedded_materials = True
        return marker

    def make_block_marker(self, block: BlockSpec) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = block.frame_id
        marker.ns = BLOCK_MARKER_NS
        marker.id = self.marker_id_for_object(block.object_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = copy_pose(block.pose)
        marker.scale.x = block.size[0]
        marker.scale.y = block.size[1]
        marker.scale.z = block.size[2]
        marker.color.r = block.color[0]
        marker.color.g = block.color[1]
        marker.color.b = block.color[2]
        marker.color.a = block.color[3]
        return marker

    def make_delete_marker(self, object_id: str) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = BLOCK_MARKER_NS
        marker.id = self.marker_id_for_object(object_id)
        marker.action = Marker.DELETE
        return marker

    def marker_id_for_object(self, object_id: str) -> int:
        return zlib.crc32(object_id.encode("utf-8")) & 0x7FFFFFFF


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = WorkspaceSceneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
