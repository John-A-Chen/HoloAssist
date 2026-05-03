#!/usr/bin/env python3
import json
from typing import Optional, Sequence

import rclpy
import rclpy.duration
from geometry_msgs.msg import PoseStamped
from holo_assist_depth_tracker_sim_interfaces.srv import PickCubeToBin
from rclpy.node import Node
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
import tf2_ros

CUBE_NAMES = [f"april_cube_{i}" for i in range(1, 5)]
PLANNING_FRAME = "base_link"


def _normalize_cube(name: str) -> str:
    text = name.strip()
    if text in {"1", "2", "3", "4"}:
        return f"april_cube_{text}"
    return text


def _normalize_bin(bin_id: str) -> str:
    text = bin_id.strip()
    if text in {"1", "2", "3", "4"}:
        return f"bin_{text}"
    return text


class PickPlaceServiceNode(Node):
    """Bridges the PickCubeToBin service to the pick_place_sequencer command topic."""

    def __init__(self) -> None:
        super().__init__("pick_place_service_node")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cube_poses: dict[str, Optional[PoseStamped]] = {n: None for n in CUBE_NAMES}
        for name in CUBE_NAMES:
            self.create_subscription(
                PoseStamped,
                f"/holoassist/sim/truth/{name}_pose",
                lambda msg, n=name: self._on_cube_pose(n, msg),
                10,
            )

        self._command_pub = self.create_publisher(String, "/pick_place/command", 10)
        self._mode_pub = self.create_publisher(String, "/pick_place/mode", 10)

        self.create_service(
            PickCubeToBin,
            "/holoassist/pick_cube_to_bin",
            self._handle_pick_cube_to_bin,
        )

        self.get_logger().info("PickPlaceServiceNode ready on /holoassist/pick_cube_to_bin")

    def _on_cube_pose(self, name: str, msg: PoseStamped) -> None:
        self._cube_poses[name] = msg

    def _handle_pick_cube_to_bin(
        self,
        request: PickCubeToBin.Request,
        response: PickCubeToBin.Response,
    ) -> PickCubeToBin.Response:
        cube_name = _normalize_cube(request.cube_name)
        bin_id = _normalize_bin(request.bin_id)

        if cube_name not in CUBE_NAMES:
            response.success = False
            response.message = (
                f"Unknown cube {request.cube_name!r}. "
                f"Valid: {', '.join(CUBE_NAMES)} or 1–4."
            )
            return response

        valid_bins = {f"bin_{i}" for i in range(1, 5)}
        if bin_id not in valid_bins:
            response.success = False
            response.message = (
                f"Unknown bin {request.bin_id!r}. Valid: bin_1–bin_4 or 1–4."
            )
            return response

        pose_ws = self._cube_poses.get(cube_name)
        if pose_ws is None:
            response.success = False
            response.message = (
                f"No pose received yet for {cube_name}. Is sim_cube_truth_node running?"
            )
            return response

        try:
            pose_bl = self._tf_buffer.transform(
                pose_ws,
                PLANNING_FRAME,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:
            response.success = False
            response.message = (
                f"TF {pose_ws.header.frame_id} → {PLANNING_FRAME} failed: {exc}"
            )
            return response

        x = pose_bl.pose.position.x
        y = pose_bl.pose.position.y
        z = pose_bl.pose.position.z

        mode_msg = String()
        mode_msg.data = "run"
        self._mode_pub.publish(mode_msg)

        cmd_msg = String()
        cmd_msg.data = json.dumps(
            {"block_id": cube_name, "x": x, "y": y, "z": z, "bin_id": bin_id}
        )
        self._command_pub.publish(cmd_msg)

        self.get_logger().info(
            f"Queued pick {cube_name} → {bin_id} "
            f"(base_link x={x:.3f} y={y:.3f} z={z:.3f})"
        )
        response.success = True
        response.message = (
            f"Pick {cube_name} → {bin_id} queued "
            f"(x={x:.3f} y={y:.3f} z={z:.3f} in {PLANNING_FRAME})"
        )
        return response


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = PickPlaceServiceNode()
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
