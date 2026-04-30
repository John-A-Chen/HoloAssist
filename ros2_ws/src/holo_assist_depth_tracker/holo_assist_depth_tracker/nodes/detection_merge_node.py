#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class CachedDetection:
    stamp_ns: int
    frame_id: str
    detection: AprilTagDetection


def merge_detections_by_id(
    cache: Dict[int, CachedDetection],
    detections: list[AprilTagDetection],
    stamp_ns: int,
    frame_id: str,
) -> None:
    for det in detections:
        det_id = int(det.id)
        cached = cache.get(det_id)
        if cached is None or stamp_ns >= cached.stamp_ns:
            cache[det_id] = CachedDetection(
                stamp_ns=stamp_ns,
                frame_id=frame_id,
                detection=det,
            )


def prune_stale_detections(
    cache: Dict[int, CachedDetection],
    now_ns: int,
    stale_timeout_s: float,
) -> None:
    max_age_ns = int(stale_timeout_s * 1e9)
    stale_ids = [
        det_id
        for det_id, cached in cache.items()
        if (now_ns - cached.stamp_ns) > max_age_ns
    ]
    for det_id in stale_ids:
        cache.pop(det_id, None)


class DetectionMergeNode(Node):
    """Merge board/cube AprilTag streams into a deduplicated unified stream."""

    def __init__(self) -> None:
        super().__init__("holoassist_detection_merge")

        self.declare_parameter("board_topic", "/detections_board")
        self.declare_parameter("cubes_topic", "/detections_cubes")
        self.declare_parameter("output_topic", "/detections_all")
        self.declare_parameter("stale_timeout_s", 1.0)

        self.board_topic = str(self.get_parameter("board_topic").value)
        self.cubes_topic = str(self.get_parameter("cubes_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.stale_timeout_s = max(0.05, float(self.get_parameter("stale_timeout_s").value))

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._detections_by_id: Dict[int, CachedDetection] = {}

        self._board_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.board_topic,
            self._on_board,
            reliable_qos,
        )
        self._cubes_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.cubes_topic,
            self._on_cubes,
            reliable_qos,
        )
        self._pub = self.create_publisher(AprilTagDetectionArray, self.output_topic, reliable_qos)

        self.get_logger().info(
            "detection merge started board=%s cubes=%s out=%s stale_timeout=%.2fs"
            % (self.board_topic, self.cubes_topic, self.output_topic, self.stale_timeout_s)
        )

    def _on_board(self, msg: AprilTagDetectionArray) -> None:
        self._merge_message(msg)

    def _on_cubes(self, msg: AprilTagDetectionArray) -> None:
        self._merge_message(msg)

    def _merge_message(self, msg: AprilTagDetectionArray) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        stamp_ns = int(stamp.nanoseconds)
        if stamp_ns == 0:
            stamp_ns = int(self.get_clock().now().nanoseconds)

        frame_id = str(msg.header.frame_id)
        merge_detections_by_id(
            cache=self._detections_by_id,
            detections=list(msg.detections),
            stamp_ns=stamp_ns,
            frame_id=frame_id,
        )

        self._prune_stale()
        self._publish_union()

    def _prune_stale(self) -> None:
        prune_stale_detections(
            cache=self._detections_by_id,
            now_ns=int(self.get_clock().now().nanoseconds),
            stale_timeout_s=self.stale_timeout_s,
        )

    def _publish_union(self) -> None:
        msg = AprilTagDetectionArray()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        if not self._detections_by_id:
            msg.header.frame_id = ""
            self._pub.publish(msg)
            return

        latest = max(self._detections_by_id.values(), key=lambda item: item.stamp_ns)
        msg.header.frame_id = latest.frame_id

        for det_id in sorted(self._detections_by_id.keys()):
            msg.detections.append(self._detections_by_id[det_id].detection)

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
