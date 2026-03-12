#!/usr/bin/env python3

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray


class DepthTrackerNode(Node):
    """Depth-only blob tracker using thresholding + morphology + connected components."""

    def __init__(self) -> None:
        super().__init__("depth_tracker")

        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("min_depth_m", 2.0)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("min_area_px", 400)
        self.declare_parameter("morph_kernel", 5)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.morph_kernel = int(self.get_parameter("morph_kernel").value)

        if self.max_depth_m <= self.min_depth_m:
            self.get_logger().warn(
                "max_depth_m must be greater than min_depth_m. Using defaults 2.0 to 5.0 m."
            )
            self.min_depth_m = 2.0
            self.max_depth_m = 5.0

        if self.morph_kernel < 1:
            self.get_logger().warn("morph_kernel must be >= 1. Using morph_kernel=1.")
            self.morph_kernel = 1
        if self.morph_kernel % 2 == 0:
            self.get_logger().warn(
                f"morph_kernel should be odd for centered morphology. Using {self.morph_kernel + 1}."
            )
            self.morph_kernel += 1

        self.bridge = CvBridge()
        self.latest_camera_info: Optional[CameraInfo] = None
        self.frame_count = 0

        # Match RealSense depth publisher QoS (RELIABLE) for robust delivery.
        image_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            image_sub_qos,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            image_sub_qos,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/depth_tracker/debug_image",
            qos_profile_sensor_data,
        )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray,
            "/depth_tracker/bbox",
            10,
        )

        self.get_logger().info(
            "Depth tracker started. bbox payload: [x_min, y_min, x_max, y_max, cx, cy, median_depth_m, area_px]"
        )
        self.get_logger().info(
            f"Subscribing depth={self.depth_topic}, camera_info={self.camera_info_topic}, "
            f"band=[{self.min_depth_m:.2f}, {self.max_depth_m:.2f}] m, "
            f"min_area_px={self.min_area_px}, morph_kernel={self.morph_kernel}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def depth_callback(self, msg: Image) -> None:
        self.frame_count += 1
        depth_m = self._depth_to_meters(msg)
        if depth_m is None:
            return

        valid_mask = np.isfinite(depth_m) & (depth_m > 0.0)
        band_mask = (
            valid_mask
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        )

        binary_mask = (band_mask.astype(np.uint8) * 255)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morph_kernel, self.morph_kernel),
        )
        cleaned = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            (cleaned > 0).astype(np.uint8),
            connectivity=8,
        )

        largest_label = -1
        largest_area = 0
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area >= self.min_area_px and area > largest_area:
                largest_area = area
                largest_label = label

        debug_bgr = self._create_debug_image(depth_m)

        bbox_msg = Float32MultiArray()
        bbox_msg.data = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.0]

        if largest_label != -1:
            x = int(stats[largest_label, cv2.CC_STAT_LEFT])
            y = int(stats[largest_label, cv2.CC_STAT_TOP])
            w = int(stats[largest_label, cv2.CC_STAT_WIDTH])
            h = int(stats[largest_label, cv2.CC_STAT_HEIGHT])
            area_px = int(stats[largest_label, cv2.CC_STAT_AREA])
            cx, cy = centroids[largest_label]

            blob_mask = labels == largest_label
            blob_depths = depth_m[blob_mask]
            blob_depths = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0.0)]
            if blob_depths.size == 0:
                median_depth_m = -1.0
            else:
                median_depth_m = float(np.median(blob_depths))

            cv2.rectangle(debug_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug_bgr, (int(cx), int(cy)), 4, (0, 255, 255), -1)

            label = f"z={median_depth_m:.2f}m area={area_px}"
            cv2.putText(
                debug_bgr,
                label,
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            bbox_msg.data = [
                float(x),
                float(y),
                float(x + w),
                float(y + h),
                float(cx),
                float(cy),
                float(median_depth_m),
                float(area_px),
            ]
        else:
            cv2.putText(
                debug_bgr,
                "No blob in depth band",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.latest_camera_info is None:
            cv2.putText(
                debug_bgr,
                "camera_info: waiting...",
                (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        debug_msg.header = msg.header

        self.debug_image_pub.publish(debug_msg)
        self.bbox_pub.publish(bbox_msg)

    def _depth_to_meters(self, msg: Image) -> Optional[np.ndarray]:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge conversion failed: {exc}")
            return None

        if msg.encoding == "16UC1":
            return depth.astype(np.float32) * 0.001

        if msg.encoding == "32FC1":
            return depth.astype(np.float32)

        self.get_logger().warn(
            f"Unsupported depth encoding '{msg.encoding}'. Expected 16UC1 or 32FC1."
        )
        return None

    def _create_debug_image(self, depth_m: np.ndarray) -> np.ndarray:
        depth_range = max(1e-3, self.max_depth_m - self.min_depth_m)
        clipped = np.clip(depth_m, self.min_depth_m, self.max_depth_m)

        norm = ((clipped - self.min_depth_m) / depth_range * 255.0).astype(np.uint8)
        invalid = ~np.isfinite(depth_m) | (depth_m <= 0.0)
        norm[invalid] = 0

        return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


def main(args=None):
    rclpy.init(args=args)
    node = DepthTrackerNode()
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
