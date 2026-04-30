#!/usr/bin/env python3

import math
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class WebcamImagePublisherNode(Node):
    """Publish laptop webcam frames as sensor_msgs/Image for RGB fallback demos."""

    def __init__(self) -> None:
        super().__init__("holo_assist_webcam_image_publisher")

        self.declare_parameter("device_index", 0)
        self.declare_parameter("image_topic", "/holo_assist/webcam/image_raw")
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("use_sensor_data_qos", True)
        self.declare_parameter("qos_depth", 5)
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("camera_info_topic", "/holo_assist/webcam/camera_info")
        self.declare_parameter("camera_info_frame_id", "")
        self.declare_parameter("hfov_deg", 69.4)
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("distortion_model", "plumb_bob")
        self.declare_parameter(
            "distortion_coefficients", [0.0, 0.0, 0.0, 0.0, 0.0]
        )

        self.device_index = int(self.get_parameter("device_index").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.use_sensor_data_qos = bool(
            self.get_parameter("use_sensor_data_qos").value
        )
        self.qos_depth = int(self.get_parameter("qos_depth").value)
        self.publish_camera_info = bool(
            self.get_parameter("publish_camera_info").value
        )
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_info_frame_id = str(
            self.get_parameter("camera_info_frame_id").value
        )
        self.hfov_deg = float(self.get_parameter("hfov_deg").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.distortion_model = str(self.get_parameter("distortion_model").value)
        self.distortion_coefficients = [
            float(v) for v in self.get_parameter("distortion_coefficients").value
        ]

        if self.fps <= 0.0:
            self.get_logger().warn("fps must be > 0. Using 15.0.")
            self.fps = 15.0
        if self.width <= 0:
            self.width = 640
        if self.height <= 0:
            self.height = 480
        if self.qos_depth < 1:
            self.qos_depth = 1
        self.hfov_deg = max(1.0, min(179.0, self.hfov_deg))
        if not self.camera_info_frame_id:
            self.camera_info_frame_id = self.frame_id

        self.bridge = CvBridge()
        if self.use_sensor_data_qos:
            pub_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=self.qos_depth,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=qos_profile_sensor_data.durability,
            )
            qos_name = f"sensor_data(best_effort,depth={self.qos_depth})"
        else:
            pub_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=self.qos_depth,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            qos_name = f"default(reliable,depth={self.qos_depth})"
        self.publisher = self.create_publisher(Image, self.image_topic, pub_qos)
        self.camera_info_publisher = None
        if self.publish_camera_info:
            self.camera_info_publisher = self.create_publisher(
                CameraInfo, self.camera_info_topic, pub_qos
            )
        self._camera_info_template: Optional[CameraInfo] = None
        self.frame_count = 0
        self.capture: Optional[cv2.VideoCapture] = cv2.VideoCapture(self.device_index)

        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError(
                f"Failed to open webcam device_index={self.device_index}. "
                "Check camera permissions and device availability."
            )

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.capture.set(cv2.CAP_PROP_FPS, float(self.fps))

        period_s = 1.0 / self.fps
        self.timer = self.create_timer(period_s, self._publish_frame_cb)
        self.status_timer = self.create_timer(2.0, self._status_cb)

        self.get_logger().info(
            f"Webcam publisher started. topic={self.image_topic}, device_index={self.device_index}, "
            f"target={self.width}x{self.height}@{self.fps:.1f}Hz, "
            f"camera_info={self.publish_camera_info} ({self.camera_info_topic}), qos={qos_name}"
        )

    def _build_camera_info_template(self, width: int, height: int) -> CameraInfo:
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy

        if fx <= 0.0 or fy <= 0.0:
            hfov_rad = math.radians(self.hfov_deg)
            auto_fx = float(width) / (2.0 * math.tan(0.5 * hfov_rad))
            fx = auto_fx if fx <= 0.0 else fx
            fy = auto_fx if fy <= 0.0 else fy

        if cx <= 0.0:
            cx = 0.5 * (float(width) - 1.0)
        if cy <= 0.0:
            cy = 0.5 * (float(height) - 1.0)

        msg = CameraInfo()
        msg.header.frame_id = self.camera_info_frame_id
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = self.distortion_model
        msg.d = list(self.distortion_coefficients)
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish_frame_cb(self) -> None:
        if self.capture is None:
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warn("Failed to read webcam frame.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        if self.camera_info_publisher is not None:
            frame_h, frame_w = frame.shape[:2]
            if (
                self._camera_info_template is None
                or int(self._camera_info_template.width) != int(frame_w)
                or int(self._camera_info_template.height) != int(frame_h)
            ):
                self._camera_info_template = self._build_camera_info_template(
                    width=int(frame_w), height=int(frame_h)
                )

            cam_info = CameraInfo()
            cam_info.header.stamp = msg.header.stamp
            cam_info.header.frame_id = self._camera_info_template.header.frame_id
            cam_info.width = self._camera_info_template.width
            cam_info.height = self._camera_info_template.height
            cam_info.distortion_model = self._camera_info_template.distortion_model
            cam_info.d = list(self._camera_info_template.d)
            cam_info.k = list(self._camera_info_template.k)
            cam_info.r = list(self._camera_info_template.r)
            cam_info.p = list(self._camera_info_template.p)
            self.camera_info_publisher.publish(cam_info)
        self.frame_count += 1

    def _status_cb(self) -> None:
        self.get_logger().info(
            f"status: webcam_frames_published={self.frame_count}, topic={self.image_topic}"
        )

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamImagePublisherNode()
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
