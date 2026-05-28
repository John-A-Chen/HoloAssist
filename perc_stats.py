#!/usr/bin/env python3
"""
perc_stats.py — Live perception stack latency and rate monitor.

Maps directly to the subsystem 1 evaluation criteria (P/C/D/HD).

Usage:
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  python3 perc_stats.py
"""

import threading
import time
from collections import deque
from typing import Optional

import rclpy
import rclpy.duration
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    HAVE_APRILTAG = True
except ImportError:
    HAVE_APRILTAG = False

CUBE_GROUPS = {
    1: list(range(10, 16)),
    2: list(range(16, 22)),
    3: list(range(22, 28)),
    4: list(range(28, 34)),
}
CALIB_TAG_ID = 1


# ── helpers ───────────────────────────────────────────────────────────────────

class TopicMonitor:
    """Rolling-window Hz + header-stamp latency tracker."""

    def __init__(self, maxlen: int = 30) -> None:
        self._times: deque = deque(maxlen=maxlen)
        self._last_latency_ms: Optional[float] = None
        self._lock = threading.Lock()

    def record(self, stamp_ns: Optional[int] = None, now_ns: Optional[int] = None) -> None:
        t = time.monotonic_ns()
        with self._lock:
            self._times.append(t)
            if stamp_ns is not None and now_ns is not None and stamp_ns > 0:
                self._last_latency_ms = (now_ns - stamp_ns) / 1e6

    @property
    def hz(self) -> float:
        with self._lock:
            if len(self._times) < 2:
                return 0.0
            span = (self._times[-1] - self._times[0]) / 1e9
            return (len(self._times) - 1) / span if span > 0 else 0.0

    @property
    def age_s(self) -> float:
        with self._lock:
            if not self._times:
                return float("inf")
            return (time.monotonic_ns() - self._times[-1]) / 1e9

    @property
    def latency_ms(self) -> Optional[float]:
        with self._lock:
            return self._last_latency_ms


# ── node ──────────────────────────────────────────────────────────────────────

class PercStatsNode(Node):
    def __init__(self) -> None:
        super().__init__("perc_stats")

        self.monitors = {
            "color":       TopicMonitor(),
            "depth":       TopicMonitor(),
            "debug_image": TopicMonitor(),
            "detections":  TopicMonitor(),
            "cube_1":      TopicMonitor(),
            "cube_2":      TopicMonitor(),
            "cube_3":      TopicMonitor(),
            "cube_4":      TopicMonitor(),
        }
        self._visible_ids: set = set()
        self._cube_poses: dict = {}
        self._cube_status: dict = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera topics (P grade)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
            lambda m: self._on_img(m, "color"), 5)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
            lambda m: self._on_img(m, "depth"), 5)
        self.create_subscription(Image, "/holo_assist_depth_tracker/debug_image",
            lambda m: self._on_img(m, "debug_image"), 5)

        # Detections (C/D grade)
        if HAVE_APRILTAG:
            self.create_subscription(AprilTagDetectionArray, "/detections_all",
                self._on_detections, 10)
        else:
            self.get_logger().warn("apriltag_msgs not found — detection stats unavailable")

        # Cube poses (D/HD grade)
        for n in range(1, 5):
            self.create_subscription(PoseStamped, f"/holoassist/perception/april_cube_{n}_pose",
                lambda m, n=n: self._on_cube_pose(m, n), 10)
            self.create_subscription(String, f"/holoassist/perception/april_cube_{n}_status",
                lambda m, n=n: self._on_cube_status(m, n), 10)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_img(self, msg: Image, key: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.monitors[key].record(stamp_ns, now_ns)

    def _on_detections(self, msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.monitors["detections"].record(stamp_ns, now_ns)
        self._visible_ids = {int(d.id) for d in msg.detections}

    def _on_cube_pose(self, msg: PoseStamped, cube_n: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.monitors[f"cube_{cube_n}"].record(stamp_ns, now_ns)
        p = msg.pose.position
        self._cube_poses[cube_n] = (p.x, p.y, p.z)

    def _on_cube_status(self, msg: String, cube_n: int) -> None:
        self._cube_status[cube_n] = msg.data

    # ── queries ───────────────────────────────────────────────────────────────

    def _tf_available(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            return True
        except TransformException:
            return False

    def _pointcloud_live(self) -> bool:
        try:
            return len(self.get_publishers_info_by_topic(
                "/camera/camera/depth/color/points")) > 0
        except Exception:
            return False

    # ── display ───────────────────────────────────────────────────────────────

    def print_status(self) -> None:
        G  = "\033[92m"   # green
        Y  = "\033[93m"   # yellow
        R  = "\033[91m"   # red
        DIM = "\033[90m"
        B  = "\033[1m"
        RS = "\033[0m"
        OK = G + "✓" + RS
        WN = Y + "~" + RS
        NO = R + "✗" + RS

        def hz_badge(key: str, expected: float = 15.0) -> str:
            m = self.monitors[key]
            if m.age_s > 3.0:
                return f"{NO} {DIM}no data{RS}"
            hz = m.hz
            icon = OK if hz >= expected * 0.75 else WN
            return f"{icon} {hz:.1f} Hz  (age {m.age_s:.2f}s)"

        def lat_badge(key: str) -> str:
            lat = self.monitors[key].latency_ms
            if lat is None or self.monitors[key].age_s > 3.0:
                return f"{DIM}--{RS}"
            c = G if lat < 80 else Y if lat < 150 else R
            return f"{c}{lat:.0f} ms{RS}"

        tf_ok = self._tf_available()
        pc_ok = self._pointcloud_live()
        calib_vis = CALIB_TAG_ID in self._visible_ids

        rows = []
        a = rows.append

        a(f"{B}{'═'*56}{RS}")
        a(f"{B}  HoloAssist Perception — Live Stats{RS}")
        a(f"{'═'*56}")
        a("")

        # P grade
        a(f"{B}P  Camera Publishing{RS}")
        a(f"  color  /camera/.../color/image_raw   {hz_badge('color')}")
        a(f"  depth  /camera/.../depth/image_rect  {hz_badge('depth')}")
        if pc_ok:
            a(f"  pointcloud                          {OK} publishers active")
        else:
            a(f"  pointcloud                          {WN} not publishing")
            a(f"    {DIM}→ launch camera_only.launch.py pointcloud:=true{RS}")
        a(f"  debug_image (tracker overlay)       {hz_badge('debug_image')}")
        a("")

        # C/D grade detection
        a(f"{B}C/D  AprilTag Detection{RS}")
        a(f"  /detections_all                     {hz_badge('detections')}")
        a(f"  detection latency (stamp→recv)      {lat_badge('detections')}")
        vis_str = str(sorted(self._visible_ids)) if self._visible_ids else f"{DIM}(none){RS}"
        a(f"  visible tag IDs                     {vis_str}")
        c_icon = OK if calib_vis else DIM + "--" + RS
        a(f"  calib tag id={CALIB_TAG_ID}                       {c_icon}")
        a("")

        # D grade poses
        a(f"{B}D    Cube Poses  (frame: workspace_frame){RS}")
        active_cubes = 0
        for n in range(1, 5):
            m = self.monitors[f"cube_{n}"]
            ids = CUBE_GROUPS[n]
            seen = sorted(t for t in ids if t in self._visible_ids)
            if m.age_s < 2.0:
                active_cubes += 1
                pos = self._cube_poses.get(n, (0.0, 0.0, 0.0))
                pos_s = f"({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
                a(f"  cube_{n}  ids {ids[0]}-{ids[-1]}   {OK} {m.hz:.0f}Hz  lat={lat_badge(f'cube_{n}')}  {pos_s}")
            else:
                tag_info = f"tags_in_frame={seen}" if seen else "not visible"
                a(f"  cube_{n}  ids {ids[0]}-{ids[-1]}   {NO} stale  {DIM}{tag_info}{RS}")

        a("")
        a(f"{B}D    Calibration TF{RS}")
        if tf_ok:
            a(f"  base_link → camera_color_optical   {OK} available")
        else:
            a(f"  base_link → camera_color_optical   {NO} not found")
            a(f"    {DIM}→ ros2 launch easy_handeye2 publish.launch.py name:=holoassist_calibration{RS}")
        a("")

        # HD grade / latency summary
        a(f"{B}HD   Tracking Latency Summary{RS}")
        det_lat = self.monitors["detections"].latency_ms
        a(f"  image → /detections_all             {lat_badge('detections')}")

        cube_lats = [
            self.monitors[f"cube_{n}"].latency_ms
            for n in range(1, 5)
            if self.monitors[f"cube_{n}"].latency_ms is not None
            and self.monitors[f"cube_{n}"].age_s < 2.0
        ]
        if cube_lats:
            avg = sum(cube_lats) / len(cube_lats)
            c = G if avg < 80 else Y if avg < 150 else R
            a(f"  image → cube pose (avg {active_cubes} cubes)      {c}{avg:.0f} ms{RS}")
        else:
            a(f"  image → cube pose                   {DIM}-- (no cubes visible){RS}")

        a(f"  HD tracking: {active_cubes}/4 cubes live,  identity persists via tag ID group")
        a("")
        a(f"{'═'*56}")
        a(f"  {DIM}Ctrl+C to exit   —   refreshes every 2s{RS}")
        a("")

        print("\033[2J\033[H", end="")
        print("\n".join(rows))


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = PercStatsNode()

    def _print_loop() -> None:
        while rclpy.ok():
            node.print_status()
            time.sleep(2.0)

    t = threading.Thread(target=_print_loop, daemon=True)
    t.start()

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
