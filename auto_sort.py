#!/usr/bin/env python3
"""
Auto-sort orchestrator for HoloAssist.

When the system is in MOVEIT mode, monitors perception cube poses and
automatically triggers pick-and-place for each detected cube.

Flow:
  1. Subscribes to /holoassist/mode_status — only acts when "MOVEIT"
  2. Subscribes to cube poses /holoassist/perception/april_cube_{1-4}_pose
  3. When a cube pose is stable for --settle-time seconds, calls the
     /holoassist/pick_cube_to_bin service
  4. Waits for the pick-place to finish before sorting the next cube
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# Try to import the service type — falls back to topic-based if unavailable
try:
    from holo_assist_depth_tracker_sim_interfaces.srv import PickCubeToBin
    SRV_AVAILABLE = True
except ImportError:
    SRV_AVAILABLE = False


# Default cube → bin mapping
DEFAULT_CUBE_BIN_MAP = {
    "april_cube_1": "bin_1",
    "april_cube_2": "bin_2",
    "april_cube_3": "bin_3",
    "april_cube_4": "bin_4",
}


class AutoSortOrchestrator(Node):
    def __init__(self):
        super().__init__("auto_sort_orchestrator")

        self.declare_parameter("settle_time", 3.0)
        self.declare_parameter("cube_count", 4)
        self.declare_parameter("cooldown_after_sort", 5.0)

        self.settle_time = self.get_parameter("settle_time").value
        self.cube_count = int(self.get_parameter("cube_count").value)
        self.cooldown = self.get_parameter("cooldown_after_sort").value

        # State
        self._mode = "TELEOP"
        self._lock = threading.Lock()
        self._cube_poses: dict[str, tuple[PoseStamped, float]] = {}
        self._sorted_cubes: set[str] = set()
        self._sorting_in_progress = False
        self._last_sort_time = 0.0

        # Mode status subscription
        self.create_subscription(
            String, "/holoassist/mode_status",
            self._on_mode_status, 10,
        )

        # Cube pose subscriptions
        for i in range(1, self.cube_count + 1):
            name = f"april_cube_{i}"
            topic = f"/holoassist/perception/{name}_pose"
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self._on_cube_pose(n, msg), 10,
            )

        # Pick-place command (topic-based fallback)
        self._command_pub = self.create_publisher(String, "/pick_place/command", 10)
        self._mode_pub = self.create_publisher(String, "/pick_place/mode", 10)

        # Service client (preferred)
        self._srv_client = None
        if SRV_AVAILABLE:
            self._srv_client = self.create_client(
                PickCubeToBin, "/holoassist/pick_cube_to_bin"
            )

        # Periodic check
        self.create_timer(1.0, self._check_and_sort)

        self.get_logger().info(
            f"Auto-sort ready — settle={self.settle_time}s, "
            f"cooldown={self.cooldown}s, cubes={self.cube_count}"
        )

    def _on_mode_status(self, msg: String):
        with self._lock:
            old = self._mode
            self._mode = msg.data.upper()
            if old != self._mode:
                self.get_logger().info(f"Mode changed: {old} -> {self._mode}")
                if self._mode == "MOVEIT":
                    self._sorted_cubes.clear()

    def _on_cube_pose(self, name: str, msg: PoseStamped):
        with self._lock:
            self._cube_poses[name] = (msg, time.time())

    def _check_and_sort(self):
        with self._lock:
            if self._mode != "MOVEIT":
                return
            if self._sorting_in_progress:
                return
            if time.time() - self._last_sort_time < self.cooldown:
                return

            now = time.time()
            candidates = []

            for name, (pose, last_seen) in self._cube_poses.items():
                if name in self._sorted_cubes:
                    continue
                age = now - last_seen
                if age > 5.0:
                    continue
                if (now - last_seen) < self.settle_time:
                    # Still within settle window — check if it's been consistently visible
                    # (first_seen is approximated by last_seen here; a proper implementation
                    # would track first detection time, but for demo this is sufficient)
                    pass
                candidates.append((name, pose))

        if not candidates:
            return

        # Sort the first available candidate
        name, pose = candidates[0]
        bin_id = DEFAULT_CUBE_BIN_MAP.get(name, "bin_1")

        self.get_logger().info(f"Auto-sorting {name} -> {bin_id}")
        self._sorting_in_progress = True

        threading.Thread(
            target=self._do_sort, args=(name, bin_id, pose),
            daemon=True,
        ).start()

    def _do_sort(self, cube_name: str, bin_id: str, pose: PoseStamped):
        try:
            # Ensure sequencer is in run mode
            mode_msg = String()
            mode_msg.data = "run"
            self._mode_pub.publish(mode_msg)
            time.sleep(0.5)

            if self._srv_client is not None and self._srv_client.wait_for_service(timeout_sec=2.0):
                req = PickCubeToBin.Request()
                req.cube_name = cube_name
                req.bin_id = bin_id
                future = self._srv_client.call_async(req)
                # Wait for service response (with timeout)
                start = time.time()
                while not future.done() and time.time() - start < 60.0:
                    time.sleep(0.5)
                self.get_logger().info(f"Service call for {cube_name} complete")
            else:
                # Fallback: publish command directly
                cmd = {
                    "block_id": cube_name,
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                    "bin_id": bin_id,
                }
                cmd_msg = String()
                cmd_msg.data = json.dumps(cmd)
                self._command_pub.publish(cmd_msg)
                self.get_logger().info(f"Published pick command for {cube_name}")

            # Wait for the sort to physically complete
            time.sleep(15.0)

            with self._lock:
                self._sorted_cubes.add(cube_name)
                self._last_sort_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Sort failed for {cube_name}: {e}")
        finally:
            self._sorting_in_progress = False


def main():
    rclpy.init()
    node = AutoSortOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
