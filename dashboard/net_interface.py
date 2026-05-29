"""
Network-backed dashboard interface — connects to bridge_server.py via WebSocket.
Drop-in replacement for RosInterface when running on the Steam Deck.

No ROS 2 dependencies. Only needs: websockets, plus the dataclass/enum
definitions from ros_interface.py (which import fine without rclpy).
"""

import asyncio
import json
import threading
import time

import websockets

from ros_interface import DashboardStatus, RobotState, TopicStatus

BINARY_HEADSET = 0x01


class NetInterface:
    """
    Same public API as RosInterface, backed by a WebSocket connection
    to bridge_server.py instead of rclpy.
    """

    def __init__(self, url: str = "ws://10.0.0.1:9090"):
        self.url = url
        self._lock = threading.Lock()
        self._data: dict = {}
        self._headset_jpeg: bytes | None = None
        self._connected = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._cmd_queue: asyncio.Queue | None = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._cmd_queue = asyncio.Queue()
        self._loop.run_until_complete(self._connect_loop())

    async def _connect_loop(self):
        while self._running:
            try:
                async with websockets.connect(
                    self.url, max_size=10 * 1024 * 1024
                ) as ws:
                    self._connected = True
                    recv_task = asyncio.create_task(self._recv_loop(ws))
                    send_task = asyncio.create_task(self._send_loop(ws))
                    done, pending = await asyncio.wait(
                        [recv_task, send_task],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    for t in pending:
                        t.cancel()
            except (OSError, websockets.ConnectionClosed, ConnectionRefusedError):
                pass

            self._connected = False
            if self._running:
                await asyncio.sleep(1.0)

    async def _recv_loop(self, ws):
        async for message in ws:
            if isinstance(message, str):
                try:
                    data = json.loads(message)
                    if data.get("t") == "s":
                        with self._lock:
                            self._data = data
                except json.JSONDecodeError:
                    pass
            elif isinstance(message, bytes) and len(message) > 1:
                if message[0] == BINARY_HEADSET:
                    with self._lock:
                        self._headset_jpeg = message[1:]

    async def _send_loop(self, ws):
        while self._running:
            try:
                cmd = await asyncio.wait_for(
                    self._cmd_queue.get(), timeout=0.05
                )
                await ws.send(json.dumps(cmd))
            except asyncio.TimeoutError:
                pass

    def _send_cmd(self, cmd: dict):
        if self._loop and self._running:
            self._loop.call_soon_threadsafe(self._cmd_queue.put_nowait, cmd)

    # ── Public API (matches RosInterface) ─────────────────────────

    def get_status(self) -> DashboardStatus:
        with self._lock:
            d = self._data
            if not d:
                return DashboardStatus(
                    ros_connected=False,
                    robot_state=RobotState.DISCONNECTED,
                )

            topic_rates = {}
            for name, info in d.get("topic_rates", {}).items():
                topic_rates[name] = TopicStatus(
                    rx_count=info.get("rx_count", 0),
                    last_rx_time=info.get("last_rx_time", 0.0),
                    hz=info.get("hz", 0.0),
                )

            session_info = d.get("session_info", {})

            return DashboardStatus(
                robot_state=RobotState[d.get("robot_state", "DISCONNECTED")],
                ros_connected=self._connected and d.get("ros_connected", False),
                controller_active=d.get("controller_active", False),
                joint_names=d.get("joint_names", []),
                joint_positions=d.get("joint_positions", []),
                joint_velocities=d.get("joint_velocities", []),
                last_joint_time=d.get("last_joint_time", 0.0),
                joint_hz=d.get("joint_hz", 0.0),
                events=[(t, m) for t, m in d.get("events", [])],
                topic_rates=topic_rates,
                camera_width=d.get("camera_width", 0),
                camera_height=d.get("camera_height", 0),
                headset_jpeg=self._headset_jpeg,
                eef_pose=d.get("eef_pose"),
                target_pose=d.get("target_pose"),
                last_target_age_s=d.get("last_target_age_s", -1.0),
                last_twist_age_s=d.get("last_twist_age_s", -1.0),
                unity_map_loaded=d.get("unity_map_loaded"),
                session_info=session_info,
                gripper_value=d.get("gripper_value", 0.0),
                gripper_grips=d.get("gripper_grips", 0),
                collision_scale=d.get("collision_scale", 1.0),
                collision_blocked=d.get("collision_blocked", False),
                collision_events=d.get("collision_events", 0),
                ee_locked=d.get("ee_locked", False),
                ee_lock_count=d.get("ee_lock_count", 0),
                operating_mode=d.get("operating_mode", "TELEOP"),
                pick_service_ready=d.get("pick_service_ready", False),
                pick_request_pending=d.get("pick_request_pending", False),
                last_pick_cube=d.get("last_pick_cube", ""),
                last_pick_success=d.get("last_pick_success"),
                last_pick_message=d.get("last_pick_message", ""),
                pick_place_status=d.get("pick_place_status", ""),
                pick_place_status_lines=d.get("pick_place_status_lines", []),
                pick_place_block_id=d.get("pick_place_block_id", ""),
                pick_place_destination=d.get("pick_place_destination", ""),
                pick_place_step=d.get("pick_place_step", ""),
                pick_place_step_label=d.get("pick_place_step_label", ""),
                pick_place_step_index=d.get("pick_place_step_index", 0),
                pick_place_step_total=d.get("pick_place_step_total", 0),
                pick_place_state=d.get("pick_place_state", ""),
                pick_place_error=d.get("pick_place_error", ""),
                pick_place_error_detail=d.get("pick_place_error_detail", ""),
                calibration_ready=d.get("calibration_ready", False),
                calibration_running=d.get("calibration_running", False),
                calibration_state=d.get("calibration_state", ""),
                calibration_message=d.get("calibration_message", ""),
                calibration_sample_count=d.get("calibration_sample_count", 0),
                calibration_pose_index=d.get("calibration_pose_index", 0),
                calibration_pose_total=d.get("calibration_pose_total", 0),
                calibration_computed=d.get("calibration_computed", False),
                calibration_result=d.get("calibration_result", {}),
                calibration_latest_path=d.get("calibration_latest_path", ""),
                calibration_archive_path=d.get("calibration_archive_path", ""),
                calibration_error=d.get("calibration_error", ""),
                calibration_marker_frame=d.get("calibration_marker_frame", "tag36h11:1"),
                camera_type=d.get("camera_type", ""),
                headset_type=d.get("headset_type", ""),
                velocity_history=d.get("velocity_history", []),
                rate_history=d.get("rate_history", []),
                latency_history=d.get("latency_history", []),
                video_fps_history=d.get("video_fps_history", []),
            )

    def emergency_stop(self):
        self._send_cmd({"cmd": "estop"})

    def resume(self):
        self._send_cmd({"cmd": "resume"})

    def switch_to_teleop(self):
        self._send_cmd({"cmd": "switch_teleop"})

    def switch_to_moveit(self):
        self._send_cmd({"cmd": "switch_moveit"})

    def pick_cube_to_bin(self, cube_id, bin_id=None):
        self._send_cmd({
            "cmd": "pick_cube",
            "cube_id": cube_id,
            "bin_id": bin_id if bin_id is not None else cube_id,
        })

    def calibration_command(self, action: str):
        self._send_cmd({"cmd": "calibration", "action": action})


    def shutdown(self):
        self._running = False
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
