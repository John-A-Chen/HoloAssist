"""
HoloAssist System Manager Node
===============================
Central supervisor that:
 - Manages operating mode (MANUAL / HYBRID)
 - Monitors heartbeats from all subsystem nodes
 - Publishes system-wide status diagnostics
 - Exposes services for mode switching and system queries
"""

from enum import IntEnum
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ── Operating modes ──────────────────────────────────────────────────────────

class OperatingMode(IntEnum):
    """System-wide operating mode."""
    MANUAL = 0   # Full human-in-the-loop teleoperation
    HYBRID = 1   # Combined manual + autonomous behaviour


# ── Subsystem descriptor ─────────────────────────────────────────────────────

class SubsystemInfo:
    """Tracks health / liveness of one supervised subsystem."""

    def __init__(self, name: str, heartbeat_topic: str, timeout_sec: float = 5.0):
        self.name = name
        self.heartbeat_topic = heartbeat_topic
        self.timeout_sec = timeout_sec
        self.last_seen: float = 0.0

    @property
    def alive(self) -> bool:
        return (time.monotonic() - self.last_seen) < self.timeout_sec


# ── Manager node ──────────────────────────────────────────────────────────────

class ManagerNode(Node):
    """
    Central manager for the HoloAssist framework.

    Published topics
    ----------------
    ~/mode              (std_msgs/String)       Current operating mode name
    ~/diagnostics       (diagnostic_msgs/DiagnosticArray)  Per-subsystem health

    Services
    --------
    ~/set_manual        (std_srvs/Trigger)  Switch to MANUAL mode
    ~/set_hybrid        (std_srvs/Trigger)  Switch to HYBRID mode
    ~/get_mode          (std_srvs/Trigger)  Query current mode
    ~/system_status     (std_srvs/Trigger)  Return aggregated subsystem health
    """

    # Default set of supervised subsystems – extend as new nodes appear.
    DEFAULT_SUBSYSTEMS = [
        SubsystemInfo('xr_interface',   '/xr_interface/heartbeat'),
        SubsystemInfo('perception',     '/perception/heartbeat'),
        SubsystemInfo('planning',       '/planning/heartbeat'),
        SubsystemInfo('control',        '/control/heartbeat'),
        SubsystemInfo('rviz',           '/rviz/heartbeat',           timeout_sec=10.0),
        SubsystemInfo('unity_bridge',   '/unity_bridge/heartbeat'),
        SubsystemInfo('robot_bringup',  '/robot_bringup/heartbeat'),
    ]

    def __init__(self) -> None:
        super().__init__('holoassist_manager')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('initial_mode', 'MANUAL')
        self.declare_parameter('status_publish_rate', 1.0)       # Hz
        self.declare_parameter('heartbeat_timeout_sec', 5.0)

        initial_mode_str = self.get_parameter('initial_mode').value
        self._mode = OperatingMode[initial_mode_str.upper()]
        self._status_rate = self.get_parameter('status_publish_rate').value
        default_timeout = self.get_parameter('heartbeat_timeout_sec').value

        # ── subsystem registry ────────────────────────────────────────────
        self._subsystems: list[SubsystemInfo] = []
        for s in self.DEFAULT_SUBSYSTEMS:
            s.timeout_sec = default_timeout
            self._subsystems.append(s)

        # ── publishers ────────────────────────────────────────────────────
        latching_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_pub = self.create_publisher(String, '~/mode', latching_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)

        # ── services ──────────────────────────────────────────────────────
        self.create_service(Trigger, '~/set_manual',    self._srv_set_manual)
        self.create_service(Trigger, '~/set_hybrid',    self._srv_set_hybrid)
        self.create_service(Trigger, '~/get_mode',      self._srv_get_mode)
        self.create_service(Trigger, '~/system_status', self._srv_system_status)

        # ── heartbeat subscribers ─────────────────────────────────────────
        for sub in self._subsystems:
            self.create_subscription(
                String,
                sub.heartbeat_topic,
                lambda msg, s=sub: self._on_heartbeat(s, msg),
                10,
            )

        # ── periodic diagnostics timer ────────────────────────────────────
        period = 1.0 / max(self._status_rate, 0.01)
        self._diag_timer = self.create_timer(period, self._publish_diagnostics)

        # Publish the initial mode immediately
        self._publish_mode()
        self.get_logger().info(
            f'HoloAssist Manager started – mode={self._mode.name}'
        )

    # ── mode helpers ──────────────────────────────────────────────────────

    @property
    def mode(self) -> OperatingMode:
        return self._mode

    def _set_mode(self, new_mode: OperatingMode) -> None:
        old = self._mode
        self._mode = new_mode
        self._publish_mode()
        if old != new_mode:
            self.get_logger().info(f'Mode changed: {old.name} → {new_mode.name}')

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode.name
        self._mode_pub.publish(msg)

    # ── heartbeat callback ────────────────────────────────────────────────

    def _on_heartbeat(self, subsystem: SubsystemInfo, _msg: String) -> None:
        subsystem.last_seen = time.monotonic()

    # ── diagnostics timer ─────────────────────────────────────────────────

    def _publish_diagnostics(self) -> None:
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        for sub in self._subsystems:
            status = DiagnosticStatus()
            status.name = f'holoassist/{sub.name}'
            status.hardware_id = sub.name
            if sub.alive:
                status.level = DiagnosticStatus.OK
                status.message = 'alive'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = 'no heartbeat'
            status.values = [
                KeyValue(key='heartbeat_topic', value=sub.heartbeat_topic),
                KeyValue(key='last_seen', value=str(sub.last_seen)),
            ]
            diag.status.append(status)

        # Overall mode entry
        mode_status = DiagnosticStatus()
        mode_status.name = 'holoassist/mode'
        mode_status.level = DiagnosticStatus.OK
        mode_status.message = self._mode.name
        diag.status.append(mode_status)

        self._diag_pub.publish(diag)

    # ── service callbacks ─────────────────────────────────────────────────

    def _srv_set_manual(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self._set_mode(OperatingMode.MANUAL)
        resp.success = True
        resp.message = 'Mode set to MANUAL'
        return resp

    def _srv_set_hybrid(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self._set_mode(OperatingMode.HYBRID)
        resp.success = True
        resp.message = 'Mode set to HYBRID'
        return resp

    def _srv_get_mode(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        resp.success = True
        resp.message = self._mode.name
        return resp

    def _srv_system_status(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        alive = [s.name for s in self._subsystems if s.alive]
        down = [s.name for s in self._subsystems if not s.alive]
        resp.success = len(down) == 0
        resp.message = (
            f'mode={self._mode.name}  '
            f'alive=[{", ".join(alive)}]  '
            f'down=[{", ".join(down)}]'
        )
        return resp


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
