#!/usr/bin/env python3
"""
ROS 2 node for OnRobot RG2 gripper control.

Subscribes to /gripper/command (Float32, 0.0=open → 1.0=closed) from Unity
and sends width commands to the RG2.

Three backends, tried in order:
  1. Modbus RTU over TCP — direct to RS485 Daemon port (no socat)
  2. URScript — sends rg_grip() via /urscript_interface/script_command
  3. Dry-run — logs commands without sending

The RG2 width range is 0–110 mm. This node maps:
  0.0 (trigger released) → 110 mm (fully open)
  1.0 (trigger fully pressed) → 0 mm (fully closed)

Usage:
  source /opt/ros/humble/setup.bash
  python3 gripper_node.py --robot-ip 192.168.0.194
  python3 gripper_node.py --robot-ip 192.168.0.194 --mode urscript
  python3 gripper_node.py --robot-ip 192.168.0.194 --mode modbus
  python3 gripper_node.py --dry-run
"""

import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

RG2_WIDTH_MAX = 1100   # 1/10 mm (fully open, 110mm)
MODBUS_SLAVE = 65
RG2_CTR_GRIP = 16
PUBLISH_RATE = 10.0


class GripperNode(Node):
    def __init__(self, robot_ip, daemon_port, force_n, mode, dry_run):
        super().__init__('gripper_controller')
        self.dry_run = dry_run
        self.force_reg = int(force_n * 10)
        self.last_width_reg = -1
        self.target_width_reg = RG2_WIDTH_MAX
        self.mode = None
        self.modbus_client = None
        self.urscript_pub = None

        self.sub = self.create_subscription(
            Float32, '/gripper/command', self.on_gripper_command, 10)

        if not dry_run:
            if mode in ('auto', 'modbus'):
                if self._try_modbus(robot_ip, daemon_port):
                    self.mode = 'modbus'

            if self.mode is None and mode in ('auto', 'urscript'):
                self._setup_urscript()
                self.mode = 'urscript'

            if self.mode is None:
                self.get_logger().error('All backends failed — running dry-run')
                self.dry_run = True

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_command)
        effective = 'dry-run' if self.dry_run else self.mode
        self.get_logger().info(f'Gripper node ready (mode={effective}, force={force_n}N)')

    def _try_modbus(self, robot_ip, daemon_port):
        try:
            from pymodbus.client.sync import ModbusTcpClient
            from pymodbus.transaction import ModbusRtuFramer
            from pymodbus.exceptions import ModbusIOException
        except ImportError:
            self.get_logger().warn('pymodbus not installed — skipping Modbus backend')
            return False

        try:
            client = ModbusTcpClient(
                host=robot_ip,
                port=daemon_port,
                framer_class=ModbusRtuFramer,
                timeout=2,
            )
            if not client.connect():
                self.get_logger().warn(f'Modbus TCP connect to {robot_ip}:{daemon_port} failed')
                return False

            result = client.read_holding_registers(address=267, count=1, unit=MODBUS_SLAVE)
            if not isinstance(result, ModbusIOException) and hasattr(result, 'registers') and result.registers:
                width_mm = result.registers[0] / 10.0
                self.get_logger().info(f'Modbus RTU/TCP connected — gripper width: {width_mm:.1f} mm')
                self.modbus_client = client
                return True
            else:
                self.get_logger().warn(f'Modbus connected but gripper not responding: {result}')
                client.close()
                return False
        except Exception as e:
            self.get_logger().warn(f'Modbus backend failed: {e}')
            return False

    def _setup_urscript(self):
        self.urscript_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 1)
        self.get_logger().info('Using URScript backend via /urscript_interface/script_command')

    def on_gripper_command(self, msg: Float32):
        value = max(0.0, min(1.0, msg.data))
        self.target_width_reg = int(RG2_WIDTH_MAX - value * RG2_WIDTH_MAX)

    def publish_command(self):
        width = self.target_width_reg
        if abs(width - self.last_width_reg) < 5:
            return
        self.last_width_reg = width

        if self.dry_run:
            self.get_logger().info(f'[dry-run] width={width / 10.0:.1f}mm force={self.force_reg / 10.0:.0f}N')
            return

        if self.mode == 'modbus':
            self._send_modbus(width)
        elif self.mode == 'urscript':
            self._send_urscript(width)

    def _send_modbus(self, width):
        from pymodbus.exceptions import ModbusIOException
        try:
            params = [self.force_reg, width, RG2_CTR_GRIP]
            result = self.modbus_client.write_registers(address=0, values=params, unit=MODBUS_SLAVE)
            if isinstance(result, ModbusIOException):
                self.get_logger().warn(f'Modbus write error: {result}')
            else:
                self.get_logger().info(f'Gripper → {width / 10.0:.1f} mm')
        except Exception as e:
            self.get_logger().error(f'Modbus exception: {e}')

    def _send_urscript(self, width):
        script = (
            f'def gripper_cmd():\n'
            f'  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)\n'
            f'  sleep(0.01)\n'
            f'  write_port_register(0, {self.force_reg})\n'
            f'  write_port_register(1, {width})\n'
            f'  write_port_register(2, {RG2_CTR_GRIP})\n'
            f'end\n'
        )
        msg = String()
        msg.data = script
        self.urscript_pub.publish(msg)
        self.get_logger().info(f'URScript → {width / 10.0:.1f} mm')

    def destroy_node(self):
        if self.modbus_client:
            self.modbus_client.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='OnRobot RG2 gripper ROS 2 node')
    parser.add_argument('--robot-ip', type=str, default='192.168.0.194')
    parser.add_argument('--daemon-port', type=int, default=54321,
                        help='RS485 Daemon TCP port (default: 54321)')
    parser.add_argument('--force', type=float, default=40.0,
                        help='Grip force in Newtons (default: 40)')
    parser.add_argument('--mode', choices=['auto', 'modbus', 'urscript'], default='auto',
                        help='Backend: auto tries modbus then urscript (default: auto)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Log commands without sending')
    args = parser.parse_args()

    rclpy.init()
    node = GripperNode(
        robot_ip=args.robot_ip,
        daemon_port=args.daemon_port,
        force_n=args.force,
        mode=args.mode,
        dry_run=args.dry_run,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
