import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from std_msgs.msg import String


JOINT_NAMES = {
    "1": "shoulder_pan_joint",
    "2": "shoulder_lift_joint",
    "3": "elbow_joint",
    "4": "wrist_1_joint",
    "5": "wrist_2_joint",
    "6": "wrist_3_joint",
}

ANTI_CLOCKWISE = -1
CLOCKWISE = 1
STOP = 0


class KeyboardJointTeleop(Node):
    def __init__(self) -> None:
        super().__init__("ur3_keyboard_teleop")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("hold_timeout_sec", 0.08)

        self.selected_joint = 1
        self.selected_joint_name = JOINT_NAMES[str(self.selected_joint)]
        self.active_direction = STOP
        self.last_motion_input_time = 0.0

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.hold_timeout_sec = float(self.get_parameter("hold_timeout_sec").value)

        self.selected_joint_pub = self.create_publisher(
            Int32, "/ur3_keyboard/selected_joint", 10
        )
        self.direction_pub = self.create_publisher(
            Int8, "/ur3_keyboard/joint_direction", 10
        )
        self.command_text_pub = self.create_publisher(
            String, "/ur3_keyboard/command_text", 10
        )

        self.create_timer(1.0 / publish_rate_hz, self.publish_motion_if_active)

        self.publish_selected_joint()
        self.publish_status("Selected joint 1 (shoulder_pan_joint)")

    def publish_selected_joint(self) -> None:
        msg = Int32()
        msg.data = self.selected_joint
        self.selected_joint_pub.publish(msg)

    def publish_direction(self, direction: int) -> None:
        msg = Int8()
        msg.data = direction
        self.direction_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.command_text_pub.publish(msg)
        self.get_logger().info(text)

    def set_joint(self, joint_key: str) -> None:
        self.selected_joint = int(joint_key)
        self.selected_joint_name = JOINT_NAMES[joint_key]
        self.publish_selected_joint()
        self.publish_status(
            f"Selected joint {self.selected_joint} ({self.selected_joint_name})"
        )

    def move_selected_joint(self, direction: int) -> None:
        previous_direction = self.active_direction
        self.active_direction = direction
        self.last_motion_input_time = time.monotonic()

        if direction == ANTI_CLOCKWISE:
            direction_name = "anticlockwise"
        elif direction == CLOCKWISE:
            direction_name = "clockwise"
        else:
            direction_name = "stop"

        if previous_direction != direction:
            self.publish_status(
                f"Move joint {self.selected_joint} ({self.selected_joint_name}) {direction_name}"
            )

    def stop_motion(self) -> None:
        if self.active_direction == STOP:
            return

        self.active_direction = STOP
        self.publish_selected_joint()
        self.publish_direction(STOP)
        self.publish_status(
            f"Stop joint {self.selected_joint} ({self.selected_joint_name})"
        )

    def publish_motion_if_active(self) -> None:
        if self.active_direction == STOP:
            return

        if time.monotonic() - self.last_motion_input_time > self.hold_timeout_sec:
            self.stop_motion()
            return

        self.publish_selected_joint()
        self.publish_direction(self.active_direction)


class KeyboardReader:
    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def read_key(self, timeout_sec: float = 0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return ""
        return sys.stdin.read(1)

    def close(self) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


def print_help() -> None:
    print("")
    print("UR3 Keyboard Teleop")
    print("-------------------")
    print("Press 1-6 to select a joint.")
    print("Press a to move anticlockwise.")
    print("Press d to move clockwise.")
    print("Press s to publish stop.")
    print("Press q to quit.")
    print("Hold a/d to keep moving. Releasing the key stops motion automatically.")
    print("")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardJointTeleop()
    keyboard = KeyboardReader()

    print_help()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = keyboard.read_key().lower()

            if not key:
                continue

            if key in JOINT_NAMES:
                node.set_joint(key)
            elif key == "a":
                node.move_selected_joint(ANTI_CLOCKWISE)
            elif key == "d":
                node.move_selected_joint(CLOCKWISE)
            elif key == "s":
                node.stop_motion()
            elif key == "q":
                node.publish_status("Quitting UR3 keyboard teleop")
                break
    except KeyboardInterrupt:
        node.publish_status("Keyboard interrupt received, stopping teleop")
    finally:
        keyboard.close()
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
