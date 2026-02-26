import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import Buffer, TransformListener

class PoseToTwistServo(Node):
    def __init__(self):
        super().__init__("pose_to_twist_servo")

        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("eef_frame", "tool0")
        self.declare_parameter("target_topic", "/servo_target_pose")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("k_lin", 1.5)
        self.declare_parameter("max_lin", 0.25)

        self.command_frame = self.get_parameter("command_frame").value
        self.eef_frame = self.get_parameter("eef_frame").value
        self.k_lin = float(self.get_parameter("k_lin").value)
        self.max_lin = float(self.get_parameter("max_lin").value)

        self.target_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("target_topic").value,
            self.on_target,
            10
        )
        self.pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            10
        )

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def on_target(self, msg: PoseStamped):
        self.target_pose = msg

    def clamp(self, v, vmax):
        return max(-vmax, min(vmax, v))

    def tick(self):
        if self.target_pose is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.eef_frame,
                rclpy.time.Time()
            )
        except Exception:
            return

        # current position of EEF in command_frame
        cx = tf.transform.translation.x
        cy = tf.transform.translation.y
        cz = tf.transform.translation.z

        # desired position (assume already in command_frame for now)
        dx = self.target_pose.pose.position.x - cx
        dy = self.target_pose.pose.position.y - cy
        dz = self.target_pose.pose.position.z - cz

        vx = self.clamp(self.k_lin * dx, self.max_lin)
        vy = self.clamp(self.k_lin * dy, self.max_lin)
        vz = self.clamp(self.k_lin * dz, self.max_lin)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        # keep angular zero for MVP (add orientation tracking later)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PoseToTwistServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()