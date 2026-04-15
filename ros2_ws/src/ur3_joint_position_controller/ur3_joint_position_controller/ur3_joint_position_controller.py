import time
from typing import Dict, List, Optional

import rclpy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.msg import SafetyMode


JOINT_NUMBER_TO_NAME = {
    1: "shoulder_pan_joint",
    2: "shoulder_lift_joint",
    3: "elbow_joint",
    4: "wrist_1_joint",
    5: "wrist_2_joint",
    6: "wrist_3_joint",
}

CONTROLLER_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

SCALED_CONTROLLER_NAME = "scaled_joint_trajectory_controller"
RUNNING_ROBOT_MODE = RobotMode.RUNNING
SAFE_SAFETY_MODES = {SafetyMode.NORMAL, SafetyMode.REDUCED}


class UR3JointPositionController(Node):
    def __init__(self) -> None:
        super().__init__("ur3_joint_position_controller")

        self.declare_parameter("step_size_rad", 0.05)
        self.declare_parameter(
            "command_topic", "/scaled_joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("selected_joint_topic", "/ur3_keyboard/selected_joint")
        self.declare_parameter("direction_topic", "/ur3_keyboard/joint_direction")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("status_topic", "/ur3_keyboard/robot_command_text")
        self.declare_parameter(
            "controller_state_topic", "/scaled_joint_trajectory_controller/state"
        )
        self.declare_parameter(
            "robot_program_running_topic", "/io_and_status_controller/robot_program_running"
        )
        self.declare_parameter("robot_mode_topic", "/io_and_status_controller/robot_mode")
        self.declare_parameter("safety_mode_topic", "/io_and_status_controller/safety_mode")
        self.declare_parameter("min_position_rad", -6.28)
        self.declare_parameter("max_position_rad", 6.28)
        self.declare_parameter("trajectory_duration_sec", 0.5)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("hold_timeout_sec", 0.12)
        self.declare_parameter("jog_speed_rad_per_sec", 0.4)

        self.step_size_rad = float(self.get_parameter("step_size_rad").value)
        self.min_position_rad = float(self.get_parameter("min_position_rad").value)
        self.max_position_rad = float(self.get_parameter("max_position_rad").value)
        self.trajectory_duration_sec = float(
            self.get_parameter("trajectory_duration_sec").value
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.hold_timeout_sec = float(self.get_parameter("hold_timeout_sec").value)
        self.jog_speed_rad_per_sec = float(
            self.get_parameter("jog_speed_rad_per_sec").value
        )

        command_topic = str(self.get_parameter("command_topic").value)
        selected_joint_topic = str(self.get_parameter("selected_joint_topic").value)
        direction_topic = str(self.get_parameter("direction_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        controller_state_topic = str(self.get_parameter("controller_state_topic").value)
        robot_program_running_topic = str(
            self.get_parameter("robot_program_running_topic").value
        )
        robot_mode_topic = str(self.get_parameter("robot_mode_topic").value)
        safety_mode_topic = str(self.get_parameter("safety_mode_topic").value)

        self.selected_joint = 1
        self.latest_positions: Dict[str, float] = {}
        self.latest_command: Optional[List[float]] = None
        self.controller_active: Optional[bool] = None
        self.controller_state_seen = False
        self.robot_program_running: Optional[bool] = None
        self.robot_mode: Optional[int] = None
        self.safety_mode: Optional[int] = None
        self.active_direction = 0
        self.last_direction_input_time = 0.0
        self.last_readiness_issue: Optional[str] = None

        transient_status_qos = QoSProfile(depth=1)
        transient_status_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.command_pub = self.create_publisher(JointTrajectory, command_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.controller_manager_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.create_subscription(Int32, selected_joint_topic, self.selected_joint_callback, 10)
        self.create_subscription(Int8, direction_topic, self.direction_callback, 10)
        self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            controller_state_topic,
            self.controller_state_callback,
            10,
        )
        self.create_subscription(
            Bool,
            robot_program_running_topic,
            self.robot_program_running_callback,
            transient_status_qos,
        )
        self.create_subscription(
            RobotMode,
            robot_mode_topic,
            self.robot_mode_callback,
            transient_status_qos,
        )
        self.create_subscription(
            SafetyMode,
            safety_mode_topic,
            self.safety_mode_callback,
            transient_status_qos,
        )
        self.create_timer(1.0, self.refresh_controller_status)
        self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.publish_status(
            "UR3 joint position controller ready. Waiting for /joint_states and robot readiness before sending commands."
        )

    def selected_joint_callback(self, msg: Int32) -> None:
        if msg.data not in JOINT_NUMBER_TO_NAME:
            self.publish_status(f"Ignoring invalid selected joint {msg.data}")
            return

        self.selected_joint = msg.data
        self.publish_status(
            f"Active joint set to {self.selected_joint} ({JOINT_NUMBER_TO_NAME[self.selected_joint]})"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self.publish_status("Received malformed /joint_states message, ignoring it")
            return

        self.latest_positions = {
            joint_name: joint_position
            for joint_name, joint_position in zip(msg.name, msg.position)
        }

    def controller_state_callback(self, _: JointTrajectoryControllerState) -> None:
        self.controller_state_seen = True

    def robot_program_running_callback(self, msg: Bool) -> None:
        self.robot_program_running = msg.data

    def robot_mode_callback(self, msg: RobotMode) -> None:
        self.robot_mode = msg.mode

    def safety_mode_callback(self, msg: SafetyMode) -> None:
        self.safety_mode = msg.mode

    def refresh_controller_status(self) -> None:
        if not self.controller_manager_client.service_is_ready():
            if not self.controller_manager_client.wait_for_service(timeout_sec=0.0):
                return

        future = self.controller_manager_client.call_async(ListControllers.Request())
        future.add_done_callback(self.handle_controller_list_response)

    def handle_controller_list_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().debug(f"Failed to refresh controller state: {exc}")
            return

        for controller in response.controller:
            if controller.name == SCALED_CONTROLLER_NAME:
                self.controller_active = controller.state == "active"
                return

        self.controller_active = False

    def activate_scaled_controller(self, timeout_sec: float = 5.0) -> bool:
        if not self.switch_controller_client.wait_for_service(timeout_sec=timeout_sec):
            self.publish_status("/controller_manager/switch_controller is not available")
            return False

        request = SwitchController.Request()
        request.activate_controllers = [SCALED_CONTROLLER_NAME]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(timeout_sec)
        request.timeout.nanosec = 0

        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec + 2.0)
        response = future.result()

        if response is None or not response.ok:
            self.publish_status(
                "Failed to activate scaled_joint_trajectory_controller through controller_manager"
            )
            return False

        self.publish_status("Activated scaled_joint_trajectory_controller")
        self.refresh_controller_status()
        return True

    def robot_ready_status(self) -> Optional[str]:
        if not self.have_all_joint_positions():
            return "Cannot move robot yet because not all UR3 joint states have been received"

        if self.controller_active is False:
            if self.activate_scaled_controller():
                return None
            return "Cannot move robot because scaled_joint_trajectory_controller is not active"

        if self.controller_active is None and not self.controller_state_seen:
            return (
                "Cannot confirm controller readiness yet because scaled_joint_trajectory_controller status is unknown"
            )

        if self.robot_program_running is False:
            return "Cannot move robot because the External Control program is not running"

        if self.robot_program_running is None:
            return "Cannot confirm robot readiness yet because robot_program_running has not been received"

        if self.robot_mode is not None and self.robot_mode != RUNNING_ROBOT_MODE:
            return f"Cannot move robot because robot_mode is {self.robot_mode}, not RUNNING"

        if self.robot_mode is None:
            return "Cannot confirm robot readiness yet because robot_mode has not been received"

        if self.safety_mode is not None and self.safety_mode not in SAFE_SAFETY_MODES:
            return (
                f"Cannot move robot because safety_mode is {self.safety_mode}, not NORMAL or REDUCED"
            )

        if self.safety_mode is None:
            return "Cannot confirm robot readiness yet because safety_mode has not been received"

        return None

    def direction_callback(self, msg: Int8) -> None:
        if msg.data == 0:
            self.active_direction = 0
            self.publish_hold_position()
            self.publish_status("Received stop command")
            return

        if msg.data not in (-1, 1):
            self.publish_status(f"Ignoring invalid direction {msg.data}")
            return

        self.active_direction = msg.data
        self.last_direction_input_time = time.monotonic()
        self.last_readiness_issue = None

    def control_loop(self) -> None:
        if self.active_direction == 0:
            return

        if time.monotonic() - self.last_direction_input_time > self.hold_timeout_sec:
            self.active_direction = 0
            self.publish_hold_position()
            self.publish_status(
                f"Stop joint {self.selected_joint} ({JOINT_NUMBER_TO_NAME[self.selected_joint]})"
            )
            return

        readiness_issue = self.robot_ready_status()
        if readiness_issue is not None:
            if readiness_issue != self.last_readiness_issue:
                self.publish_status(readiness_issue)
                self.last_readiness_issue = readiness_issue
            return

        self.last_readiness_issue = None
        joint_name = JOINT_NUMBER_TO_NAME[self.selected_joint]
        target_positions = self.build_target_positions()
        joint_index = CONTROLLER_JOINT_ORDER.index(joint_name)
        joint_step = self.jog_speed_rad_per_sec / self.control_rate_hz

        target_positions[joint_index] += self.active_direction * joint_step
        target_positions[joint_index] = max(
            self.min_position_rad,
            min(self.max_position_rad, target_positions[joint_index]),
        )

        self.publish_command(target_positions)

    def have_all_joint_positions(self) -> bool:
        return all(joint_name in self.latest_positions for joint_name in CONTROLLER_JOINT_ORDER)

    def build_target_positions(self) -> List[float]:
        if self.latest_command is not None:
            return list(self.latest_command)

        return [self.latest_positions[joint_name] for joint_name in CONTROLLER_JOINT_ORDER]

    def publish_command(self, target_positions: List[float]) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(CONTROLLER_JOINT_ORDER)

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start.sec = int(self.trajectory_duration_sec)
        point.time_from_start.nanosec = int(
            (self.trajectory_duration_sec - int(self.trajectory_duration_sec)) * 1e9
        )
        msg.points = [point]

        self.command_pub.publish(msg)
        self.latest_command = list(target_positions)

    def publish_hold_position(self) -> None:
        if not self.have_all_joint_positions():
            return

        hold_positions = [self.latest_positions[joint_name] for joint_name in CONTROLLER_JOINT_ORDER]
        self.publish_command(hold_positions)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UR3JointPositionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
