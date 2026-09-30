#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64, UInt32
from sensor_msgs.msg import JointState

try:
    from franka_msgs.action import Move
except ImportError:
    Move = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def low_pass_filter(current_input: float, previous_output: float, alpha: float) -> float:
    return alpha * current_input + (1.0 - alpha) * previous_output


class LambdaGripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("lambda_gripper_bridge")

        self.declare_parameter("gripper_topic", "/gripper/angle_rad")
        self.declare_parameter("button_topic", "/buttons")
        self.declare_parameter("gripper_action", "/NS_1/franka_gripper/move")
        self.declare_parameter("franka_joint_state_topic", "/NS_1/franka_gripper/joint_states")

        self.declare_parameter("require_enable_button", False)

        self.declare_parameter("lambda_gripper_max_angle", 0.26)
        self.declare_parameter("franka_gripper_min_width", 0.00)
        self.declare_parameter("franka_gripper_max_width", 0.08)

        self.declare_parameter("num_width_steps", 160)
        self.declare_parameter("gripper_speed", 0.09)
        self.declare_parameter("filter_alpha", 0.25)

        self.declare_parameter("min_send_period", 0.08)
        self.declare_parameter("retry_period", 0.05)

        # Important anti-oscillation parameters
        self.declare_parameter("same_width_deadband", 0.001)
        self.declare_parameter("unreachable_error_threshold", 0.003)
        self.declare_parameter("unlock_margin", 0.004)

        self.gripper_topic = self.get_parameter("gripper_topic").value
        self.button_topic = self.get_parameter("button_topic").value
        self.gripper_action = self.get_parameter("gripper_action").value
        self.franka_joint_state_topic = self.get_parameter("franka_joint_state_topic").value

        self.require_enable_button = bool(self.get_parameter("require_enable_button").value)

        self.lambda_gripper_max_angle = float(self.get_parameter("lambda_gripper_max_angle").value)
        self.franka_gripper_min_width = float(self.get_parameter("franka_gripper_min_width").value)
        self.franka_gripper_max_width = float(self.get_parameter("franka_gripper_max_width").value)

        self.num_width_steps = int(self.get_parameter("num_width_steps").value)
        self.gripper_speed = float(self.get_parameter("gripper_speed").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.min_send_period = float(self.get_parameter("min_send_period").value)
        self.retry_period = float(self.get_parameter("retry_period").value)

        self.same_width_deadband = float(self.get_parameter("same_width_deadband").value)
        self.unreachable_error_threshold = float(
            self.get_parameter("unreachable_error_threshold").value
        )
        self.unlock_margin = float(self.get_parameter("unlock_margin").value)

        if self.num_width_steps < 2:
            self.num_width_steps = 2

        if self.lambda_gripper_max_angle <= 1e-9:
            self.lambda_gripper_max_angle = 0.26

        if self.franka_gripper_max_width < self.franka_gripper_min_width:
            temp = self.franka_gripper_max_width
            self.franka_gripper_max_width = self.franka_gripper_min_width
            self.franka_gripper_min_width = temp

        step_size = (
            self.franka_gripper_max_width - self.franka_gripper_min_width
        ) / float(self.num_width_steps - 1)

        self.gripper_width_steps = [
            self.franka_gripper_min_width + i * step_size
            for i in range(self.num_width_steps)
        ]

        self.enable_active = False
        self.last_button_value = 0

        self.filtered_gripper_angle = 0.0
        self.gripper_initialized = False

        # Measured Franka gripper width from joint_states
        self.measured_width = None

        # Last accepted/reached width
        self.current_width = None

        # Command management
        self.pending_width = None
        self.in_flight_width = None
        self.goal_active = False

        # Blocked state: activated when Franka cannot reach commanded width
        self.blocked = False
        self.blocked_width = None

        self.last_send_time = 0.0
        self.last_server_log_time = 0.0

        self.gripper_client = None

        self.gripper_sub = self.create_subscription(
            Float64,
            self.gripper_topic,
            self.gripper_cb,
            10,
        )

        self.button_sub = self.create_subscription(
            UInt32,
            self.button_topic,
            self.button_cb,
            10,
        )

        self.joint_sub = self.create_subscription(
            JointState,
            self.franka_joint_state_topic,
            self.joint_state_cb,
            10,
        )

        if Move is not None:
            self.gripper_client = ActionClient(
                self,
                Move,
                self.gripper_action,
            )
        else:
            self.get_logger().error(
                "Could not import franka_msgs.action.Move. "
                "The gripper node will run, but cannot send gripper goals."
            )

        self.retry_timer = self.create_timer(self.retry_period, self.retry_cb)

        self.get_logger().info("Lambda gripper bridge started")
        self.get_logger().info(f"Lambda input topic: {self.gripper_topic}")
        self.get_logger().info(f"Franka joint state topic: {self.franka_joint_state_topic}")
        self.get_logger().info(f"Franka gripper action: {self.gripper_action}")

    def button_cb(self, msg: UInt32) -> None:
        button_value = int(msg.data)
        self.last_button_value = button_value
        self.enable_active = button_value == 6 or button_value == 7

    def joint_state_cb(self, msg: JointState) -> None:
        """
        Franka gripper has two finger joints.
        Total gripper opening width is normally:
            width = finger_1_position + finger_2_position
        """
        if len(msg.position) >= 2:
            width = float(msg.position[0] + msg.position[1])
            width = clamp(
                width,
                self.franka_gripper_min_width,
                self.franka_gripper_max_width,
            )
            self.measured_width = width

            if self.current_width is None:
                self.current_width = width

    def gripper_cb(self, msg: Float64) -> None:
        if self.require_enable_button and not self.enable_active:
            return

        angle = float(msg.data)
        angle = clamp(angle, 0.0, self.lambda_gripper_max_angle)

        if not self.gripper_initialized:
            self.filtered_gripper_angle = angle
            self.gripper_initialized = True
        else:
            self.filtered_gripper_angle = low_pass_filter(
                angle,
                self.filtered_gripper_angle,
                self.filter_alpha,
            )

        normalized = self.filtered_gripper_angle / self.lambda_gripper_max_angle
        normalized = clamp(normalized, 0.0, 1.0)

        idx = int(round(normalized * (len(self.gripper_width_steps) - 1)))
        idx = max(0, min(len(self.gripper_width_steps) - 1, idx))

        target_width = float(self.gripper_width_steps[idx])

        # If previous command was unreachable, lock at measured width.
        # Only allow command that opens/moves away from blocked object.
        if self.blocked:
            if self.blocked_width is None:
                return

            if target_width > self.blocked_width + self.unlock_margin:
                self.blocked = False
                self.blocked_width = None
                self.get_logger().info("Blocked state released by opening command")
            else:
                return

        self.pending_width = target_width
        self.try_send_pending_goal()

    def retry_cb(self) -> None:
        self.try_send_pending_goal()

    def try_send_pending_goal(self) -> None:
        if self.pending_width is None:
            return

        if self.require_enable_button and not self.enable_active:
            return

        if self.goal_active:
            return

        if self.blocked:
            return

        if self.gripper_client is None or Move is None:
            return

        reference_width = (
            self.measured_width
            if self.measured_width is not None
            else self.current_width
        )

        if reference_width is not None:
            if abs(self.pending_width - reference_width) < self.same_width_deadband:
                return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if (now_sec - self.last_send_time) < self.min_send_period:
            return

        if not self.gripper_client.wait_for_server(timeout_sec=0.0):
            if (now_sec - self.last_server_log_time) > 2.0:
                self.last_server_log_time = now_sec
                self.get_logger().warn(
                    f"Gripper action server not ready: {self.gripper_action}"
                )
            return

        width = float(self.pending_width)

        goal = Move.Goal()
        goal.width = width
        goal.speed = float(self.gripper_speed)

        self.goal_active = True
        self.in_flight_width = width
        self.last_send_time = now_sec

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("Gripper goal rejected")
                self.goal_active = False
                self.in_flight_width = None
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_cb)

        except Exception as exc:
            self.get_logger().warn(f"Gripper goal response failed: {exc}")
            self.goal_active = False
            self.in_flight_width = None

    def result_cb(self, future) -> None:
        try:
            result_wrapper = future.result()
            result = result_wrapper.result

            success = getattr(result, "success", True)

            if success:
                self.current_width = self.in_flight_width
            else:
                self.get_logger().warn(
                    "Goal not reached. Locking at last measured gripper width."
                )

                if self.measured_width is not None:
                    measured = self.measured_width
                elif self.current_width is not None:
                    measured = self.current_width
                else:
                    measured = self.in_flight_width

                self.current_width = measured
                self.pending_width = measured
                self.blocked_width = measured
                self.blocked = True

                self.get_logger().warn(
                    f"Blocked at measured width: {self.blocked_width:.4f} m"
                )

        except Exception as exc:
            self.get_logger().warn(f"Gripper result failed: {exc}")

            if self.measured_width is not None:
                self.current_width = self.measured_width
                self.pending_width = self.measured_width
                self.blocked_width = self.measured_width
                self.blocked = True

        self.goal_active = False
        self.in_flight_width = None

        if not self.blocked:
            self.try_send_pending_goal()


def main() -> None:
    rclpy.init()
    node = LambdaGripperBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
