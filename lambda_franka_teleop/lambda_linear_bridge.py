#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt32


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value - deadband if value > 0.0 else value + deadband


def low_pass_filter(current_input: float, previous_output: float, alpha: float) -> float:
    return alpha * current_input + (1.0 - alpha) * previous_output


def adaptive_low_pass_filter(
    current_input: float,
    previous_output: float,
    alpha_slow: float,
    alpha_fast: float,
    adaptive_threshold: float,
) -> float:
    error = abs(current_input - previous_output)

    if adaptive_threshold <= 1e-12:
        alpha = alpha_fast
    else:
        weight = smoothstep01(error / adaptive_threshold)
        alpha = alpha_slow + weight * (alpha_fast - alpha_slow)

    return low_pass_filter(current_input, previous_output, alpha)


def limit_vector_norm(x: float, y: float, z: float, max_norm: float):
    norm = math.sqrt(x * x + y * y + z * z)

    if max_norm <= 0.0:
        return 0.0, 0.0, 0.0

    if norm > max_norm and norm > 1e-12:
        scale = max_norm / norm
        return x * scale, y * scale, z * scale

    return x, y, z


def smoothstep01(value: float) -> float:
    s = max(0.0, min(1.0, value))
    return s * s * (3.0 - 2.0 * s)


class LambdaLinearBridge(Node):
    def __init__(self) -> None:
        super().__init__("lambda_linear_bridge")

        self.declare_parameter("input_topic", "/twist")
        self.declare_parameter("output_topic", "/cmd_vel_raw")
        self.declare_parameter("output_frame", "base")
        self.declare_parameter("button_topic", "/buttons")

        self.declare_parameter("sign_x", -1.0)
        self.declare_parameter("sign_y", -1.0)
        self.declare_parameter("sign_z", 1.0)

        self.declare_parameter("sign_ax", -1.0)
        self.declare_parameter("sign_ay", -1.0)
        self.declare_parameter("sign_az", 0.0)

        self.declare_parameter("deadband_x", 0.0005)
        self.declare_parameter("deadband_y", 0.0005)
        self.declare_parameter("deadband_z", 0.0005)

        # Adaptive low-pass filter limits.
        # alpha_slow: very smooth near steady/noisy input.
        # alpha_fast: faster response when input changes intentionally.
        self.declare_parameter("filter_alpha_slow_x", 0.0007)
        self.declare_parameter("filter_alpha_slow_y", 0.0007)
        self.declare_parameter("filter_alpha_slow_z", 0.0007)

        self.declare_parameter("filter_alpha_fast_x", 0.0008)
        self.declare_parameter("filter_alpha_fast_y", 0.0008)
        self.declare_parameter("filter_alpha_fast_z", 0.0008)

        # Difference between raw input and filtered output needed to move
        # from slow filtering toward fast filtering.
        self.declare_parameter("adaptive_threshold_x", 0.0008)
        self.declare_parameter("adaptive_threshold_y", 0.0008)
        self.declare_parameter("adaptive_threshold_z", 0.0008)

        self.declare_parameter("output_deadband_x", 0.0)
        self.declare_parameter("output_deadband_y", 0.0)
        self.declare_parameter("output_deadband_z", 0.0)

        self.declare_parameter("max_linear_velocity", 0.25)
        self.declare_parameter("max_angular_velocity", 0.60)

        self.declare_parameter("enable_ramp_duration", 0.3)
        self.declare_parameter("disable_ramp_duration", 0.25)
        self.declare_parameter("decel_publish_period", 0.01)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.button_topic = self.get_parameter("button_topic").value

        self.linear_scale_x = 2.5
        self.linear_scale_y = 2.5
        self.linear_scale_z = 2.5

        self.angular_scale_x = 0.5
        self.angular_scale_y = 0.5
        self.angular_scale_z = 0.5

        self.sign_x = float(self.get_parameter("sign_x").value)
        self.sign_y = float(self.get_parameter("sign_y").value)
        self.sign_z = float(self.get_parameter("sign_z").value)

        self.sign_ax = float(self.get_parameter("sign_ax").value)
        self.sign_ay = float(self.get_parameter("sign_ay").value)
        self.sign_az = float(self.get_parameter("sign_az").value)

        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.deadband_z = float(self.get_parameter("deadband_z").value)

        self.filter_alpha_slow_x = float(self.get_parameter("filter_alpha_slow_x").value)
        self.filter_alpha_slow_y = float(self.get_parameter("filter_alpha_slow_y").value)
        self.filter_alpha_slow_z = float(self.get_parameter("filter_alpha_slow_z").value)

        self.filter_alpha_fast_x = float(self.get_parameter("filter_alpha_fast_x").value)
        self.filter_alpha_fast_y = float(self.get_parameter("filter_alpha_fast_y").value)
        self.filter_alpha_fast_z = float(self.get_parameter("filter_alpha_fast_z").value)

        self.adaptive_threshold_x = float(self.get_parameter("adaptive_threshold_x").value)
        self.adaptive_threshold_y = float(self.get_parameter("adaptive_threshold_y").value)
        self.adaptive_threshold_z = float(self.get_parameter("adaptive_threshold_z").value)

        self.output_deadband_x = float(self.get_parameter("output_deadband_x").value)
        self.output_deadband_y = float(self.get_parameter("output_deadband_y").value)
        self.output_deadband_z = float(self.get_parameter("output_deadband_z").value)

        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)

        self.enable_ramp_duration = float(self.get_parameter("enable_ramp_duration").value)
        self.disable_ramp_duration = float(self.get_parameter("disable_ramp_duration").value)
        self.decel_publish_period = float(self.get_parameter("decel_publish_period").value)

        if self.decel_publish_period <= 0.0:
            self.decel_publish_period = 0.01

        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0

        self.filtered_ax = 0.0
        self.filtered_ay = 0.0
        self.filtered_az = 0.0

        self.initialized = False

        self.enable_active = False
        self.last_button_value = 0
        self.enable_start_time = None

        self.decelerating = False
        self.disable_start_time = None

        self.last_cmd_linear = [0.0, 0.0, 0.0]
        self.last_cmd_angular = [0.0, 0.0, 0.0]

        self.decel_start_linear = [0.0, 0.0, 0.0]
        self.decel_start_angular = [0.0, 0.0, 0.0]

        self.sub = self.create_subscription(
            TwistStamped, self.input_topic, self.cb, 10
        )

        self.button_sub = self.create_subscription(
            UInt32, self.button_topic, self.button_cb, 10
        )

        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)

        self.decel_timer = self.create_timer(
            self.decel_publish_period, self.decel_cb
        )

        self.get_logger().info(f"Input:  {self.input_topic}")
        self.get_logger().info(f"Buttons: {self.button_topic}")
        self.get_logger().info(f"Output: {self.output_topic}")

        self.get_logger().info(
            f"Adaptive alpha slow: x={self.filter_alpha_slow_x} "
            f"y={self.filter_alpha_slow_y} z={self.filter_alpha_slow_z}"
        )
        self.get_logger().info(
            f"Adaptive alpha fast: x={self.filter_alpha_fast_x} "
            f"y={self.filter_alpha_fast_y} z={self.filter_alpha_fast_z}"
        )
        self.get_logger().info(
            f"Adaptive thresholds: x={self.adaptive_threshold_x} "
            f"y={self.adaptive_threshold_y} z={self.adaptive_threshold_z}"
        )

        self.get_logger().info("Motion enabled only when /buttons is 6 or 7")
        self.get_logger().info("Gripper disabled")

    def reset_filters(self) -> None:
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0

        self.filtered_ax = 0.0
        self.filtered_ay = 0.0
        self.filtered_az = 0.0

        self.initialized = False

    def publish_twist_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        ax: float,
        ay: float,
        az: float,
    ) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame

        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.linear.z = vz

        out.twist.angular.x = ax
        out.twist.angular.y = ay
        out.twist.angular.z = az

        self.last_cmd_linear = [vx, vy, vz]
        self.last_cmd_angular = [ax, ay, az]

        self.pub.publish(out)

    def start_disable_ramp(self) -> None:
        self.disable_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.decelerating = True

        self.decel_start_linear = list(self.last_cmd_linear)
        self.decel_start_angular = list(self.last_cmd_angular)

        self.reset_filters()

    def decel_cb(self) -> None:
        if not self.decelerating:
            return

        if self.enable_active:
            self.decelerating = False
            self.disable_start_time = None
            return

        if self.disable_start_time is None:
            self.decelerating = False
            self.publish_twist_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_sec - self.disable_start_time

        if self.disable_ramp_duration <= 1e-6:
            drop = 1.0
        else:
            drop = smoothstep01(elapsed / self.disable_ramp_duration)

        factor = 1.0 - drop

        if factor <= 1e-4:
            self.decelerating = False
            self.disable_start_time = None
            self.publish_twist_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        vx = self.decel_start_linear[0] * factor
        vy = self.decel_start_linear[1] * factor
        vz = self.decel_start_linear[2] * factor

        ax = self.decel_start_angular[0] * factor
        ay = self.decel_start_angular[1] * factor
        az = self.decel_start_angular[2] * factor

        self.publish_twist_command(vx, vy, vz, ax, ay, az)

    def button_cb(self, msg: UInt32) -> None:
        button_value = int(msg.data)
        was_enabled = self.enable_active
        new_enabled = button_value == 6 or button_value == 7

        self.last_button_value = button_value
        self.enable_active = new_enabled

        if self.enable_active and not was_enabled:
            self.decelerating = False
            self.disable_start_time = None
            self.enable_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.reset_filters()

        elif not self.enable_active and was_enabled:
            self.enable_start_time = None
            self.start_disable_ramp()

        elif not self.enable_active:
            self.enable_start_time = None

    def cb(self, msg: TwistStamped) -> None:
        if not self.enable_active:
            if not self.decelerating:
                self.publish_twist_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        raw_x = apply_deadband(msg.twist.linear.x, self.deadband_x)
        raw_y = apply_deadband(msg.twist.linear.y, self.deadband_y)
        raw_z = apply_deadband(msg.twist.linear.z, self.deadband_z)

        raw_ax = apply_deadband(msg.twist.angular.x, self.deadband_x)
        raw_ay = apply_deadband(msg.twist.angular.y, self.deadband_y)
        raw_az = apply_deadband(msg.twist.angular.z, self.deadband_z)

        if not self.initialized:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.filtered_z = raw_z

            self.filtered_ax = raw_ax
            self.filtered_ay = raw_ay
            self.filtered_az = raw_az

            self.initialized = True
        else:
            self.filtered_x = adaptive_low_pass_filter(
                raw_x,
                self.filtered_x,
                self.filter_alpha_slow_x,
                self.filter_alpha_fast_x,
                self.adaptive_threshold_x,
            )

            self.filtered_y = adaptive_low_pass_filter(
                raw_y,
                self.filtered_y,
                self.filter_alpha_slow_y,
                self.filter_alpha_fast_y,
                self.adaptive_threshold_y,
            )

            self.filtered_z = adaptive_low_pass_filter(
                raw_z,
                self.filtered_z,
                self.filter_alpha_slow_z,
                self.filter_alpha_fast_z,
                self.adaptive_threshold_z,
            )

            self.filtered_ax = adaptive_low_pass_filter(
                raw_ax,
                self.filtered_ax,
                self.filter_alpha_slow_x,
                self.filter_alpha_fast_x,
                self.adaptive_threshold_x,
            )

            self.filtered_ay = adaptive_low_pass_filter(
                raw_ay,
                self.filtered_ay,
                self.filter_alpha_slow_y,
                self.filter_alpha_fast_y,
                self.adaptive_threshold_y,
            )

            self.filtered_az = adaptive_low_pass_filter(
                raw_az,
                self.filtered_az,
                self.filter_alpha_slow_z,
                self.filter_alpha_fast_z,
                self.adaptive_threshold_z,
            )

        vx = self.sign_x * self.linear_scale_x * self.filtered_x
        vy = self.sign_y * self.linear_scale_y * self.filtered_y
        vz = self.sign_z * self.linear_scale_z * self.filtered_z

        ax = self.sign_ax * self.angular_scale_x * self.filtered_ax
        ay = self.sign_ay * self.angular_scale_y * self.filtered_ay
        az = self.sign_az * self.angular_scale_z * self.filtered_az

        if self.enable_start_time is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now_sec - self.enable_start_time

            if self.enable_ramp_duration <= 1e-6:
                ramp = 1.0
            else:
                ramp = smoothstep01(elapsed / self.enable_ramp_duration)
        else:
            ramp = 1.0

        vx *= ramp
        vy *= ramp
        vz *= ramp

        ax *= ramp
        ay *= ramp
        az *= ramp

        vx, vy, vz = limit_vector_norm(
            vx, vy, vz, self.max_linear_velocity
        )

        ax, ay, az = limit_vector_norm(
            ax, ay, az, self.max_angular_velocity
        )

        vx = apply_deadband(vx, self.output_deadband_x)
        vy = apply_deadband(vy, self.output_deadband_y)
        vz = apply_deadband(vz, self.output_deadband_z)

        ax = apply_deadband(ax, self.output_deadband_x)
        ay = apply_deadband(ay, self.output_deadband_y)
        az = apply_deadband(az, self.output_deadband_z)

        self.publish_twist_command(vx, vy, vz, ax, ay, az)


def main() -> None:
    rclpy.init()
    node = LambdaLinearBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
