#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, WrenchStamped
from std_msgs.msg import Float64, UInt8
from franka_msgs.msg import FrankaRobotState


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_delta(current: float, target: float, max_step: float) -> float:
    delta = target - current

    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step

    return current + delta


def clamp_norm3(x: float, y: float, z: float, max_norm: float):
    n = math.sqrt(x * x + y * y + z * z)

    if max_norm > 0.0 and n > max_norm and n > 1e-9:
        s = max_norm / n
        return x * s, y * s, z * s

    return x, y, z


def smoothstep(s: float) -> float:
    s = clamp(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


class SmoothedCmdVel(Node):

    def __init__(self) -> None:
        super().__init__("smoothed_cmd_vel")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter(
            "input_topic",
            "/cmd_vel_raw"
        )

        self.declare_parameter(
            "output_topic",
            "/NS_1/mobile_cartesian_velocity_controller/cmd_vel",
        )

        # ============================================================
        # NEW: diagnostic pre-depth filtered velocity topic
        # ============================================================

        self.declare_parameter(
            "pre_depth_output_topic",
            "/cmd_vel_filtered_pre_depth",
        )

        self.declare_parameter(
            "frame_id",
            "base"
        )

        self.declare_parameter(
            "publish_rate_hz",
            500.0
        )

        self.declare_parameter(
            "command_timeout_sec",
            0.50
        )

        self.declare_parameter(
            "max_linear_vel_x",
            3.5
        )

        self.declare_parameter(
            "max_linear_vel_y",
            3.5
        )

        self.declare_parameter(
            "max_linear_vel_z",
            3.5
        )

        self.declare_parameter(
            "linear_accel_limit_x",
            2.30
        )

        self.declare_parameter(
            "linear_accel_limit_y",
            2.30
        )

        self.declare_parameter(
            "linear_accel_limit_z",
            2.30
        )

        self.declare_parameter(
            "max_angular_vel_x",
            3.50
        )

        self.declare_parameter(
            "max_angular_vel_y",
            3.20
        )

        self.declare_parameter(
            "max_angular_vel_z",
            3.20
        )

        self.declare_parameter(
            "angular_accel_limit_x",
            2.50
        )

        self.declare_parameter(
            "angular_accel_limit_y",
            2.50
        )

        self.declare_parameter(
            "angular_accel_limit_z",
            2.50
        )

        self.declare_parameter(
            "depth_topic",
            "/rgb_depth"
        )

        self.declare_parameter(
            "depth_timeout_sec",
            1.0
        )

        self.declare_parameter(
            "d_safe",
            0.16
        )

        self.declare_parameter(
            "d_stop",
            0.07
        )

        self.declare_parameter(
            "min_depth_scale",
            0.20
        )

        self.declare_parameter(
            "alpha_rate_limit",
            0.25
        )

        self.declare_parameter(
            "depth_filter_alpha",
            0.02
        )

        self.declare_parameter(
            "fruit_grasp_topic",
            "/fruit_grasp"
        )

        self.declare_parameter(
            "franka_robot_state_topic",
            "/NS_1/franka_robot_state_broadcaster/robot_state",
        )

        self.declare_parameter(
            "franka_state_timeout_sec",
            0.20
        )

        self.declare_parameter(
            "franka_velocity_filter_alpha",
            0.20
        )

        self.declare_parameter(
            "force_feedback_topic",
            "/cmd/force_feedback/wrench"
        )

        self.declare_parameter(
            "force_feedback_frame_id",
            "lambda7_base"
        )

        self.declare_parameter(
            "velocity_error_force_gain",
            80.0
        )

        self.declare_parameter(
            "tracking_force_gain",
            40.0
        )

        self.declare_parameter(
            "tracking_velocity_deadband",
            0.005
        )

        self.declare_parameter(
            "invert_velocity_error_force",
            False
        )

        self.declare_parameter(
            "invert_tracking_force",
            False
        )

        self.declare_parameter(
            "max_feedback_force",
            7.5
        )

        self.declare_parameter(
            "depth_log_rate_hz",
            0.5
        )

        # ============================================================
        # Read parameters
        # ============================================================

        self.input_topic = (
            self.get_parameter("input_topic").value
        )

        self.output_topic = (
            self.get_parameter("output_topic").value
        )

        # ============================================================
        # NEW
        # ============================================================

        self.pre_depth_output_topic = (
            self.get_parameter(
                "pre_depth_output_topic"
            ).value
        )

        self.frame_id = (
            self.get_parameter("frame_id").value
        )

        self.publish_rate_hz = float(
            self.get_parameter(
                "publish_rate_hz"
            ).value
        )

        self.command_timeout_sec = float(
            self.get_parameter(
                "command_timeout_sec"
            ).value
        )

        self.max_linear_vel_x = float(
            self.get_parameter(
                "max_linear_vel_x"
            ).value
        )

        self.max_linear_vel_y = float(
            self.get_parameter(
                "max_linear_vel_y"
            ).value
        )

        self.max_linear_vel_z = float(
            self.get_parameter(
                "max_linear_vel_z"
            ).value
        )

        self.linear_accel_limit_x = float(
            self.get_parameter(
                "linear_accel_limit_x"
            ).value
        )

        self.linear_accel_limit_y = float(
            self.get_parameter(
                "linear_accel_limit_y"
            ).value
        )

        self.linear_accel_limit_z = float(
            self.get_parameter(
                "linear_accel_limit_z"
            ).value
        )

        self.max_angular_vel_x = float(
            self.get_parameter(
                "max_angular_vel_x"
            ).value
        )

        self.max_angular_vel_y = float(
            self.get_parameter(
                "max_angular_vel_y"
            ).value
        )

        self.max_angular_vel_z = float(
            self.get_parameter(
                "max_angular_vel_z"
            ).value
        )

        self.angular_accel_limit_x = float(
            self.get_parameter(
                "angular_accel_limit_x"
            ).value
        )

        self.angular_accel_limit_y = float(
            self.get_parameter(
                "angular_accel_limit_y"
            ).value
        )

        self.angular_accel_limit_z = float(
            self.get_parameter(
                "angular_accel_limit_z"
            ).value
        )

        self.depth_topic = (
            self.get_parameter(
                "depth_topic"
            ).value
        )

        self.depth_timeout_sec = float(
            self.get_parameter(
                "depth_timeout_sec"
            ).value
        )

        self.d_safe = float(
            self.get_parameter(
                "d_safe"
            ).value
        )

        self.d_stop = float(
            self.get_parameter(
                "d_stop"
            ).value
        )

        self.min_depth_scale = float(
            self.get_parameter(
                "min_depth_scale"
            ).value
        )

        self.alpha_rate_limit = float(
            self.get_parameter(
                "alpha_rate_limit"
            ).value
        )

        self.depth_filter_alpha = float(
            self.get_parameter(
                "depth_filter_alpha"
            ).value
        )

        self.fruit_grasp_topic = (
            self.get_parameter(
                "fruit_grasp_topic"
            ).value
        )

        self.franka_robot_state_topic = (
            self.get_parameter(
                "franka_robot_state_topic"
            ).value
        )

        self.franka_state_timeout_sec = float(
            self.get_parameter(
                "franka_state_timeout_sec"
            ).value
        )

        self.franka_velocity_filter_alpha = float(
            self.get_parameter(
                "franka_velocity_filter_alpha"
            ).value
        )

        self.force_feedback_topic = (
            self.get_parameter(
                "force_feedback_topic"
            ).value
        )

        self.force_feedback_frame_id = (
            self.get_parameter(
                "force_feedback_frame_id"
            ).value
        )

        self.velocity_error_force_gain = float(
            self.get_parameter(
                "velocity_error_force_gain"
            ).value
        )

        self.tracking_force_gain = float(
            self.get_parameter(
                "tracking_force_gain"
            ).value
        )

        self.tracking_velocity_deadband = float(
            self.get_parameter(
                "tracking_velocity_deadband"
            ).value
        )

        self.invert_velocity_error_force = bool(
            self.get_parameter(
                "invert_velocity_error_force"
            ).value
        )

        self.invert_tracking_force = bool(
            self.get_parameter(
                "invert_tracking_force"
            ).value
        )

        self.max_feedback_force = float(
            self.get_parameter(
                "max_feedback_force"
            ).value
        )

        self.depth_log_rate_hz = float(
            self.get_parameter(
                "depth_log_rate_hz"
            ).value
        )

        self.depth_log_period = (
            1.0 / self.depth_log_rate_hz
            if self.depth_log_rate_hz > 0.0
            else 2.0
        )

        self.last_depth_log_time = 0.0

        # ============================================================
        # State
        # ============================================================

        self.depth_scaling_enabled = True
        self.fruit_grasp_state = 0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # ============================================================
        # NEW:
        # Independent diagnostic acceleration-limited signal
        # before depth scaling.
        #
        # This does NOT affect self.current_x/y/z.
        # ============================================================

        self.pre_depth_x = 0.0
        self.pre_depth_y = 0.0
        self.pre_depth_z = 0.0

        self.target_ax = 0.0
        self.target_ay = 0.0
        self.target_az = 0.0

        self.current_ax = 0.0
        self.current_ay = 0.0
        self.current_az = 0.0

        self.latest_depth = None
        self.filtered_depth = None
        self.last_depth_time = None

        self.depth_scale = 1.0
        self.filtered_depth_scale = 1.0

        self.franka_meas_x = 0.0
        self.franka_meas_y = 0.0
        self.franka_meas_z = 0.0

        self.franka_meas_ax = 0.0
        self.franka_meas_ay = 0.0
        self.franka_meas_az = 0.0

        self.franka_meas_x_f = 0.0
        self.franka_meas_y_f = 0.0
        self.franka_meas_z_f = 0.0

        self.franka_state_des_x = 0.0
        self.franka_state_des_y = 0.0
        self.franka_state_des_z = 0.0

        self.franka_state_des_ax = 0.0
        self.franka_state_des_ay = 0.0
        self.franka_state_des_az = 0.0

        self.have_franka_state = False
        self.last_franka_state_time = None

        self.last_cmd_time = None

        self.last_update_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # ============================================================
        # Subscribers / publishers
        # ============================================================

        self.sub = self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        # ============================================================
        # NEW pre-depth publisher
        # ============================================================

        self.pre_depth_pub = self.create_publisher(
            TwistStamped,
            self.pre_depth_output_topic,
            10
        )

        self.depth_sub = self.create_subscription(
            Float64,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.fruit_grasp_sub = self.create_subscription(
            UInt8,
            self.fruit_grasp_topic,
            self.fruit_grasp_callback,
            10
        )

        self.franka_state_sub = self.create_subscription(
            FrankaRobotState,
            self.franka_robot_state_topic,
            self.franka_state_callback,
            10,
        )

        self.force_pub = self.create_publisher(
            WrenchStamped,
            self.force_feedback_topic,
            10
        )

        timer_period = (
            1.0 / self.publish_rate_hz
        )

        self.timer = self.create_timer(
            timer_period,
            self.update
        )

        # ============================================================
        # Startup logging
        # ============================================================

        self.get_logger().info(
            f"Input topic:  {self.input_topic}"
        )

        self.get_logger().info(
            f"Output topic: {self.output_topic}"
        )

        self.get_logger().info(
            f"Pre-depth filtered output: "
            f"{self.pre_depth_output_topic}"
        )

        self.get_logger().info(
            f"Depth: topic={self.depth_topic}, "
            f"d_safe={self.d_safe}, "
            f"d_stop={self.d_stop}, "
            f"min_scale={self.min_depth_scale}"
        )

        self.get_logger().info(
            f"Fruit grasp topic: {self.fruit_grasp_topic}; "
            f"1 disables scaling, 0 enables scaling"
        )

        self.get_logger().info(
            f"Franka robot_state topic: "
            f"{self.franka_robot_state_topic}"
        )

        self.get_logger().info(
            f"Force feedback: "
            f"output={self.force_feedback_topic}, "
            f"Kprox={self.velocity_error_force_gain}, "
            f"Ktrack={self.tracking_force_gain}, "
            f"max_force={self.max_feedback_force}"
        )

    # ================================================================
    # Command callback
    # ================================================================

    def cmd_callback(
        self,
        msg: TwistStamped
    ) -> None:

        self.target_x = clamp(
            msg.twist.linear.x,
            -self.max_linear_vel_x,
            self.max_linear_vel_x,
        )

        self.target_y = clamp(
            msg.twist.linear.y,
            -self.max_linear_vel_y,
            self.max_linear_vel_y,
        )

        self.target_z = clamp(
            msg.twist.linear.z,
            -self.max_linear_vel_z,
            self.max_linear_vel_z,
        )

        self.target_ax = clamp(
            msg.twist.angular.x,
            -self.max_angular_vel_x,
            self.max_angular_vel_x,
        )

        self.target_ay = clamp(
            msg.twist.angular.y,
            -self.max_angular_vel_y,
            self.max_angular_vel_y,
        )

        self.target_az = clamp(
            msg.twist.angular.z,
            -self.max_angular_vel_z,
            self.max_angular_vel_z,
        )

        self.last_cmd_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

    # ================================================================
    # Depth callback
    # ================================================================

    def depth_callback(
        self,
        msg: Float64
    ) -> None:

        raw_depth = float(
            msg.data
        )

        # Tomato not detected
        if math.isnan(raw_depth):

            self.latest_depth = None
            self.filtered_depth = None
            self.last_depth_time = None

            self.depth_scale = 1.0
            self.filtered_depth_scale = 1.0

            return

        if self.filtered_depth is None:

            self.filtered_depth = (
                raw_depth
            )

        else:

            self.filtered_depth = (
                self.depth_filter_alpha
                * raw_depth
                + (
                    1.0
                    - self.depth_filter_alpha
                )
                * self.filtered_depth
            )

        self.latest_depth = (
            self.filtered_depth
        )

        self.last_depth_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

    # ================================================================
    # Fruit grasp callback
    # ================================================================

    def fruit_grasp_callback(
        self,
        msg: UInt8
    ) -> None:

        self.fruit_grasp_state = (
            1
            if int(msg.data) != 0
            else 0
        )

        new_scaling_enabled = (
            self.fruit_grasp_state == 0
        )

        if (
            new_scaling_enabled
            != self.depth_scaling_enabled
        ):

            self.depth_scaling_enabled = (
                new_scaling_enabled
            )

            self.get_logger().info(
                "fruit_grasp=0 -> depth scaling ENABLED"
                if self.depth_scaling_enabled
                else
                "fruit_grasp=1 -> depth scaling DISABLED"
            )

    # ================================================================
    # Franka state callback
    # ================================================================

    def franka_state_callback(
        self,
        msg: FrankaRobotState
    ) -> None:

        try:
            v_meas = msg.o_dp_ee_c
            v_des = msg.o_dp_ee_d

        except AttributeError:

            self.get_logger().warn(
                "FrankaRobotState does not contain "
                "o_dp_ee_c / o_dp_ee_d fields."
            )

            return

        self.franka_meas_x = float(
            v_meas.twist.linear.x
        )

        self.franka_meas_y = float(
            v_meas.twist.linear.y
        )

        self.franka_meas_z = float(
            v_meas.twist.linear.z
        )

        self.franka_meas_ax = float(
            v_meas.twist.angular.x
        )

        self.franka_meas_ay = float(
            v_meas.twist.angular.y
        )

        self.franka_meas_az = float(
            v_meas.twist.angular.z
        )

        a = clamp(
            self.franka_velocity_filter_alpha,
            0.0,
            1.0
        )

        self.franka_meas_x_f = (
            a * self.franka_meas_x
            + (1.0 - a)
            * self.franka_meas_x_f
        )

        self.franka_meas_y_f = (
            a * self.franka_meas_y
            + (1.0 - a)
            * self.franka_meas_y_f
        )

        self.franka_meas_z_f = (
            a * self.franka_meas_z
            + (1.0 - a)
            * self.franka_meas_z_f
        )

        self.franka_state_des_x = float(
            v_des.twist.linear.x
        )

        self.franka_state_des_y = float(
            v_des.twist.linear.y
        )

        self.franka_state_des_z = float(
            v_des.twist.linear.z
        )

        self.franka_state_des_ax = float(
            v_des.twist.angular.x
        )

        self.franka_state_des_ay = float(
            v_des.twist.angular.y
        )

        self.franka_state_des_az = float(
            v_des.twist.angular.z
        )

        self.have_franka_state = True

        self.last_franka_state_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

    # ================================================================
    # Depth scaling
    # ================================================================

    def compute_depth_scale(
        self,
        now_sec: float
    ) -> float:

        if not self.depth_scaling_enabled:
            return 1.0

        if (
            self.latest_depth is None
            or self.last_depth_time is None
        ):
            return 1.0

        if (
            now_sec
            - self.last_depth_time
        ) > self.depth_timeout_sec:

            return 1.0

        d = self.latest_depth

        if d >= self.d_safe:
            return 1.0

        if d <= self.d_stop:
            return self.min_depth_scale

        s = (
            d - self.d_stop
        ) / max(
            self.d_safe - self.d_stop,
            1e-9
        )

        return clamp(
            smoothstep(s),
            self.min_depth_scale,
            1.0
        )

    # ================================================================
    # Force feedback
    # ================================================================

    def publish_force_feedback(
        self,
        raw_x: float,
        raw_y: float,
        raw_z: float,
        cmd_x: float,
        cmd_y: float,
        cmd_z: float,
    ) -> None:

        # No tomato -> disable force feedback
        if self.latest_depth is None:
            return

        fx = 0.0
        fy = 0.0
        fz = 0.0

        prox_sign = (
            -1.0
            if self.invert_velocity_error_force
            else 1.0
        )

        fx += (
            prox_sign
            * self.velocity_error_force_gain
            * (cmd_x - raw_x)
        )

        fy += (
            prox_sign
            * self.velocity_error_force_gain
            * (cmd_y - raw_y)
        )

        fz += (
            prox_sign
            * self.velocity_error_force_gain
            * (cmd_z - raw_z)
        )

        now_sec = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        franka_valid = (
            self.have_franka_state
            and
            self.last_franka_state_time is not None
            and
            (
                now_sec
                - self.last_franka_state_time
            )
            <= self.franka_state_timeout_sec
        )

        if franka_valid:

            track_sign = (
                -1.0
                if self.invert_tracking_force
                else 1.0
            )

            if (
                abs(cmd_x)
                > self.tracking_velocity_deadband
            ):

                fx += (
                    track_sign
                    * self.tracking_force_gain
                    * (
                        self.franka_meas_x_f
                        - cmd_x
                    )
                )

            if (
                abs(cmd_y)
                > self.tracking_velocity_deadband
            ):

                fy += (
                    track_sign
                    * self.tracking_force_gain
                    * (
                        self.franka_meas_y_f
                        - cmd_y
                    )
                )

            if (
                abs(cmd_z)
                > self.tracking_velocity_deadband
            ):

                fz += (
                    track_sign
                    * self.tracking_force_gain
                    * (
                        self.franka_meas_z_f
                        - cmd_z
                    )
                )

        fx, fy, fz = clamp_norm3(
            fx,
            fy,
            fz,
            self.max_feedback_force
        )

        out = WrenchStamped()

        out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        out.header.frame_id = (
            self.force_feedback_frame_id
        )

        out.wrench.force.x = fx
        out.wrench.force.y = fy
        out.wrench.force.z = fz

        out.wrench.torque.x = 0.0
        out.wrench.torque.y = 0.0
        out.wrench.torque.z = 0.0

        self.force_pub.publish(out)

    # ================================================================
    # Status logging
    # ================================================================

    def log_depth_status(
        self,
        now_sec: float
    ) -> None:

        if (
            now_sec
            - self.last_depth_log_time
        ) < self.depth_log_period:

            return

        self.last_depth_log_time = (
            now_sec
        )

        depth_text = (
            "None"
            if self.latest_depth is None
            else
            f"{self.latest_depth:.4f} m"
        )

        self.get_logger().info(
            f"Depth: {depth_text} | "
            f"alpha={self.depth_scale:.3f} | "
            f"scaling_enabled="
            f"{self.depth_scaling_enabled} | "
            f"fruit_grasp="
            f"{self.fruit_grasp_state} | "
            f"v_meas=("
            f"{self.franka_meas_x_f:.3f}, "
            f"{self.franka_meas_y_f:.3f}, "
            f"{self.franka_meas_z_f:.3f})"
        )

    # ================================================================
    # Main update loop
    # ================================================================

    def update(self) -> None:

        now_sec = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        dt = (
            now_sec
            - self.last_update_time
        )

        self.last_update_time = (
            now_sec
        )

        if dt <= 0.0:
            return

        # ============================================================
        # Original target logic
        # ============================================================

        if (
            self.last_cmd_time is None
            or
            (
                now_sec
                - self.last_cmd_time
            )
            > self.command_timeout_sec
        ):

            tx = ty = tz = 0.0
            tax = tay = taz = 0.0

        else:

            tx = self.target_x
            ty = self.target_y
            tz = self.target_z

            tax = self.target_ax
            tay = self.target_ay
            taz = self.target_az

        # ============================================================
        # Original raw command
        # ============================================================

        raw_tx = tx
        raw_ty = ty
        raw_tz = tz

        # ============================================================
        # NEW:
        # Parallel diagnostic acceleration-limited velocity
        # BEFORE depth scaling.
        #
        # This does not modify tx, ty, tz or current_x/y/z.
        # ============================================================

        self.pre_depth_x = clamp_delta(
            self.pre_depth_x,
            raw_tx,
            self.linear_accel_limit_x * dt,
        )

        self.pre_depth_y = clamp_delta(
            self.pre_depth_y,
            raw_ty,
            self.linear_accel_limit_y * dt,
        )

        self.pre_depth_z = clamp_delta(
            self.pre_depth_z,
            raw_tz,
            self.linear_accel_limit_z * dt,
        )

        # ============================================================
        # NEW:
        # Publish diagnostic pre-depth velocity
        # ============================================================

        pre_depth_out = TwistStamped()

        pre_depth_out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pre_depth_out.header.frame_id = (
            self.frame_id
        )

        pre_depth_out.twist.linear.x = (
            self.pre_depth_x
        )

        pre_depth_out.twist.linear.y = (
            self.pre_depth_y
        )

        pre_depth_out.twist.linear.z = (
            self.pre_depth_z
        )

        # Angular values are copied from the raw target.
        # Depth scaling in your existing code only affects
        # linear velocity.
        pre_depth_out.twist.angular.x = tax
        pre_depth_out.twist.angular.y = tay
        pre_depth_out.twist.angular.z = taz

        self.pre_depth_pub.publish(
            pre_depth_out
        )

        # ============================================================
        # EVERYTHING BELOW IS YOUR ORIGINAL CONTROL PATH
        # ============================================================

        alpha_target = (
            self.compute_depth_scale(
                now_sec
            )
        )

        max_alpha_step = (
            self.alpha_rate_limit
            * dt
        )

        self.filtered_depth_scale = (
            clamp_delta(
                self.filtered_depth_scale,
                alpha_target,
                max_alpha_step,
            )
        )

        self.depth_scale = (
            self.filtered_depth_scale
        )

        # ============================================================
        # Original depth scaling
        # ============================================================

        tx *= self.depth_scale
        ty *= self.depth_scale
        tz *= self.depth_scale

        # ============================================================
        # Original linear acceleration limiting
        # ============================================================

        self.current_x = clamp_delta(
            self.current_x,
            tx,
            self.linear_accel_limit_x * dt,
        )

        self.current_y = clamp_delta(
            self.current_y,
            ty,
            self.linear_accel_limit_y * dt,
        )

        self.current_z = clamp_delta(
            self.current_z,
            tz,
            self.linear_accel_limit_z * dt,
        )

        # ============================================================
        # Original angular acceleration limiting
        # ============================================================

        self.current_ax = clamp_delta(
            self.current_ax,
            tax,
            self.angular_accel_limit_x * dt,
        )

        self.current_ay = clamp_delta(
            self.current_ay,
            tay,
            self.angular_accel_limit_y * dt,
        )

        self.current_az = clamp_delta(
            self.current_az,
            taz,
            self.angular_accel_limit_z * dt,
        )

        # ============================================================
        # Original output
        # ============================================================

        out = TwistStamped()

        out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        out.header.frame_id = (
            self.frame_id
        )

        out.twist.linear.x = (
            self.current_x
        )

        out.twist.linear.y = (
            self.current_y
        )

        out.twist.linear.z = (
            self.current_z
        )

        out.twist.angular.x = (
            self.current_ax
        )

        out.twist.angular.y = (
            self.current_ay
        )

        out.twist.angular.z = (
            self.current_az
        )

        self.pub.publish(out)

        # ============================================================
        # Original force feedback
        # ============================================================

        self.publish_force_feedback(
            raw_tx,
            raw_ty,
            raw_tz,
            self.current_x,
            self.current_y,
            self.current_z,
        )

        # ============================================================
        # Original status logging
        # ============================================================

        self.log_depth_status(
            now_sec
        )


def main() -> None:

    rclpy.init()

    node = SmoothedCmdVel()

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
