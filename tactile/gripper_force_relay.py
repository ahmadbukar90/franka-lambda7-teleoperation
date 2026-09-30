#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperForceRelay(Node):
    def __init__(self):
        super().__init__("gripper_force_relay")

        self.latest_force = 0.0

        self.sub = self.create_subscription(
            Float64,
            "/gelsight/desired_grip_force",
            self.force_callback,
            10,
        )

        self.pub = self.create_publisher(
            Float64,
            "/cmd/force_feedback/gripper_force",
            10,
        )

        self.timer = self.create_timer(1.0 / 100.0, self.publish_callback)

        self.get_logger().info(
            "Relaying /gelsight/desired_grip_force to /cmd/force_feedback/gripper_force at 100 Hz"
        )

    def force_callback(self, msg):
        self.latest_force = float(msg.data)

    def publish_callback(self):
        self.pub.publish(Float64(data=self.latest_force))

    def destroy_node(self):
        try:
            self.pub.publish(Float64(data=0.0))
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperForceRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
