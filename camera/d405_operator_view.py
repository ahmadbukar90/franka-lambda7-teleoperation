#!/usr/bin/env python3

import os
import cv2
import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class D405OperatorView(Node):
    def __init__(self):
        super().__init__("d405_operator_view")

        self.bridge = CvBridge()
        self.window_name = "D405 Operator View"
        self.fullscreen = False

        self.screen_width = 1920
        self.screen_height = 1080

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.save_dir = os.path.expanduser(
            f"~/ros2_ws/data/operator_captures/{timestamp}"
        )
        os.makedirs(self.save_dir, exist_ok=True)

        self.save_count = 0
        self.latest_frame = None
        self.recording = False
        self.frame_counter = 0
        self.frame_save_interval = 1

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1000, 600)
        cv2.moveWindow(self.window_name, 100, 100)

        self.sub = self.create_subscription(
            Image,
            "/d405/operator_rgb_image",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Operator view started.")
        self.get_logger().info(f"Saving folder: {self.save_dir}")
        self.get_logger().info("f = fullscreen | s = save one frame | r = record all frames | q/ESC = quit")

    def save_frame(self, frame):
        self.save_count += 1
        filename = os.path.join(
            self.save_dir,
            f"operator_frame_{self.save_count:06d}.png"
        )
        cv2.imwrite(filename, frame)
        return filename

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_frame = frame.copy()

        self.frame_counter += 1

        if self.recording and self.frame_counter % self.frame_save_interval == 0:
            self.save_frame(self.latest_frame)

        display_frame = frame

        if self.fullscreen:
            display_frame = cv2.resize(
                frame,
                (self.screen_width, self.screen_height),
                interpolation=cv2.INTER_LINEAR,
            )

        cv2.imshow(self.window_name, display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            rclpy.shutdown()

        elif key == ord("f"):
            self.fullscreen = not self.fullscreen
            if self.fullscreen:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
                self.get_logger().info("Fullscreen ON")
            else:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_NORMAL,
                )
                cv2.resizeWindow(self.window_name, 1000, 600)
                self.get_logger().info("Fullscreen OFF")

        elif key == ord("s"):
            if self.latest_frame is not None:
                filename = self.save_frame(self.latest_frame)
                self.get_logger().info(f"Saved one frame: {filename}")

        elif key == ord("r"):
            self.recording = not self.recording
            self.get_logger().info(
                "Continuous saving ON" if self.recording else "Continuous saving OFF"
            )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D405OperatorView()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
