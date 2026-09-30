#!/usr/bin/env python3

import cv2
import time
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, Bool, UInt8

from .utilities.reconstruction import Reconstruction3D
from .config import GSConfig
from .utilities.gelsightmini import GelSightMini


LEFT_CAMERA_PATH = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._GelSight_Mini_R0B_2DUN-7MA1_2DUN7MA1-video-index0"

RIGHT_CAMERA_PATH = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._GelSight_Mini_R0B_2G6P-P4LP_2G6PP4LP-video-index0"

BUFFER_SIZE = 10
DEPTH_PIXEL_THRESHOLD = 0.5

CONTACT_ON_THRESHOLD = 0.5
CONTACT_OFF_THRESHOLD = 0.15

BASELINE_FRAMES = 30
WARMUP_FRAMES = 30

FORCE_GAIN = 1
MAX_GRIP_FORCE = 5.0
MIN_CONTACT_FORCE = 0.02

CONTACT_HOLD_TIME_SEC = 2.0

SENSING_RATE_HZ = 30.0


class GelSightDepthPublisher(Node):
    def __init__(self):
        super().__init__("gelsight_depth_publisher")

        self.left_buffer = []
        self.right_buffer = []

        self.baseline_left_depth = None
        self.baseline_right_depth = None

        self.left_contact_latched = False
        self.right_contact_latched = False

        self.desired_grip_force = 0.0

        self.contact_start_time = None
        self.contact_ready_state = 0

        self.left_pub = self.create_publisher(Float32, "/gelsight/left_depth", 10)
        self.right_pub = self.create_publisher(Float32, "/gelsight/right_depth", 10)
        self.contact_pub = self.create_publisher(Bool, "/gelsight/contact_state", 10)

        # This is the delayed contact state:
        # 0 = contact not held for 8 seconds yet
        # 1 = contact held continuously for 8 seconds
        self.contact_ready_pub = self.create_publisher(
            UInt8,
            "/fruit_grasp",
            10,
        )

        self.desired_force_pub = self.create_publisher(
            Float64,
            "/gelsight/desired_grip_force",
            10,
        )

        self.get_logger().info("Initializing GelSight cameras...")

        self.cam_left = GelSightMini(target_width=320, target_height=240)
        self.cam_left.select_device(LEFT_CAMERA_PATH)
        self.cam_left.start()

        self.cam_right = GelSightMini(target_width=320, target_height=240)
        self.cam_right.select_device(RIGHT_CAMERA_PATH)
        self.cam_right.start()

        time.sleep(2)

        self.get_logger().info("Loading GelSight depth model...")

        package_dir = Path(__file__).resolve().parent
        self.gs_config = GSConfig(str(package_dir / "default_config.json"))

        self.reconstruction = Reconstruction3D(
            image_width=self.gs_config.config.camera_width,
            image_height=self.gs_config.config.camera_height,
            use_gpu=self.gs_config.config.use_gpu,
        )

        model_path = Path(self.gs_config.config.nn_model_path)
        if not model_path.is_absolute():
            model_path = package_dir / model_path

        if self.reconstruction.load_nn(str(model_path)) is None:
            raise RuntimeError("Failed to load depth model")

        self.warmup()
        self.capture_baseline()

        self.get_logger().info("GelSight ROS 2 publisher started.")
        self.get_logger().info(
            "Publishing:\n"
            "  /gelsight/left_depth\n"
            "  /gelsight/right_depth\n"
            "  /gelsight/contact_state\n"
            "  /gelsight/contact_ready_after_8s\n"
            "  /gelsight/desired_grip_force"
        )

        self.sensing_timer = self.create_timer(
            1.0 / SENSING_RATE_HZ,
            self.sensing_callback,
        )

    def resize_crop_mini(self, img, imgw=320, imgh=240):
        border_x = int(img.shape[0] * (1 / 7))
        border_y = int(img.shape[1] * (1 / 7))

        img = img[
            border_x:img.shape[0] - border_x,
            border_y:img.shape[1] - border_y,
        ]

        return cv2.resize(img, (imgw, imgh))

    def get_depth_map(self, frame):
        frame = self.resize_crop_mini(frame)

        depth_map, _, _, _ = self.reconstruction.get_depthmap(
            image=cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB),
            markers_threshold=(
                self.gs_config.config.marker_mask_min,
                self.gs_config.config.marker_mask_max,
            ),
        )

        return frame, depth_map

    def compute_stable_depth(self, depth_map, baseline_depth, buffer, sensor_side):
        if sensor_side == "left":
            difference = depth_map - baseline_depth
        elif sensor_side == "right":
            difference = baseline_depth - depth_map
        else:
            raise ValueError("sensor_side must be left or right")

        difference[difference < 0] = 0

        contact_region = difference[difference > DEPTH_PIXEL_THRESHOLD]

        if len(contact_region) > 0:
            median_depth = float(np.median(contact_region))
        else:
            median_depth = 0.0

        buffer.append(median_depth)

        if len(buffer) > BUFFER_SIZE:
            buffer.pop(0)

        stable_depth = float(np.median(buffer))

        if sensor_side == "left":
            previous_contact = self.left_contact_latched
        else:
            previous_contact = self.right_contact_latched

        if previous_contact:
            contact = stable_depth > CONTACT_OFF_THRESHOLD
        else:
            contact = stable_depth > CONTACT_ON_THRESHOLD

        if sensor_side == "left":
            self.left_contact_latched = contact
        else:
            self.right_contact_latched = contact

        return stable_depth, contact

    def update_desired_force(self, average_depth, contact_state):
        if contact_state:
            force = average_depth * FORCE_GAIN
            force = max(MIN_CONTACT_FORCE, min(force, MAX_GRIP_FORCE))
            self.desired_grip_force = force
        else:
            self.desired_grip_force = 0.0

    def update_contact_ready_state(self, contact_state):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if contact_state:
            if self.contact_start_time is None:
                self.contact_start_time = now_sec
                self.contact_ready_state = 0

            elapsed = now_sec - self.contact_start_time

            if elapsed >= CONTACT_HOLD_TIME_SEC:
                self.contact_ready_state = 1
            else:
                self.contact_ready_state = 0

            return elapsed

        self.contact_start_time = None
        self.contact_ready_state = 0
        return 0.0

    def warmup(self):
        self.get_logger().info("Warming up cameras/model...")

        for _ in range(WARMUP_FRAMES):
            left_frame = self.cam_left.update(dt=0)
            right_frame = self.cam_right.update(dt=0)

            if left_frame is None or right_frame is None:
                raise RuntimeError("Camera error during warmup")

            self.get_depth_map(left_frame)
            self.get_depth_map(right_frame)

        self.get_logger().info("Warmup complete.")

    def capture_baseline(self):
        self.get_logger().info("Capturing automatic baseline. Do not touch sensors...")

        left_baselines = []
        right_baselines = []

        for _ in range(BASELINE_FRAMES):
            left_frame = self.cam_left.update(dt=0)
            right_frame = self.cam_right.update(dt=0)

            if left_frame is None or right_frame is None:
                raise RuntimeError("Camera error during baseline")

            _, left_depth_map = self.get_depth_map(left_frame)
            _, right_depth_map = self.get_depth_map(right_frame)

            left_baselines.append(left_depth_map.copy())
            right_baselines.append(right_depth_map.copy())

        self.baseline_left_depth = np.median(np.stack(left_baselines, axis=0), axis=0)
        self.baseline_right_depth = np.median(np.stack(right_baselines, axis=0), axis=0)

        self.left_buffer.clear()
        self.right_buffer.clear()

        self.left_contact_latched = False
        self.right_contact_latched = False

        self.desired_grip_force = 0.0
        self.contact_start_time = None
        self.contact_ready_state = 0

        self.get_logger().info("Baseline captured.")

    def sensing_callback(self):
        left_frame = self.cam_left.update(dt=0)
        right_frame = self.cam_right.update(dt=0)

        if left_frame is None or right_frame is None:
            self.get_logger().error("Camera error")
            return

        left_display, left_depth_map = self.get_depth_map(left_frame)
        right_display, right_depth_map = self.get_depth_map(right_frame)

        left_depth, left_contact = self.compute_stable_depth(
            depth_map=left_depth_map,
            baseline_depth=self.baseline_left_depth,
            buffer=self.left_buffer,
            sensor_side="left",
        )

        right_depth, right_contact = self.compute_stable_depth(
            depth_map=right_depth_map,
            baseline_depth=self.baseline_right_depth,
            buffer=self.right_buffer,
            sensor_side="right",
        )

        average_depth = float((left_depth + right_depth) / 2.0)
        contact_state = bool(left_contact or right_contact)

        self.update_contact_ready_state(contact_state)
        self.update_desired_force(average_depth, contact_state)

        self.left_pub.publish(Float32(data=left_depth))
        self.right_pub.publish(Float32(data=right_depth))
        self.contact_pub.publish(Bool(data=contact_state))
        self.contact_ready_pub.publish(UInt8(data=self.contact_ready_state))
        self.desired_force_pub.publish(Float64(data=float(self.desired_grip_force)))

        left_status = "Haptic Enabled" if left_contact else "Haptic Disabled"
        right_status = "Haptic Enabled" if right_contact else "Haptic Disabled"

        # OpenCV uses BGR color order:
        # Green = (0, 255, 0), Blue = (255, 0, 0)
        left_color = (0, 255, 0) if left_contact else (255, 0, 0)
        right_color = (0, 255, 0) if right_contact else (255, 0, 0)

        cv2.putText(
            left_display,
            f"L: {left_depth:.2f}  {left_status}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            left_color,
            3,  # thicker text gives a bold appearance
            cv2.LINE_AA,
        )

        cv2.putText(
            right_display,
            f"R: {right_depth:.2f}  {right_status}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            right_color,
            3,  # thicker text gives a bold appearance
            cv2.LINE_AA,
        )

        cv2.imshow("Left GelSight", left_display)
        cv2.imshow("Right GelSight", right_display)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            self.desired_force_pub.publish(Float64(data=0.0))
            self.contact_ready_pub.publish(UInt8(data=0))
            self.cam_left.camera.release()
            self.cam_right.camera.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GelSightDepthPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
