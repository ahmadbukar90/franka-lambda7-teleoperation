#!/usr/bin/env python3

import math
import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, Bool, String
from cv_bridge import CvBridge


WIDTH, HEIGHT, FPS = 640, 480, 30
OP_WIDTH, OP_HEIGHT = 424, 240

MIN_DEPTH_M, MAX_DEPTH_M = 0.07, 1.00
MIN_FRUIT_AREA = 1500

MIN_CIRCULARITY = 0.55
MIN_SOLIDITY = 0.85
ASPECT_RANGE = (0.55, 1.80)
MAX_SKIN_FRACTION = 0.55

COL_HAPTIC_ENABLE = (0, 255, 0)    # Green
COL_HAPTIC_DISABLE = (255, 0, 0)   # Blue

HAPTIC_MIN_CM = 7.0
HAPTIC_MAX_CM = 15.0


def color_mask(hsv, sat_floor):
    m1 = cv2.inRange(hsv, np.array([0, sat_floor, 60]), np.array([15, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([160, sat_floor, 60]), np.array([180, 255, 255]))
    m3 = cv2.inRange(hsv, np.array([5, sat_floor, 60]), np.array([25, 255, 255]))

    mask = cv2.bitwise_or(cv2.bitwise_or(m1, m2), m3)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def skin_mask_ycrcb(bgr):
    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    skin = cv2.inRange(ycrcb, np.array([0, 133, 77]), np.array([255, 173, 127]))
    return skin > 0


def shape_metrics(comp_mask):
    m = comp_mask.astype(np.uint8) * 255
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not cnts:
        return 0.0, 0.0, 99.0

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    peri = cv2.arcLength(c, True)

    if peri <= 0 or area <= 0:
        return 0.0, 0.0, 99.0

    circularity = 4.0 * np.pi * area / (peri * peri)

    hull = cv2.convexHull(c)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 0.0

    x, y, w, h = cv2.boundingRect(c)
    aspect = w / h if h > 0 else 99.0

    return float(circularity), float(solidity), float(aspect)


def segment_tomatoes(bgr, sat_floor, use_shape=True, use_skin=True):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, sat_floor)
    skin = skin_mask_ycrcb(bgr) if use_skin else None

    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    accepted = []

    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < MIN_FRUIT_AREA:
            continue

        comp = labels == i
        ys, xs = np.where(comp)

        item = {
            "mask": comp,
            "bbox": (
                int(xs.min()),
                int(ys.min()),
                int(xs.max()) + 1,
                int(ys.max()) + 1,
            ),
            "cx": int(cents[i][0]),
            "cy": int(cents[i][1]),
        }

        reject = False

        if use_shape:
            circ, sol, asp = shape_metrics(comp)

            if circ < MIN_CIRCULARITY:
                reject = True
            elif sol < MIN_SOLIDITY:
                reject = True
            elif not (ASPECT_RANGE[0] <= asp <= ASPECT_RANGE[1]):
                reject = True

        if not reject and use_skin:
            skin_frac = float(skin[comp].mean())

            if skin_frac > MAX_SKIN_FRACTION:
                reject = True

        if not reject:
            accepted.append(item)

    accepted.sort(key=lambda f: f["cx"])
    return accepted


def fruit_distance(depth_m, fruit_mask):
    ys, xs = np.where(fruit_mask)

    if xs.size < 20:
        return float("nan")

    cx = int(np.mean(xs))
    cy = int(np.mean(ys))

    x1 = max(cx - 20, 0)
    x2 = min(cx + 21, depth_m.shape[1])
    y1 = max(cy - 20, 0)
    y2 = min(cy + 21, depth_m.shape[0])

    center_mask = fruit_mask[y1:y2, x1:x2]
    center_depth = depth_m[y1:y2, x1:x2]

    d = center_depth[center_mask]
    valid = d[(d >= MIN_DEPTH_M) & (d <= MAX_DEPTH_M)]

    if valid.size < 10:
        d = depth_m[fruit_mask]
        valid = d[(d >= MIN_DEPTH_M) & (d <= MAX_DEPTH_M)]

    if valid.size < 20:
        return float("nan")

    return float(np.percentile(valid, 35))


class D405TomatoNode(Node):
    def __init__(self):
        super().__init__("d405_tomato_node")

        self.bridge = CvBridge()

        self.operator_rgb_pub = self.create_publisher(
            Image,
            "/d405/operator_rgb_image",
            qos_profile_sensor_data,
        )

        self.cv_pub = self.create_publisher(Image, "/d405/cv_image", 10)
        self.depth_pub = self.create_publisher(Image, "/d405/depth_image", 10)

        self.target_depth_pub = self.create_publisher(
            Float32,
            "/d405/target_depth_cm",
            10,
        )

        self.rgb_depth_pub = self.create_publisher(
            Float64,
            "/rgb_depth",
            10,
        )

        self.detected_pub = self.create_publisher(
            Bool,
            "/d405/tomato_detected",
            10,
        )

        self.ripeness_pub = self.create_publisher(
            String,
            "/d405/ripeness_label",
            10,
        )

        self.ripeness_score_pub = self.create_publisher(
            Float32,
            "/d405/ripeness_score",
            10,
        )

        self.sat_floor = 80
        self.use_shape = True
        self.use_skin = True

        self.filtered_rgb_depth = None
        self.last_valid_rgb_depth = None
        self.last_valid_depth_time = None

        self.depth_filter_alpha = 0.02
        self.rgb_depth_publish_dt = 0.05
        self.depth_latch_timeout = 1.0
        self.last_rgb_depth_publish_time = 0.0

        self.locked_target_cx = None
        self.locked_target_cy = None
        self.target_switch_margin_m = 0.04
        self.target_match_px = 120

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        self.config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

        self.get_logger().info("Starting RealSense D405 pipeline...")

        self.profile = self.pipeline.start(self.config)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        self.timer = self.create_timer(1.0 / FPS, self.timer_callback)

        self.get_logger().info("D405 tomato node started.")
        self.get_logger().info("Display: haptic enabled/disabled only.")

    def publish_conditioned_rgb_depth(self, target_depth_cm):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if (now_sec - self.last_rgb_depth_publish_time) < self.rgb_depth_publish_dt:
            return

        self.last_rgb_depth_publish_time = now_sec
        msg = Float64()

        if math.isfinite(target_depth_cm):
            raw_depth_m = target_depth_cm / 100.0

            if self.filtered_rgb_depth is None:
                self.filtered_rgb_depth = raw_depth_m
            else:
                self.filtered_rgb_depth = (
                    self.depth_filter_alpha * raw_depth_m
                    + (1.0 - self.depth_filter_alpha) * self.filtered_rgb_depth
                )

            self.last_valid_rgb_depth = self.filtered_rgb_depth
            self.last_valid_depth_time = now_sec
            msg.data = float(self.filtered_rgb_depth)

        else:
            if (
                self.last_valid_rgb_depth is not None
                and self.last_valid_depth_time is not None
                and (now_sec - self.last_valid_depth_time) <= self.depth_latch_timeout
            ):
                msg.data = float(self.last_valid_rgb_depth)
            else:
                self.filtered_rgb_depth = None
                self.last_valid_rgb_depth = None
                self.last_valid_depth_time = None
                msg.data = float("nan")

        self.rgb_depth_pub.publish(msg)

    def select_closest_locked_target(self, fruits, distances):
        valid_indices = [
            i for i, d in enumerate(distances)
            if math.isfinite(d)
        ]

        if not valid_indices:
            self.locked_target_cx = None
            self.locked_target_cy = None
            return None

        closest_index = min(valid_indices, key=lambda i: distances[i])
        target_index = closest_index

        if self.locked_target_cx is not None and self.locked_target_cy is not None:
            matched_index = None
            best_pixel_dist = float("inf")

            for i in valid_indices:
                dx = fruits[i]["cx"] - self.locked_target_cx
                dy = fruits[i]["cy"] - self.locked_target_cy
                pixel_dist = math.sqrt(dx * dx + dy * dy)

                if pixel_dist < best_pixel_dist:
                    best_pixel_dist = pixel_dist
                    matched_index = i

            if matched_index is not None and best_pixel_dist < self.target_match_px:
                current_d = distances[matched_index]
                closest_d = distances[closest_index]

                if closest_d < current_d - self.target_switch_margin_m:
                    target_index = closest_index
                else:
                    target_index = matched_index

        target = fruits[target_index]
        self.locked_target_cx = target["cx"]
        self.locked_target_cy = target["cy"]

        return target_index

    def timer_callback(self):
        frames = self.align.process(self.pipeline.wait_for_frames())

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        bgr = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale

        now = self.get_clock().now().to_msg()

        operator_bgr = cv2.resize(
            bgr,
            (OP_WIDTH, OP_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )

        operator_msg = self.bridge.cv2_to_imgmsg(operator_bgr, encoding="bgr8")
        operator_msg.header.stamp = now
        operator_msg.header.frame_id = "d405_operator_frame"
        self.operator_rgb_pub.publish(operator_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "d405_color_frame"
        self.depth_pub.publish(depth_msg)

        cv_image = bgr.copy()

        fruits = segment_tomatoes(
            bgr,
            self.sat_floor,
            self.use_shape,
            self.use_skin,
        )

        target_depth_cm = float("nan")
        target_label = "NONE"
        target_score = 0.0
        detected = False

        if fruits:
            distances = [fruit_distance(depth_m, f["mask"]) for f in fruits]
            target_index = self.select_closest_locked_target(fruits, distances)

            if target_index is not None:
                target = fruits[target_index]
                dist_m = distances[target_index]

                target_depth_cm = dist_m * 100.0
                detected = True

                haptic_enabled = HAPTIC_MIN_CM <= target_depth_cm <= HAPTIC_MAX_CM

                if haptic_enabled:
                    target_label = "HAPTIC ENABLED"
                    haptic_text = "HAPTIC ENABLED"
                    haptic_color = COL_HAPTIC_ENABLE
                else:
                    target_label = "HAPTIC DISABLED"
                    haptic_text = "HAPTIC DISABLED"
                    haptic_color = COL_HAPTIC_DISABLE

                target_score = 1.0 if haptic_enabled else 0.0

                for i, f in enumerate(fruits):
                    x1, y1, x2, y2 = f["bbox"]
                    is_target = i == target_index

                    if is_target:
                        box_thickness = 4
                        text_thickness = 3
                    else:
                        box_thickness = 1
                        text_thickness = 1

                    if is_target and haptic_enabled:
                        color = COL_HAPTIC_ENABLE
                    else:
                        color = COL_HAPTIC_DISABLE

                    cv2.rectangle(
                        cv_image,
                        (x1, y1),
                        (x2, y2),
                        color,
                        box_thickness,
                    )

                    dist_i = distances[i]
                    dist_text = (
                        f"{dist_i * 100.0:.1f} cm"
                        if math.isfinite(dist_i)
                        else "NaN"
                    )

                    cv2.putText(
                        cv_image,
                        f"#{i + 1} {dist_text}",
                        (x1, max(y1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        text_thickness,
                        cv2.LINE_AA,
                    )

                cv2.drawMarker(
                    cv_image,
                    (target["cx"], target["cy"]),
                    haptic_color,
                    cv2.MARKER_CROSS,
                    18,
                    2,
                )

                cv2.putText(
                    cv_image,
                    haptic_text,
                    (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.1,
                    haptic_color,
                    5,
                    cv2.LINE_AA,
                )

        else:
            self.locked_target_cx = None
            self.locked_target_cy = None

        self.publish_conditioned_rgb_depth(target_depth_cm)

        cv_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        cv_msg.header.stamp = now
        cv_msg.header.frame_id = "d405_color_frame"
        self.cv_pub.publish(cv_msg)

        self.target_depth_pub.publish(Float32(data=float(target_depth_cm)))
        self.detected_pub.publish(Bool(data=detected))
        self.ripeness_pub.publish(String(data=target_label))
        self.ripeness_score_pub.publish(Float32(data=float(target_score)))

    def destroy_node(self):
        self.get_logger().info("Stopping D405 pipeline...")
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D405TomatoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
