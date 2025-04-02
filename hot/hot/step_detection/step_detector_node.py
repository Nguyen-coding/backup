#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool

import numpy as np
import cv2
from cv_bridge import CvBridge


class StepDetector(Node):
    def __init__(self):
        super().__init__('step_detector')

        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.alert_pub = self.create_publisher(Bool, '/step_detected', 10)

        self.latest_depth = None
        self.latest_rgb = None
        self.latest_scan = None

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.check_step()
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB conversion failed: {e}")

    def lidar_callback(self, msg):
        self.latest_scan = msg
        self.check_step()

    def check_step(self):
        if self.latest_depth is None or self.latest_scan is None or self.latest_rgb is None:
            return

        depth_image = self.latest_depth.copy()
        rgb_image = self.latest_rgb.copy()

        # ROI 설정 (중앙 하단)
        x_start, x_end = 280, 360
        y_start, y_end = 200, 300
        roi = depth_image[y_start:y_end, x_start:x_end]
        roi = roi[np.isfinite(roi)]

        if roi.size == 0:
            return

        avg_depth = np.mean(roi)
        self.get_logger().info(f"Avg depth: {avg_depth:.2f}")

        # Lidar 데이터
        scan = np.array(self.latest_scan.ranges)
        front_ranges = scan[0:10].tolist() + scan[-10:].tolist()
        front_ranges = [r for r in front_ranges if not np.isnan(r) and r < 10.0]
        avg_scan = np.mean(front_ranges) if front_ranges else 0

        # 단차 조건
        step_detected = avg_depth > 1.0 and avg_scan > 1.5
        self.alert_pub.publish(Bool(data=bool(step_detected)))

        if step_detected:
            self.get_logger().info("⚠️ 단차 감지됨!")

        # RGB 이미지 위에 시각화
        color = (0, 0, 255) if step_detected else (0, 255, 0)
        cv2.rectangle(rgb_image, (x_start, y_start), (x_end, y_end), color, 2)
        cv2.putText(rgb_image, f"Step: {'YES' if step_detected else 'NO'}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        cv2.imshow("RGB with Step Detection", rgb_image)
        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC 눌러 종료")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
