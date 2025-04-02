#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import numpy as np
import cv2
from cv_bridge import CvBridge


class DepthStepDetector(Node):
    def __init__(self):
        super().__init__('depth_step_detector')

        self.bridge = CvBridge()

        # Depth + RGB 이미지 구독
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

        self.alert_pub = self.create_publisher(Bool, '/depth_step_detected', 10)

        self.latest_depth = None
        self.latest_rgb = None

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"Depth 변환 실패: {e}")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.check_step()
        except Exception as e:
            self.get_logger().warn(f"RGB 변환 실패: {e}")

    def check_step(self):
        if self.latest_depth is None or self.latest_rgb is None:
            return

        depth = np.nan_to_num(self.latest_depth, nan=0.0, posinf=0.0, neginf=0.0)
        rgb_image = self.latest_rgb.copy()

        h, w = depth.shape
        x_start, x_end = int(w * 0.4), int(w * 0.6)
        y_start, y_end = int(h * 0.6), int(h * 0.9)
        
        self.get_logger().info(f"ROI size: {roi.shape}, valid pixels: {valid_roi.size}")

        roi = depth[y_start:y_end, x_start:x_end]
        valid_roi = roi[(roi > 0.1) & (roi < 4.0)]

        if valid_roi.size == 0:
            self.get_logger().warn("⚠️ 유효한 depth ROI 없음")
            step_detected = False
        else:
            depth_min = np.min(valid_roi)
            depth_max = np.max(valid_roi)
            depth_diff = depth_max - depth_min

            self.get_logger().info(f"Depth range: {depth_min:.2f} ~ {depth_max:.2f} m")
            step_detected = depth_diff > 0.05 # 얼마나 차이나면 감지하는지 5cm

        self.alert_pub.publish(Bool(data=step_detected))

        # ✅ RGB 영상 위에 시각화
        color = (0, 0, 255) if step_detected else (0, 255, 0)
        cv2.rectangle(rgb_image, (x_start, y_start), (x_end, y_end), color, 2)

        text = "STEP DETECTED" if step_detected else "No Step"
        cv2.putText(rgb_image, text, (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2)

        cv2.imshow("RGB View + Depth-based Step Detection", rgb_image)
        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC 눌러 종료")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DepthStepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
