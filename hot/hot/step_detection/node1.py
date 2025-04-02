#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import numpy as np
import cv2
from cv_bridge import CvBridge


class ContourStepDetector(Node):
    def __init__(self):
        super().__init__('contour_step_detector')

        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.alert_pub = self.create_publisher(Bool, '/contour_step_detected', 10)

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            step_detected = self.detect_step_by_contour(frame)

            # 결과 publish
            self.alert_pub.publish(Bool(data=step_detected))
        except Exception as e:
            self.get_logger().warn(f"[RGB 처리 오류] {e}")

    def detect_step_by_contour(self, frame):
        height, width, _ = frame.shape

        # ROI 확대: 하단 40% → 전체 또는 하단 70%
        roi_start_y = int(height * 0.3)
        roi = frame[roi_start_y:, :]  # 아래 70%만 사용

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # 윤곽선 검출
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 전체 윤곽선 시각화
        contour_vis = roi.copy()
        cv2.drawContours(contour_vis, contours, -1, (0, 255, 255), 1)  # 노란색 윤곽선

        step_detected = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 800:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # 조건 만족 시 사각형 강조 표시
            if w > 30 and h > 30:
                step_detected = True
                cv2.rectangle(contour_vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(contour_vis, "STEP", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 화면 표시
        cv2.imshow("Contour Step Detection (Expanded View)", contour_vis)
        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC 눌러 종료")
            rclpy.shutdown()

        return step_detected


def main(args=None):
    rclpy.init(args=args)
    node = ContourStepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
