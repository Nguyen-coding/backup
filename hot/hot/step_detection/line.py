"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Line Detector Node Started.")

    def image_callback(self, msg):
        # ROS 이미지 → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 그레이 변환 + 블러 + 엣지
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 2)
        edge = cv2.Canny(gray, 50, 150, 3)

        # Hough 선 검출
        lines = cv2.HoughLinesP(edge, 1, np.pi / 180, 200, minLineLength=50, maxLineGap=10)

        if lines is not None:
            min_angle = -1.0 * (np.pi / 180.0)
            max_angle = 1.0 * (np.pi / 180.0)

            lowest_line_y = 0
            horizontal_lines = []

            for l in lines:
                x1, y1, x2, y2 = l[0]
                angle = np.arctan2(y2 - y1, x2 - x1)

                if min_angle < angle < max_angle:
                    horizontal_lines.append((x1, y1, x2, y2))
                    lowest_line_y = max(lowest_line_y, y1, y2)

            merged_lines = []
            height_threshold = 50

            for l1 in horizontal_lines:
                merged = False
                for i, l2 in enumerate(merged_lines):
                    if abs(l1[1] - l2[1]) < height_threshold and abs(l1[3] - l2[3]) < height_threshold:
                        l2[0] = min(l2[0], l1[0])
                        l2[1] = (l2[1] + l1[1]) // 2
                        l2[2] = max(l2[2], l1[2])
                        l2[3] = (l2[3] + l1[3]) // 2
                        merged = True
                        break
                if not merged:
                    merged_lines.append(list(l1))

            for x1, y1, x2, y2 in horizontal_lines:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)  # 빨간색

            for x1, y1, x2, y2 in merged_lines:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)  # 파란색

            if lowest_line_y > 0:
                self.get_logger().info(f"가장 낮은 수평선 Y 좌표: {lowest_line_y}")

        # 디버깅용 시각화 (OpenCV 창 띄우기)
        cv2.imshow("Line Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        
        # Message Filters를 사용해 컬러와 뎁스 이미지 동기화
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)
        
        self.get_logger().info("Line Detector Node with Depth Info Started.")

    def synced_callback(self, color_msg, depth_msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환 (컬러: bgr8, 뎁스: pasthrough)
            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error("Image conversion failed: " + str(e))
            return
        
        # 컬러 이미지 처리: 그레이스케일, 가우시안 블러, Canny 엣지 검출
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 2)
        edge = cv2.Canny(gray, 50, 150, apertureSize=3)
        
        # HoughLinesP를 이용한 선 검출
        lines = cv2.HoughLinesP(edge, 1, np.pi / 180, 200, minLineLength=20, maxLineGap=10)
        
        horizontal_lines = []
        lowest_line_y = 0
        
        if lines is not None:
            # 수평선 판별을 위한 각도 임계값 (라디안 단위)
            min_angle = -np.pi / 180.0
            max_angle =  np.pi / 180.0
            
            for l in lines:
                x1, y1, x2, y2 = l[0]
                angle = np.arctan2(y2 - y1, x2 - x1)
                if min_angle < angle < max_angle:
                    horizontal_lines.append((x1, y1, x2, y2))
                    lowest_line_y = max(lowest_line_y, y1, y2)
            
            # 가까운 선들을 병합하여 보다 안정적인 결과 도출
            merged_lines = []
            height_threshold = 50  # 픽셀 단위 임계값
            for l1 in horizontal_lines:
                merged = False
                for l2 in merged_lines:
                    if abs(l1[1] - l2[1]) < height_threshold and abs(l1[3] - l2[3]) < height_threshold:
                        l2[0] = min(l2[0], l1[0])
                        l2[1] = (l2[1] + l1[1]) // 2
                        l2[2] = max(l2[2], l1[2])
                        l2[3] = (l2[3] + l1[3]) // 2
                        merged = True
                        break
                if not merged:
                    merged_lines.append(list(l1))
            
            # 검출된 수평선을 각각 빨간색과 파란색 선으로 표시
            for x1, y1, x2, y2 in horizontal_lines:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            for x1, y1, x2, y2 in merged_lines:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
            
            # 가장 첫번째 수평선의 중앙 좌표를 기준으로 뎁스 값 추출
            if horizontal_lines:
                x1, y1, x2, y2 = horizontal_lines[0]
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2
                
                # 주변 픽셀 5x5 영역의 중간값으로 안정적인 뎁스 값 획득
                region = depth_frame[max(0, mid_y-2):mid_y+3, max(0, mid_x-2):mid_x+3]
                depth_value = np.median(region)
                
                self.get_logger().info(f"Detected line at ({mid_x}, {mid_y}) with depth: {depth_value}")
        
        # 디버그용 이미지 창 출력
        cv2.imshow("Line Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
