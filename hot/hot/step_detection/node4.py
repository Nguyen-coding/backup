import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DirectDepthStepDetector(Node):
    def __init__(self):
        super().__init__('direct_depth_step_detector')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_image = None
        self.rgb_image = None

    def depth_callback(self, msg):
        # Depth 이미지는 "passthrough" 인코딩 사용
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def process(self):
        if self.depth_image is None or self.rgb_image is None:
            return

        # 1. Depth 전처리: 다운샘플 및 NaN 제거
        depth = cv2.resize(self.depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        depth = np.nan_to_num(depth, nan=0.0)
        h, w = depth.shape
        # mm 단위라면, m 단위로 변환 (예: mm→m)
        depth_m = depth / 1000.0

        # 2. Sobel 필터로 x, y 방향 깊이 그라데이션 계산
        sobelx = cv2.Sobel(depth_m, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_m, cv2.CV_64F, 0, 1, ksize=3)
        grad_magnitude = np.sqrt(sobelx**2 + sobely**2)

        # 3. 그라데이션을 0~255 범위로 정규화하여 임계값 적용 용이하게 함
        grad_norm = cv2.normalize(grad_magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # 4. 임계값 적용 및 Morphology로 잡음 제거
        ret, thresh = cv2.threshold(grad_norm, 30, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # 5. 윤곽선 검출: 단차 후보 영역 추출
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6. 각 후보 영역별로 전방 거리와 높이 계산 (중앙 영역 기준)
        candidates = []
        MIN_AREA = 100
        for cnt in contours:
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            area = w_box * h_box
            if area < MIN_AREA:
                continue

            candidate_region = depth_m[y:y+h_box, x:x+w_box]
            if candidate_region.size == 0:
                continue
            median_forward = np.median(candidate_region)

            # 중앙 영역: 후보 영역의 중앙 60% (가로, 세로 각각 60%의 중앙 부분)
            cx = x + w_box // 2
            cy = y + h_box // 2
            half_w = int(w_box * 0.3)
            half_h = int(h_box * 0.3)
            x1 = max(cx - half_w, 0)
            y1 = max(cy - half_h, 0)
            # 주의: candidate_region의 크기는 (w_box, h_box)와 동일하므로, 중앙 영역 좌표는 상대 좌표로 계산
            # 상대 좌표로 변환: 후보 영역 내에서 중앙 좌표는 (w_box//2, h_box//2)
            rel_cx, rel_cy = w_box // 2, h_box // 2
            rel_x1 = max(rel_cx - half_w, 0)
            rel_y1 = max(rel_cy - half_h, 0)
            rel_x2 = min(rel_cx + half_w, candidate_region.shape[1])
            rel_y2 = min(rel_cy + half_h, candidate_region.shape[0])
            central_region = candidate_region[rel_y1:rel_y2, rel_x1:rel_x2]
            if central_region.size == 0:
                continue
            p90 = np.percentile(central_region, 90)
            p10 = np.percentile(central_region, 10)
            candidate_height = p90 - p10

            # 후보 조건: 전방 거리는 0.5~1.2m, 후보 높이는 0.02~1.2m로 임계값 조정
            if 0.02 < candidate_height < 1.2 and 0.5 < median_forward < 1.2:
                candidates.append({
                    "bbox": (x, y, w_box, h_box),
                    "median_forward": median_forward,
                    "candidate_height": candidate_height
                })
                self.get_logger().info(
                    f"Candidate: BBox=({x},{y},{w_box},{h_box}), D={median_forward:.2f}m, H={candidate_height:.2f}m"
                )

        # 7. 모든 후보를 시각화 (가장 가까운 후보만 선택하는 대신, 모두 표시)
        result = self.rgb_image.copy()
        for cand in candidates:
            x, y, w_box, h_box = cand["bbox"]
            median_forward = cand["median_forward"]
            candidate_height = cand["candidate_height"]
            cv2.rectangle(result, (x*2, y*2), ((x+w_box)*2, (y+h_box)*2), (0, 255, 0), 2)
            text = f"H:{candidate_height:.2f}m, D:{median_forward:.2f}m"
            cv2.putText(result, text, (x*2, y*2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 8. 추가: 만약 단일 후보만 선택하려면, 아래 조건으로 최종 후보를 선택 (전방 거리 가장 짧은 경우)
        if candidates:
            candidates.sort(key=lambda cand: (cand["median_forward"], -cand["bbox"][2]*cand["bbox"][3]))
            best_candidate = candidates[0]
            x, y, w_box, h_box = best_candidate["bbox"]
            median_forward = best_candidate["median_forward"]
            candidate_height = best_candidate["candidate_height"]
            cv2.rectangle(result, (x*2, y*2), ((x+w_box)*2, (y+h_box)*2), (0, 0, 255), 3)
            best_text = f"Best -> H:{candidate_height:.2f}m, D:{median_forward:.2f}m"
            cv2.putText(result, best_text, (x*2, (y-20)*2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.get_logger().info(
                f"Best Candidate: Height={candidate_height:.2f} m, Forward={median_forward:.2f} m"
            )
        else:
            self.get_logger().info("No candidate step detected within thresholds.")

        # 9. 결과 및 디버깅 창 출력
        cv2.imshow("Depth Gradient", grad_norm)
        cv2.imshow("Thresholded Gradient", thresh)
        cv2.imshow("Detected Steps", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DirectDepthStepDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DirectDepthStepDetector(Node):
    def __init__(self):
        super().__init__('direct_depth_step_detector')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_image = None
        self.rgb_image = None

    def depth_callback(self, msg):
        # Depth 이미지는 "passthrough" 인코딩 사용
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def process(self):
        if self.depth_image is None or self.rgb_image is None:
            return

        # 1. Depth 전처리: 다운샘플 및 NaN 제거
        depth = cv2.resize(self.depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        depth = np.nan_to_num(depth, nan=0.0)
        h, w = depth.shape
        # mm 단위라면, m 단위로 변환 (예: mm→m)
        depth_m = depth / 1000.0

        # 2. Sobel 필터로 x, y 방향 깊이 그라데이션 계산
        sobelx = cv2.Sobel(depth_m, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(depth_m, cv2.CV_64F, 0, 1, ksize=3)
        grad_magnitude = np.sqrt(sobelx**2 + sobely**2)

        # 3. 그라데이션을 0~255 범위로 정규화하여 임계값 적용 용이하게 함
        grad_norm = cv2.normalize(grad_magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # 4. 임계값 적용 및 Morphology로 잡음 제거
        ret, thresh = cv2.threshold(grad_norm, 30, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # 5. 윤곽선 검출: 단차 후보 영역 추출
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6. 각 후보 영역별로 전방 거리와 높이 계산 (중앙 영역 기준)
        candidates = []
        MIN_AREA = 100
        for cnt in contours:
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            area = w_box * h_box
            if area < MIN_AREA:
                continue

            candidate_region = depth_m[y:y+h_box, x:x+w_box]
            if candidate_region.size == 0:
                continue
            median_forward = np.median(candidate_region)

            # 중앙 영역: 후보 영역의 중앙 60% (가로, 세로 각각 60%의 중앙 부분)
            cx = x + w_box // 2
            cy = y + h_box // 2
            half_w = int(w_box * 0.3)
            half_h = int(h_box * 0.3)
            x1 = max(cx - half_w, 0)
            y1 = max(cy - half_h, 0)
            # 주의: candidate_region의 크기는 (w_box, h_box)와 동일하므로, 중앙 영역 좌표는 상대 좌표로 계산
            # 상대 좌표로 변환: 후보 영역 내에서 중앙 좌표는 (w_box//2, h_box//2)
            rel_cx, rel_cy = w_box // 2, h_box // 2
            rel_x1 = max(rel_cx - half_w, 0)
            rel_y1 = max(rel_cy - half_h, 0)
            rel_x2 = min(rel_cx + half_w, candidate_region.shape[1])
            rel_y2 = min(rel_cy + half_h, candidate_region.shape[0])
            central_region = candidate_region[rel_y1:rel_y2, rel_x1:rel_x2]
            if central_region.size == 0:
                continue
            p90 = np.percentile(central_region, 90)
            p10 = np.percentile(central_region, 10)
            candidate_height = p90 - p10

            # 후보 조건: 전방 거리는 0.5~1.2m, 후보 높이는 0.02~1.2m로 임계값 조정
            if 0.02 < candidate_height < 1.2 and 0.5 < median_forward < 1.2:
                candidates.append({
                    "bbox": (x, y, w_box, h_box),
                    "median_forward": median_forward,
                    "candidate_height": candidate_height
                })
                self.get_logger().info(
                    f"Candidate: BBox=({x},{y},{w_box},{h_box}), D={median_forward:.2f}m, H={candidate_height:.2f}m"
                )

        # 7. 모든 후보를 시각화 (가장 가까운 후보만 선택하는 대신, 모두 표시)
        result = self.rgb_image.copy()
        for cand in candidates:
            x, y, w_box, h_box = cand["bbox"]
            median_forward = cand["median_forward"]
            candidate_height = cand["candidate_height"]
            cv2.rectangle(result, (x*2, y*2), ((x+w_box)*2, (y+h_box)*2), (0, 255, 0), 2)
            text = f"H:{candidate_height:.2f}m, D:{median_forward:.2f}m"
            cv2.putText(result, text, (x*2, y*2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 8. 추가: 만약 단일 후보만 선택하려면, 아래 조건으로 최종 후보를 선택 (전방 거리 가장 짧은 경우)
        if candidates:
            candidates.sort(key=lambda cand: (cand["median_forward"], -cand["bbox"][2]*cand["bbox"][3]))
            best_candidate = candidates[0]
            x, y, w_box, h_box = best_candidate["bbox"]
            median_forward = best_candidate["median_forward"]
            candidate_height = best_candidate["candidate_height"]
            cv2.rectangle(result, (x*2, y*2), ((x+w_box)*2, (y+h_box)*2), (0, 0, 255), 3)
            best_text = f"Best -> H:{candidate_height:.2f}m, D:{median_forward:.2f}m"
            cv2.putText(result, best_text, (x*2, (y-20)*2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.get_logger().info(
                f"Best Candidate: Height={candidate_height:.2f} m, Forward={median_forward:.2f} m"
            )
        else:
            self.get_logger().info("No candidate step detected within thresholds.")

        # 9. 결과 및 디버깅 창 출력
        cv2.imshow("Depth Gradient", grad_norm)
        cv2.imshow("Thresholded Gradient", thresh)
        cv2.imshow("Detected Steps", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DirectDepthStepDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
