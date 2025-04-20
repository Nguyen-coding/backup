import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# RANSAC 함수: 내부에서 정규화한 법선을 사용
def estimate_ground_plane_ransac(points, max_iterations=50, distance_threshold=0.05):
    best_plane = None
    max_inliers = 0

    for _ in range(max_iterations):
        sample_idx = np.random.choice(points.shape[0], 3, replace=False)
        sample = points[sample_idx]
        v1 = sample[1] - sample[0]
        v2 = sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm_norm = np.linalg.norm(normal)
        if norm_norm == 0:
            continue

        # 정규화된 법선 사용
        normal_unit = normal / norm_norm
        a, b, c = normal_unit
        d = -np.dot(normal_unit, sample[0])
        distances = np.abs((points @ normal_unit + d))
        inliers = distances < distance_threshold
        num_inliers = np.sum(inliers)

        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_plane = (a, b, c, d)

    return best_plane

def get_step_mask_from_plane(points, depth_shape, plane_coeffs, threshold=0.10):
    a, b, c, d = plane_coeffs
    h, w = depth_shape
    points_reshaped = points.reshape((h, w, 3))
    distance_map = np.abs((points_reshaped @ np.array([a, b, c]) + d)) / np.linalg.norm([a, b, c])
    step_mask = (distance_map > threshold)
    return step_mask.astype(np.uint8) * 255, distance_map

class StepDetectorOptimized(Node):
    def __init__(self):
        super().__init__('step_detector_optimized')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)

        self.depth_image = None
        self.rgb_image = None
        self.frame_count = 0
        self.plane = None  # RANSAC으로 추정한 평면 계수 (a, b, c, d)
        self.THETA_THRESHOLD = 30  # 바닥으로 인정할 최대 기울기 (도)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def process(self):
        if self.depth_image is None or self.rgb_image is None:
            return

        self.frame_count += 1
        rgb = self.rgb_image.copy()

        # 1. Downsample depth image 및 포인트 클라우드 생성
        depth = cv2.resize(self.depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        depth = np.nan_to_num(depth, nan=0.0)
        h, w = depth.shape

        # 캘리브레이션 결과를 반영한 내부 파라미터 (픽셀 단위)
        fx, fy = 604.12383027, 604.1441706
        cx, cy = 324.92216095, 243.2522445

        zs = depth / 1000.0  # mm -> m 변환
        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        xs = (xs - cx) * zs / fx
        ys = (ys - cy) * zs / fy
        points = np.stack((xs, ys, zs), axis=-1).reshape(-1, 3)

        # ROI 제한: depth 이미지 하단 50%만 사용 (바닥이 보일 가능성이 높은 영역)
        roi_mask = np.zeros_like(zs, dtype=bool)
        roi_mask[int(h*0.5):, :] = True
        valid = (zs > 0.1) & (zs < 5.0) & roi_mask
        valid_points = points[valid.flatten()]

        # 10프레임마다 바닥 평면 추정 (RANSAC)
        if self.frame_count % 5 == 0 and valid_points.shape[0] >= 100:
            estimated_plane = estimate_ground_plane_ransac(valid_points)
            if estimated_plane is None:
                self.get_logger().warn("Plane estimation failed.")
                return
            else:
                # 세타(θ) 계산: 법선과 세계 수직 벡터(0,1,0) 사이의 각도
                a, b, c, d = estimated_plane
                normal = np.array([a, b, c])
                vertical = np.array([0, 1, 0])
                cos_theta = np.dot(normal, vertical) / (np.linalg.norm(normal) + 1e-6)
                theta = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
                self.get_logger().info(f"Estimated plane theta: {theta:.2f} deg")
                if theta < self.THETA_THRESHOLD:
                    self.plane = estimated_plane
                else:
                    self.get_logger().info(f"Rejected plane due to theta = {theta:.2f} deg")
                    self.plane = None

        # 평면 정보 오버레이
        if self.plane is not None:
            a, b, c, d = self.plane
            plane_text = f"Plane: a={a:.2f}, b={b:.2f}, c={c:.2f}, d={d:.2f}"
            cv2.putText(rgb, plane_text, (10, rgb.shape[0]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        if self.plane is not None:
            # 2. 단차 마스크와 거리 맵 생성 (downsample 해상도 사용)
            original_step_mask, distance_map = get_step_mask_from_plane(points, depth.shape, self.plane)
            
            # 평면 영역 디버깅: 0.02m 이하인 영역 오버레이 (노란색)
            plane_thresh = 0.02
            plane_mask = (distance_map < plane_thresh).astype(np.uint8) * 255
            plane_mask_resized = cv2.resize(plane_mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
            overlay = rgb.copy()
            overlay[plane_mask_resized > 0] = (0, 255, 255)
            cv2.addWeighted(overlay, 0.4, rgb, 0.6, 0, rgb)
            
            # 3. Morphological 연산으로 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            filtered_mask = cv2.morphologyEx(original_step_mask, cv2.MORPH_OPEN, kernel)
            
            # 4. 윤곽선 검출
            contours, _ = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 5. 원본 해상도 3D 배열 재구성
            points_reshaped = points.reshape((h, w, 3))
            a, b, c, d = self.plane
            distance_map = np.abs((points_reshaped @ np.array([a, b, c]) + d)) / np.linalg.norm([a, b, c])
            
            # 6. 후보 영역 결정: 각 윤곽선 별로 전방 거리와 면적 등을 구해 리스트에 저장
            candidates = []
            MIN_AREA = 200  # 최소 면적 (픽셀 단위)
            # 후보 조건 수정: 단차 높이 조건 상한을 1.0m, 전방 거리가 0.5m 이상 1.0m 미만인 경우에만 후보로 인식
            for cnt in contours:
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                area = w_box * h_box
                if area < MIN_AREA:
                    continue
                contour_mask = np.zeros_like(filtered_mask)
                cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
                contour_indices = contour_mask.astype(bool)
                step_heights = distance_map[contour_indices]
                median_step_height = np.median(step_heights)
                step_z = points_reshaped[..., 2][contour_indices]
                median_step_forward = np.median(step_z)
                aspect_ratio = w_box / h_box if h_box != 0 else 0

                if 0.05 < median_step_height < 1.0 and 0.5 < median_step_forward < 1.0:
                    candidates.append({
                        "bounding_rect": (x, y, w_box, h_box),
                        "area": area,
                        "median_step_height": median_step_height,
                        "median_step_forward": median_step_forward,
                        "aspect_ratio": aspect_ratio,
                    })

            # 7. 후보 정렬: 전방 거리가 가장 짧은(즉, 카메라에 가장 가까운) 후보 우선 선택
            if candidates:
                candidates.sort(key=lambda cand: (cand["median_step_forward"], -cand["area"]))
                best = candidates[0]
                x, y, w_box, h_box = best["bounding_rect"]
                median_step_height = best["median_step_height"]
                median_step_forward = best["median_step_forward"]

                cv2.rectangle(rgb, (x, y), (x+w_box, y+h_box), (255, 0, 0), 3)
                cv2.putText(rgb, f"Step: {median_step_forward:.2f}m", (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                self.get_logger().info(
                    f"Final candidate: Height = {median_step_height:.2f} m, Forward = {median_step_forward:.2f} m"
                )

            # 후보 영역 윤곽선 (디버깅): 빨간색 사각형
            resized_mask = cv2.resize(filtered_mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
            contours_vis, _ = cv2.findContours(resized_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_vis:
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                cv2.rectangle(rgb, (x, y), (x+w_box, y+h_box), (0, 0, 255), 1)

        # 8. 최종 시각화: RGB와 depth 컬러맵 병렬 배치
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        combined = np.hstack((rgb, depth_color))
        cv2.imshow("Optimized Step Detection", combined)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = StepDetectorOptimized()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
