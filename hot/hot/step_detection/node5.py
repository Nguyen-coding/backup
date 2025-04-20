import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StepDetectorHoughPixelHeight(Node):
    def __init__(self):
        super().__init__('step_detector_hough_pixel_height')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_image = None
        self.rgb_image = None

        # 보정된 카메라 내부 파라미터
        self.fx, self.fy = 604.12383027, 604.1441706
        self.cx, self.cy = 324.92216095, 243.2522445

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # depth가 아직 없더라도 라인만 시각화하고 싶다면 여기도 process() 호출 가능
        self.process()

    def process(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        rgb = self.rgb_image.copy()
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # 허프 선 검출
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=80,
                                minLineLength=100, maxLineGap=10)
        horizontal = []
        if lines is not None:
            for l in lines:
                x1,y1,x2,y2 = l[0]
                angle = abs(np.degrees(np.arctan2(y2-y1, x2-x1)))
                if angle < 10:  # 수평에 근접
                    horizontal.append((x1,y1,x2,y2))

        vis = rgb.copy()
        if len(horizontal) < 2:
            # 검출된 선이 부족할 때도 RGB 창에 원본만 보여줌
            self.get_logger().info("수평선이 2개 미만 검출됨.")
            cv2.putText(vis, "Less than 2 lines", (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
            cv2.imshow("RGB", vis)
            cv2.waitKey(1)
            return

        # y좌표 기준 정렬
        horizontal.sort(key=lambda t: (t[1]+t[3])/2)
        top = horizontal[0]
        bottom = horizontal[-1]

        # 두 선의 y 픽셀 좌표 중앙값
        y_top = (top[1] + top[3]) / 2.0
        y_bot = (bottom[1] + bottom[3]) / 2.0
        delta_v = y_bot - y_top

        # bottom 선 상의 깊이 샘플링 (중앙값)
        xs = np.linspace(bottom[0], bottom[2], num=20).astype(int)
        ys = np.linspace(bottom[1], bottom[3], num=20).astype(int)
        depths = []
        H, W = self.depth_image.shape
        for ux, vy in zip(xs, ys):
            if 0 <= vy < H and 0 <= ux < W:
                d = self.depth_image[vy, ux]
                if d > 0:
                    depths.append(d / 1000.0)  # mm→m
        if not depths:
            self.get_logger().info("depth 샘플링 실패.")
            cv2.putText(vis, "Depth sampling fail", (10,80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
            cv2.imshow("RGB", vis)
            cv2.waitKey(1)
            return

        Z = np.median(depths)

        # 픽셀 차이를 실제 높이로 변환
        step_height = (delta_v * Z / self.fy) * 2  # 보정 계수

        # 시각화: 수평선과 텍스트를 RGB 위에 그리기
        cv2.line(vis, (top[0],top[1]), (top[2],top[3]), (0,255,0), 2)
        cv2.line(vis, (bottom[0],bottom[1]), (bottom[2],bottom[3]), (0,255,0), 2)
        text = f"Step Height: {step_height:.2f} m"
        cv2.putText(vis, text, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        
        # 단일 창에 결과 표시
        self.get_logger().info(text)
        cv2.imshow("RGB", vis)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = StepDetectorHoughPixelHeight()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StepDetectorHoughClustering(Node):
    def __init__(self):
        super().__init__('step_detector_hough_clustering')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)

        self.depth_image = None
        self.rgb_image = None
        # 보정된 카메라 내부 파라미터
        self.fx, self.fy = 604.12383027, 604.1441706
        self.cx, self.cy = 324.92216095, 243.2522445
        # 다운샘플 비율
        self.scale = 0.5

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def sample_line_depth(self, line_ds):
        x1,y1,x2,y2 = line_ds
        length = max(int(np.hypot(x2-x1, y2-y1)), 1)
        zs = []
        H, W = self.depth_image.shape
        for i in range(0, length, max(1, length//20)):
            u_ds = int(x1 + (x2-x1)*i/length)
            v_ds = int(y1 + (y2-y1)*i/length)
            u = int(u_ds / self.scale)
            v = int(v_ds / self.scale)
            if 0 <= u < W and 0 <= v < H:
                d = self.depth_image[v, u]
                if d>0:
                    zs.append(d/1000.0)
        return np.median(zs) if zs else None

    def process(self):
        if self.depth_image is None or self.rgb_image is None:
            return

        rgb = self.rgb_image.copy()
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=80, minLineLength=100, maxLineGap=10)

        # 수평선 후보 수집
        horizontal = []
        if lines is not None:
            for l in lines:
                x1,y1,x2,y2 = l[0]
                angle = abs(np.degrees(np.arctan2(y2-y1, x2-x1)))
                if angle < 10:  # 수평
                    y_mid = (y1 + y2) / 2.0
                    horizontal.append({'line':(x1,y1,x2,y2), 'y_mid':y_mid})

        if len(horizontal) < 2:
            self.get_logger().info("수평선 2개 미만 검출됨.")
            # 단일 창에 RGB만 표시
            cv2.imshow("Step Detection", rgb)
            cv2.waitKey(1)
            return

        # y_mid 정렬 및 최대 간극 탐색
        y_vals = sorted([h['y_mid'] for h in horizontal])
        gaps = np.diff(y_vals)
        idx = np.argmax(gaps)
        # 클러스터 기준 임계 y
        threshold_y = (y_vals[idx] + y_vals[idx+1]) / 2.0

        # 두 클러스터 분리
        cluster1 = [h for h in horizontal if h['y_mid'] <= threshold_y]
        cluster2 = [h for h in horizontal if h['y_mid'] > threshold_y]

        # 대표 선 선택 (cluster 중앙값에 가장 근접한 선)
        def pick_rep(cluster):
            mids = np.array([h['y_mid'] for h in cluster])
            target = np.median(mids)
            rep = min(cluster, key=lambda h: abs(h['y_mid'] - target))
            return rep['line'], rep['y_mid']

        top_line, y_top = pick_rep(cluster1)
        bottom_line, y_bot = pick_rep(cluster2)

        # 깊이 샘플링 및 높이 계산
        Z_top = self.sample_line_depth(top_line)
        Z_bot = self.sample_line_depth(bottom_line)
        if Z_top is None or Z_bot is None:
            self.get_logger().info("깊이 샘플링 실패.")
            cv2.imshow("Step Detection", rgb)
            cv2.waitKey(1)
            return
        # 평균 깊이
        Z = (Z_top + Z_bot) / 2.0
        # 픽셀 거리 (원본)
        delta_v_ds = abs(y_bot - y_top)
        delta_v = delta_v_ds / self.scale
        # 높이 계산
        step_height = delta_v * Z / self.fy

        # 시각화
        cv2.line(rgb, (top_line[0],top_line[1]), (top_line[2],top_line[3]), (0,255,0),2)
        cv2.line(rgb, (bottom_line[0],bottom_line[1]), (bottom_line[2],bottom_line[3]), (0,255,0),2)
        text = f"Step Height: {step_height:.2f} m"
        cv2.putText(rgb, text, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255),2)
        self.get_logger().info(text)

        cv2.imshow("Step Detection", rgb)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = StepDetectorHoughClustering()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__=='__main__':
    main()
"""