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
            self.get_logger().info("수평선이 2개 미만 검출됨.")
            cv2.putText(vis, "Less than 2 lines", (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
            cv2.imshow("RGB", vis)
            cv2.waitKey(1)
            return

        # y중심 기준 오름차순 정렬
        horizontal.sort(key=lambda t: (t[1]+t[3]) / 2.0)

        # 가장 낮은 위치에서부터 두 개를 선택
        line1 = horizontal[-2]  # 두 번째로 낮은 선
        line2 = horizontal[-1]  # 가장 낮은 선

        # y중심 계산
        y1 = (line1[1] + line1[3]) / 2.0
        y2 = (line2[1] + line2[3]) / 2.0
        delta_v = abs(y2 - y1)

        # 가장 낮은 선(line2) 상의 깊이 샘플링
        xs = np.linspace(line2[0], line2[2], num=20).astype(int)
        ys = np.linspace(line2[1], line2[3], num=20).astype(int)
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
        step_height = (delta_v * Z / self.fy) * 2  # 필요시 계수 곱하기

        # 시각화
        cv2.line(vis, (line1[0],line1[1]), (line1[2],line1[3]), (0,255,0), 2)
        cv2.line(vis, (line2[0],line2[1]), (line2[2],line2[3]), (0,255,0), 2)
        text = f"Step Height: {step_height:.2f} m"
        cv2.putText(vis, text, (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

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
