import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import Image

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.cb, 10)

    def cb(self, msg: Image):
        depth = rnp.image.image_to_numpy(msg)   # H×W numpy array (mm 단위)
        # 예: 화면 중앙 100×100 영역에서 유효값만 추출
        h, w = depth.shape
        roi = depth[h//2-50:h//2+50, w//2-50:w//2+50]
        valid = roi[roi>0]
        if valid.size:
            min_d = valid.min() / 1000.0  # → m 단위
            max_d = valid.max() / 1000.0
            self.get_logger().info(f"ROI depth: {min_d:.2f} ~ {max_d:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
