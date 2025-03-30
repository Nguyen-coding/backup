import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        qos_profile = QoSProfile(depth=10)

        # sensor_msgs/Imu 타입으로 구독
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.callback,
            qos_profile)
        self.subscription   # prevent unused variable warning

    def callback(self, msg):
        # IMU 메시지에서 데이터 추출
        imu_data = {
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            }
        }
        print(imu_data)  # 출력


def main(args=None):
    rclpy.init(args=args)

    print("Starting imu_subscriber..")

    node = ImuSubscriber()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
