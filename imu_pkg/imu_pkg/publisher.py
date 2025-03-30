import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
import serial

# 시리얼 포트 설정
comport_num = "/dev/ttyUSB1"
comport_baudrate = 115200

try:
    ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)
except:
    print('Serial port error!')


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        qos_profile = QoSProfile(depth=10)

        # `sensor_msgs/Imu` 타입으로 발행
        self.publisher = self.create_publisher(Imu, 'imu_data', qos_profile)

        timer_period = 0.02  # 50Hz (조정 가능)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ser_data = ser.readline().decode('utf-8').strip()

        try:
            # 문자열에서 값 추출: "[x, y, z]" → [x, y, z]
            values = ser_data.strip("[]").split(",")
            x, y, z = map(float, values)

            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # 가속도 데이터
            imu_msg.linear_acceleration.x = x
            imu_msg.linear_acceleration.y = y
            imu_msg.linear_acceleration.z = z

            # 각속도는 0으로 설정 (필요하면 추가)
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0

            # 메시지 발행
            self.publisher.publish(imu_msg)
            self.get_logger().info(f'Published IMU Data: {imu_msg}')

        except Exception as e:
            self.get_logger().error(f"Error parsing IMU data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
