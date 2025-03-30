#When the message "Permission denied: /dev/ttyx" appears, 
#change the permission settings on your serial_port
# eg. "sudo chmod 666 /dev/ttyUSB0"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import serial
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import numpy as np

comport_num = "/dev/ttyTHS1"
comport_baudrate = 115200
#comport_num = '/dev/tty' + input("IMU Port: /dev/tty")
#comport_baudrate = input("Baudrate: ")
ser = serial.Serial(port=comport_num,baudrate=comport_baudrate)

try:
	ser = serial.Serial(port=comport_num, baudrate=comport_baudrate)
except:
	print('Serial port error!')

def open_serial_port():
    try:
        return serial.Serial(port=comport_num, baudrate=comport_baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"⚠ Serial Port Error: {e}")
        return None
    
class imuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Imu, 'imu_data', qos_profile)

        self.tf_broadcaster = TransformBroadcaster(self)  # ✅ TF Broadcaster 추가

        self.ser = open_serial_port()
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.ser is None:
            self.get_logger().warn("❗ Serial port not open. Retrying...")
            self.ser = open_serial_port()
            return
        
        try:
            ser_data = self.ser.readline().decode('utf-8').strip()
            if not ser_data or ser_data[0] != '*':
                return

            data_list = [float(x) for x in ser_data.lstrip('*').split(',')]
            if len(data_list) != 10:
                return

            quaternion = np.array(data_list[:4])

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            imu_msg.angular_velocity.x = data_list[4]
            imu_msg.angular_velocity.y = data_list[5]
            imu_msg.angular_velocity.z = data_list[6]

            imu_msg.linear_acceleration.x = data_list[7]
            imu_msg.linear_acceleration.y = data_list[8]
            imu_msg.linear_acceleration.z = data_list[9]

            self.publisher.publish(imu_msg)

            # ✅ TF 변환 브로드캐스트 (imu_link → base_link)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"  # 부모 프레임
            t.child_frame_id = "imu_link"  # 자식 프레임

            # URDF에서 imu_link의 위치 (0, 0, 0.1) 적용
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.1
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            self.tf_broadcaster.sendTransform(t)  # ✅ TF 변환 브로드캐스트

        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
            self.ser = None


def main(args=None):
	rclpy.init(args=args)

	print("Starting imu_publisher..")

	node = imuPublisher()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
