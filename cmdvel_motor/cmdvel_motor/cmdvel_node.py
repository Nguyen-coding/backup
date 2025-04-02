import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *  # Dynamixel SDK import

# === 사용자 설정 ===
DEVICENAME = "/dev/ttyUSB1"
BAUDRATE = 1000000
DXL_ID = 14
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_OPERATING_MODE = 11

TORQUE_ENABLE = 1
VELOCITY_CONTROL_MODE = 1  # 1: Velocity Mode

class CmdVelMotorNode(Node):
    def __init__(self):
        super().__init__('cmdvel_motor_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )

        # Dynamixel 초기화
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            self.get_logger().info(f"Opened port: {DEVICENAME}")
        else:
            self.get_logger().error(f"Failed to open port: {DEVICENAME}")
            return

        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f"Baudrate set to: {BAUDRATE}")
        else:
            self.get_logger().error("Failed to set baudrate")
            return

        # 모터 설정
        # 1. 속도 제어 모드로 변경
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)

        # 2. Torque Enable
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(f"Dynamixel ID {DXL_ID} ready.")

    def cmdvel_callback(self, msg: Twist):
        linear = msg.linear.x  # m/s

        # 단순화: 전진 속도만 사용 (단일 모터)
        # 속도 단위 변환: DXL은 약 0.229 rpm/단위 → 1.0 m/s → 약 2650 정도로 가정
        scale = 2650  # 속도 변환 비율은 실험적으로 맞추면 정확함
        dxl_speed = int(linear * scale)

        # 방향에 따라 부호 조정
        if dxl_speed < 0:
            dxl_speed = dxl_speed + (1 << 32)  # 음수 속도 처리 (2's complement)

        # 명령 전송
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_VELOCITY, dxl_speed)

        self.get_logger().info(f"[cmd_vel] linear.x = {linear:.2f} → Dynamixel Speed = {dxl_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    node.portHandler.closePort()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
