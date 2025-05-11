#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque
import time
import serial
from hot.msg import StepEvent

class Shock(Node):
    def __init__(self):
        super().__init__('shock')
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("✅ UART 포트 연결 성공 (/dev/ttyUSB0)")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ UART 연결 실패: {e}")
            self.serial = None
        
        # IMU 구독
        self.subscription = self.create_subscription(Imu,'/imu_data',self.imu_callback,10)
        # 단차 감지 신호 구독
        self.step_sub = self.create_subscription(StepEvent,'/step_detected',self.step_callback,10)

        # 최근 IMU 데이터 저장 (슬라이딩 윈도우 평균용)
        self.accel_history = deque()
        self.window_duration = 1.0  # seconds
        self.max_buffer_size = 150  # for 100Hz IMU

        # 튐 감지 기준값
        self.accel_threshold = 2.0  # m/s²

        self.last_step_height = 0.0   

        # 단차 감지 유효시간 (이 시간 이내의 충격만 유효)
        self.step_detected_time = 0.0
        self.step_valid_duration = 120.0  # seconds

        self.get_logger().info('✅ IMU 충격 감지 노드가 시작되었습니다.')

    def step_callback(self, msg: StepEvent):
        height = msg.height
        if height <= 0.20:
            self.last_step_height = height
            self.step_detected_time = time.time()
            self.get_logger().info(f"📌 단차 감지됨 ({height*100:.1f}cm) → 충격 감지 활성화")

    def imu_callback(self, msg: Imu):
        now = time.time()

        # 현재 가속도
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # IMU 버퍼에 추가
        self.accel_history.append((now, ax, ay, az))

        # 오래된 데이터 제거
        while self.accel_history and now - self.accel_history[0][0] > self.window_duration:
            self.accel_history.popleft()

        if len(self.accel_history) < 5:
            return  # 평균 계산에 데이터 부족

        # 평균 계산
        avg_ax = sum(d[1] for d in self.accel_history) / len(self.accel_history)
        avg_ay = sum(d[2] for d in self.accel_history) / len(self.accel_history)
        avg_az = sum(d[3] for d in self.accel_history) / len(self.accel_history)

        # 튐 정도 계산
        delta_ax = abs(ax - avg_ax)
        delta_ay = abs(ay - avg_ay)
        delta_az = abs(az - avg_az)

        # 충격 감지 여부
        impact = delta_ax > self.accel_threshold or \
                 delta_ay > self.accel_threshold or \
                 delta_az > self.accel_threshold

        if impact:
            if now - self.step_detected_time < self.step_valid_duration:
                if self.last_step_height > 0.15:
                    level = "HIGH (20cm)"
                    uart_code = 35
                elif self.last_step_height > 0.10:
                    level = "MID (15cm)"
                    uart_code = 34
                elif self.last_step_height > 0.05:
                    level = "LOW (10cm)"
                    uart_code = 33
                else:
                    level = "VERY_LOW (5cm)"
                    uart_code = 32
                
                self.get_logger().warn(f"🚨 {level} 단차로 인한 충격 감지!")

                if self.serial:
                    try:
                        self.serial.write((uart_code + "\n").encode())
                        self.get_logger().info(f"📤 UART 전송: {uart_code}")
                    except serial.SerialException as e:
                        self.get_logger().error(f"UART 전송 실패: {e}")

            else:
                self.get_logger().info("IMU 충격 감지됨 (단차 무관)")

def main(args=None):
    rclpy.init(args=args)
    node = Shock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
