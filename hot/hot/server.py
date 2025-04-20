import rclpy                         # ROS2 client library for Python
from rclpy.node import Node          # ROS2 노드 클래스
import socket                        # UDP 통신을 위한 소켓 모듈
import serial                        # UART 통신을 위한 PySerial 모듈
import threading                     # 스레드를 사용하기 위한 모듈

class UdpUartBridge(Node):
    def __init__(self):
        super().__init__('udp_uart_bridge')
        # 현재 기기의 IP 주소를 가져오기 (get_ip 함수 사용)
        self.udp_ip = self.get_ip()
        # UDP 통신에 사용할 포트 번호를 지정 8888
        self.udp_port = 8888
        # UDP 소켓 생성 (IPv4, UDP)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 지정한 IP와 포트에 소켓을 바인딩 (연결 대기 준비)
        self.sock.bind((self.udp_ip, self.udp_port))

        try:
            # /dev/ttyUSB0 포트에서 Baudrate 115200, 타임아웃 1초로 UART 시리얼 통신 연결 시도
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            # 연결 성공 메시지 출력
            self.get_logger().info("UART 연결 성공")
        except Exception as e:
            # UART 연결 실패 시, 예외 메시지를 로깅 (어떤 오류인지 출력)
            self.get_logger().error(f"UART 연결 실패: {e}")
            # UART 객체를 None으로 설정하여 이후 조건문에서 처리할 수 있도록 함
            self.ser = None

        # --------------------- UDP 수신 스레드 시작 ---------------------
        # 별도의 스레드에서 UDP 메시지를 수신받기 위한 함수 실행 (blocking I/O 처리로 메인 스레드와 분리)
        self.udp_thread = threading.Thread(target=self.receive_udp_loop)
        self.udp_thread.daemon = True  # 데몬 스레드로 설정하여 메인 스레드 종료 시 함께 종료되도록 함
        self.udp_thread.start()

        # 초기 UDP 수신 시작 로그 출력
        self.get_logger().info(f"UDP 수신 중: {self.udp_ip}:{self.udp_port}")

    def get_ip(self):
        """
        현재 기기의 로컬 IP 주소를 가져오는 함수.
        Google Public DNS 서버("8.8.8.8", 80)에 임의로 연결하여 소켓이 사용하는 IP를 확인한 후,
        연결을 종료하고 IP 주소 반환.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip

    def receive_udp_loop(self):
        """
        UDP 통신으로 들어오는 데이터를 지속적으로 확인하는 무한 루프 함수.
        각 UDP 패킷을 수신하면 해당 데이터를 디코딩하여 로그로 출력,
        응답 메시지를 해당 주소로 전송하며, 필요시 UART로도 데이터를 전송.
        """
        while True:
            # 최대 1024 바이트 크기의 패킷을 수신 (blocking call)
            data, addr = self.sock.recvfrom(1024)
            # 받은 데이터를 UTF-8 형식으로 디코딩하여 문자열로 변환
            msg = data.decode('utf-8')
            self.get_logger().info(f"UDP 수신: {msg} from {addr}")

            # UDP 응답 메시지 전송: 클라이언트에게 연결이 성공적으로 수립되었음을 알림
            self.sock.sendto("Connected successfully!".encode('utf-8'), addr)

            # UART 연결이 성공적으로 이루어졌는지 확인 후 처리
            if self.ser:
                try:
                    # UART 수신 및 송신 버퍼 초기화
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    # 받은 UDP 데이터를 그대로 UART 포트로 전송
                    self.ser.write(data)
                    self.get_logger().info(f"UART 송신: {msg.strip()}")
                except Exception as e:
                    # UART 송신 도중 오류 발생 시 해당 오류 메시지를 로그에 출력
                    self.get_logger().error(f"UART 오류: {e}")
            else:
                # UART 연결이 되어 있지 않을 경우, 디버그 목적으로 메시지 출력만 수행
                self.get_logger().info(f"[디버그] UART 없음 - 메시지 출력만: {msg.strip()}")

def main(args=None):
    """
    ROS2 노드를 초기화하고 UdpUartBridge 노드를 실행하는 메인 함수.
    rclpy.spin 함수를 사용하여 노드가 종료될 때까지 지속적으로 실행함.
    """
    # ROS2 초기화
    rclpy.init(args=args)
    # UdpUartBridge 노드 인스턴스 생성
    node = UdpUartBridge()
    # ROS2 메시지 루프 실행 (이 호출은 노드를 계속 실행 상태로 유지)
    rclpy.spin(node)
    # 노드 종료 시, 노드를 안전하게 소멸시키고 ROS2 종료
    node.destroy_node()
    rclpy.shutdown()
