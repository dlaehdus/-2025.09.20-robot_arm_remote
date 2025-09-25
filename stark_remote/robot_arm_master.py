import rclpy                                                                # ros2 pytion 라이브러리
from rclpy.node import Node                                                 # ros2 노드 클래스
from std_msgs.msg import Int32MultiArray                                    # 정수 배열 메시지 타입 모터 위치 전송용
from dynamixel_sdk import *                                                 # 모터 제어 SDK
import signal                                                               # ctrl + c  시스템 신호 처리
import sys                                                                  # 시스템 경로 관리 명령줄인수, 프로그램종료등
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy    # ROS2 통신 품질 설정

PORT_CON = '/dev/ttyACM1'                                                   # 시리얼 포트 경로
DXL_MASTER_IDS = [0, 1, 2, 3, 4, 5]                                         # MASTER MOTOR ID
RECODE_TIME = 0.02                                                          # 50Hz 즉 50ms주기로 데이터 읽기 최대로 설정한값임 건들지 마셈

class MasterNode(Node):
    def __init__(self):
        super().__init__('robot_arm_master')                                # ros2 node 클래스를 상속받아 robot_arm_master라는 노드를 생성
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,                   # 최대한 빠른 전송 (일부 손실 허용)
            history=QoSHistoryPolicy.KEEP_LAST,                             # 최신 메시지만 유지
            depth=1                                                         # 큐에 1개 메시지만 저장
        )
        
        self.publisher_ = self.create_publisher(                            # int32타입으로 master_position 토픽에 발행                            
            Int32MultiArray, 
            'master_positions', 
            qos_profile
        )                           

        self.dxl_ids = DXL_MASTER_IDS                                       # 모터 ID목록 저장
        self.portHandler = PortHandler(PORT_CON)                            # 포트 핸들러 생성
        self.packetHandler = PacketHandler(2.0)                             # 패킷 핸들러 생성

        self.previous_positions = [0] * 6                                   # 이전 위치값 저장용 배열
        self.position_threshold = 10                                        # 10단위 이상 변화할 때만 퍼블리시

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(57600):
            self.get_logger().error("보드레이트 설정 실패.")
            raise RuntimeError("Failed to set baudrate")

        for i, dxl_id in enumerate(self.dxl_ids):                           # 모든 마스터 모터의 토크를 비활성화
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            if result != COMM_SUCCESS:
                self.get_logger().error(f"모터 ID {dxl_id} 토크 비활성화 실패.")
                raise RuntimeError(f"Failed to disable torque for motor ID {dxl_id}")

        signal.signal(signal.SIGINT, self.signal_handler)                   # ctrl + c 신호 처리기 등록
        self.timer = self.create_timer(RECODE_TIME, self.control_loop)      # 0.02초마다 control_loop함수를 실행하는 타이머 생성
        
        self.get_logger().info("Master node initialized (20Hz publishing rate).")
        self.get_logger().info("Press Ctrl+C to stop.")

    def read_motor_positions(self):                                         # 모터의 현재 위치를 읽음 읽기 실패시 30000으로 설정
        positions = []
        for dxl_id in self.dxl_ids:
            position, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
            if result != COMM_SUCCESS:
                self.get_logger().warn(f"모터 ID {dxl_id} 위치 읽기 실패.")
                position = 30000  
            positions.append(int(position))
        return positions

    def has_significant_change(self, current_positions):                    # 이전 위치와 현재 위치를 비교
        """위치 변화가 임계값을 넘는지 확인"""
        for i, (current, previous) in enumerate(zip(current_positions, self.previous_positions)):
            if abs(current - previous) > self.position_threshold:
                return True
        return False

    def control_loop(self):                                                 # 메인 제어 루프
        positions = self.read_motor_positions()                             # 모터의 현재 위치를 읽음
        if self.has_significant_change(positions):                          # 10단위 이상 변화가 있으면 True
            msg = Int32MultiArray()                                         # ROS2에서 여러 개의 정수(int32)를 담을 수 있는 메시지 객체를 생성
            msg.data = positions                                            # 읽어온 모터 위치값 리스트를 메시지의 데이터 필드에 할당합니다
            self.publisher_.publish(msg)                                    # master_positions 토픽으로 현재 위치값을 퍼블리시(전송)합니다.
            self.previous_positions = positions.copy()                      # 현재 위치값을 이전 위치값으로 저장해, 다음 루프에서 변화 감지에 사용합니다.
            if sum(positions) % 1000 == 0:                                  # 위치값의 합이 1000의 배수일 때만 아래 로그를 출력합니다. (너무 자주 로그가 찍히는 것을 방지)
                self.get_logger().info(f"Motor positions: {positions}")     # 현재 모터 위치값을 info 레벨로 로그에 남깁니다.
        
    def signal_handler(self, signum, frame):                                # Ctrl+C 등 시스템 시그널이 들어왔을 때 호출되는 함수입니다.
        self.get_logger().info("Ctrl+C detected. Shutting down...")
        
        if self.timer:                                                      # 타이머가 존재하면
            self.timer.cancel()                                             # 주기적으로 실행되던 타이머를 중지합니다.
        if self.portHandler:                                                # 포트 핸들러가 존재하면
            self.portHandler.closePort()                                    # Dynamixel 통신 포트를 닫아줍니다.
        
        rclpy.shutdown()                                                    # ROS2 시스템을 안전하게 종료합니다.
        sys.exit(0)                                                         # 프로그램을 완전히 종료합니다.

    def destroy_node(self):                                                 # ROS2 노드가 소멸될 때 호출되는 함수입니다.
        if hasattr(self, 'portHandler') and self.portHandler:               # 포트 핸들러가 존재하면
            self.portHandler.closePort()                                    # 포트를 닫아줍니다.
        super().destroy_node()                                              # 부모 클래스의 소멸자도 호출하여 ROS2 내부 리소스를 정리합니다.

def main(args=None):                                                        # 프로그램의 진입점(메인 함수)입니다.
    rclpy.init(args=args)                                                   # ROS2 시스템을 초기화합니다.
    
    try:
        master_node = MasterNode()                                          # MasterNode 객체를 생성합니다. (모든 초기화가 이 시점에 실행)
        print("Master node started (optimized for complex signal environments)!")
        print("Publishing at 20Hz with change detection...")
        print("Press Ctrl+C to exit.")
        rclpy.spin(master_node)                                             # ROS2 이벤트 루프를 돌리며, 노드가 계속 동작하도록 합니다.
    except KeyboardInterrupt:                                               # Ctrl+C 등으로 인터럽트가 발생하면
        print("\nKeyboard interrupt received.") 
    except Exception as e:                                                  # 그 외의 예외가 발생하면
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
