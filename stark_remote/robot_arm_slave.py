import rclpy                                                                    # ros2 pytion라이브러리
from rclpy.node import Node                                                     # 노드 클래스
from std_msgs.msg import Int32MultiArray                                        # 여러개의 int32를 담는 ros2 메시지 타입
from dynamixel_sdk import *                                                     # 다이나믹셀 SDK
import signal                                                                   # 시스템 신호 ctrl + c
import sys                                                                      # 시스템 종료   
import time                                                                     # 시간
import os                                                                       # os관련 표준 라이브러리
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy        # ROS2 통신 품질 설정

PORT_CON = '/dev/ttyACM0'                                                       # 포트 경로
DXL_SLAVE_IDS = [10, 11, 12, 13, 14, 15]                                        # slave 모터 id
COMMUNICATION_CHECK_SECONDS = 2.0                                               # 통신 체크 주기

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_TEMPERATURE_LIMIT = 31
ADDR_MAX_CURRENT_LIMIT = 38
ADDR_PRESENT_TEMPERATURE = 146

DEFAULT_POSITIONS = [30000, 30000, 30000, 30000, 30000, 3000]                   # 기본 위치값 설정
MOTOR_CURRENT_LIMITS = [1000, 1000, 1000, 1000, 1000, 300]                      # 모터별 개별 전류 제한 설정
MOTOR_TEMPERATURE_LIMITS = [75, 75, 75, 75, 75, 70]                             # 온도 제한값

# 오류 감지 설정
MAX_CONSECUTIVE_ERRORS = 5                                                      # 바꾸지 마셈
MAX_OVERLOAD_ERRORS = 5                                                         # 연속 오버로드 오류 시 리부팅 후 프로그램 재시작

class EnhancedSlaveNode(Node):                                                  # ros 노드 클래스를 상속받아 슬레이브 노드 정의
    def __init__(self):
        super().__init__('robot_arm_slave')
    
        qos_profile = QoSProfile(                                               # 마스터와 동일한 QoS 설정
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(                           # 'master_positions' 토픽을 구독, 메시지 수신 시 position_callback 호출
            Int32MultiArray,
            'master_positions',
            self.position_callback,
            qos_profile
        )
        
        self.dxl_ids = DXL_SLAVE_IDS                                            # 모터 id
        self.portHandler = PortHandler(PORT_CON)                                
        self.packetHandler = PacketHandler(2.0)
    
        self.torque_enabled = False                                             # 토크 상태
        self.last_message_time = None                                           # 마지막 메시지 시간
        self.message_count = 0                                                  # 메시지 카운트
        self.last_positions = DEFAULT_POSITIONS.copy()                          # 마지막 위치
        self.communication_lost = False                                         # 통신 상태
        self.shutdown_requested = False                                         # 종료 요청 여부
        
        self.motor_error_count = [0] * 6                                        # 각 모터별 오류 카운트
        self.motor_last_error_time = [0] * 6                                    # 각 모터별 마지막 오류 시간
        self.motor_recovery_attempts = [0] * 6                                  # 각 모터별 복구 시도 횟수
        self.motor_status = ['정상'] * 6                                         # 각 모터별 상태
        self.motor_health_check_timer = None                                    # 체크 타이머
        
        self.overload_error_count = [0] * 6                                     # 오버로드 오류 카운트
        self.consecutive_overload_time = [0] * 6                                # 연속 오버로드 시간
        self.last_overload_time = [0] * 6                                       # 마지막 오버로드 시간
        
        self.original_current_limits = [0] * 6                                  # 원래 전류값 저장 백업용
        self.original_temperature_limits = [0] * 6                              # 원래 온도값 백업용
        self.settings_backed_up = False                                         # 백업 여부
        
        self.initialize_port()                                                  # Dynamixel 포트 초기화 함수 호출
        self.backup_original_settings()                                         # 모터 설정값 백업
        self.initialize_motors_with_protection()                                # 보호모드 적용
        self.communication_timer = self.create_timer(COMMUNICATION_CHECK_SECONDS, self.check_communication)
        self.motor_health_check_timer = self.create_timer(1.0, self.monitor_motor_health)       # 2초마다 통신 체크, 1초마다 모터 상태 모니터링 타이머 생성
        
        signal.signal(signal.SIGINT, self.signal_handler)                       # Ctrl+C, 종료 시그널 핸들러 등록
        signal.signal(signal.SIGTERM, self.signal_handler)                      # Ctrl+C, 종료 시그널 핸들러 등록
        
        self.get_logger().info("향상된 슬레이브 노드가 포트 재연결 및 토크 활성화 기능과 함께 초기화되었습니다")
        self.get_logger().info(f"포트: {PORT_CON}")
        self.get_logger().info(f"기본 위치값: {DEFAULT_POSITIONS}")
        self.get_logger().info(f"개별 전류 제한값: {MOTOR_CURRENT_LIMITS} mA")
        self.get_logger().info(f"연속 오버로드 {MAX_OVERLOAD_ERRORS}회 시 모터 리부팅 후 프로그램 자동 재시작")
        self.get_logger().info("리부팅 후 포트 자동 재연결 및 토크 자동 활성화 기능 활성화")
        self.get_logger().info("마스터 위치 데이터를 기다리는 중")
        
    def initialize_port(self):                                                  # 포트 열기, 통신속도 설정
        """포트 초기화"""
        try:
            if not self.portHandler.openPort():
                self.get_logger().error(f"포트 {PORT_CON}를 열 수 없습니다")
                raise RuntimeError("포트 열기 실패")
            if not self.portHandler.setBaudRate(57600):
                self.get_logger().error("보드레이트 설정 실패")
                raise RuntimeError("보드레이트 설정 실패")
            self.get_logger().info(f"포트 {PORT_CON} 초기화 완료 (57600 baud)")
        except Exception as e:
            self.get_logger().error(f"포트 초기화 실패: {e}")
            raise
    
    def reconnect_port(self):                                                   # 이 함수는 Dynamixel 통신 포트를 재연결하는 역할을 합니다
        """포트 재연결"""
        self.get_logger().info("포트 재연결을 시도합니다...")
        
        try:
            if self.portHandler.is_open:                                        # 포트가 이미 열려 있는지 확인
                self.portHandler.closePort()                                    # 해당 포트를 닫음
                self.get_logger().info("기존 포트 연결을 닫았습니다")
                time.sleep(1.0)
            if not self.portHandler.openPort():                                 # 포트를 다시 열기를 시도함
                self.get_logger().error(f"포트 {PORT_CON} 재연결 실패")
                return False
            if not self.portHandler.setBaudRate(57600):                         # 포트가 열렸다면 보드레이트 재설정
                self.get_logger().error("재연결 후 보드레이트 설정 실패")
                return False
            self.get_logger().info(f"포트 {PORT_CON} 재연결 성공")
            
            connected_motors = self.verify_all_motors_connection()              # 모든 모터와의 통신이 정상적으로 되는지 확인하는 함수 호출, 연결된 모터 수를 반환받음.
            if connected_motors >= len(self.dxl_ids) - 1:                       # 연결된 모터 수가 전체 모터 수 - 1 이상이면(즉, 대부분 연결되면) 원래는 -1을 안하는게 맞음
                self.get_logger().info(f"{connected_motors}개 모터와 재연결 성공") 
                return True
            else:
                self.get_logger().warn(f"일부 모터 재연결 실패: {connected_motors}/{len(self.dxl_ids)}개만 연결됨")
                return connected_motors > 0 
        except Exception as e:
            self.get_logger().error(f"포트 재연결 중 오류: {e}")
            return False
    
    def verify_all_motors_connection(self):
        """모든 모터와의 연결 상태 확인"""
        connected_count = 0
        
        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                # 모터와 통신 테스트
                position, result, error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION
                )
                
                if result == COMM_SUCCESS:
                    connected_count += 1
                    motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 연결 확인됨, 위치 {position}")
                else:
                    motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                    self.get_logger().warn(f"모터 {dxl_id} ({motor_type}): 연결 실패")
                
                time.sleep(0.01)  # 통신 안정성
                
            except Exception as e:
                motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 연결 확인 중 오류 {e}")
        
        return connected_count
        
    def backup_original_settings(self):
        """원래 설정값 백업"""
        self.get_logger().info("원래 모터 설정값을 백업하는 중...")
        
        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                # 원래 전류 제한값 백업
                current_limit, result, error = self.packetHandler.read2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_MAX_CURRENT_LIMIT
                )
                if result == COMM_SUCCESS:
                    self.original_current_limits[i] = current_limit
                else:
                    self.original_current_limits[i] = 2047
                
                # 원래 온도 제한값 백업
                temp_limit, result, error = self.packetHandler.read1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TEMPERATURE_LIMIT
                )
                if result == COMM_SUCCESS:
                    self.original_temperature_limits[i] = temp_limit
                else:
                    self.original_temperature_limits[i] = 80
                
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f"모터 {dxl_id}: 설정값 백업 실패: {e}")
                self.original_current_limits[i] = 2047
                self.original_temperature_limits[i] = 80
        
        self.settings_backed_up = True
        self.get_logger().info("원래 설정값 백업 완료")
        
    def restore_original_settings(self):
        """원래 설정값 복원"""
        if not self.settings_backed_up:
            return
        
        self.get_logger().info("원래 모터 설정값으로 복원하는 중...")
        
        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                time.sleep(0.01)
                
                self.packetHandler.write2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_MAX_CURRENT_LIMIT, self.original_current_limits[i]
                )
                
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TEMPERATURE_LIMIT, self.original_temperature_limits[i]
                )
                
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f"모터 {dxl_id}: 설정값 복원 실패: {e}")
    
    def restart_program(self):
        """프로그램 자동 재시작"""
        self.get_logger().info("프로그램 자동 재시작을 준비합니다...")
        
        # 타이머 정리
        if hasattr(self, 'communication_timer'):
            self.communication_timer.cancel()
        if hasattr(self, 'motor_health_check_timer'):
            self.motor_health_check_timer.cancel()
        
        # 토크 비활성화하지 않음 (연속 동작을 위해)
        self.get_logger().info("토크는 유지한 채로 프로그램을 재시작합니다...")
        
        # 원래 설정값 복원하지 않음 (새로운 설정 유지)
        self.get_logger().info("현재 보호 설정을 유지한 채로 재시작합니다...")
        
        # 포트 닫기
        if self.portHandler and self.portHandler.is_open:
            self.portHandler.closePort()
            self.get_logger().info("포트 연결 종료")
        
        # ROS 정리
        rclpy.shutdown()
        
        self.get_logger().info("3초 후 프로그램을 재시작합니다...")
        time.sleep(3)
        
        # 현재 스크립트 파일명과 인수 가져오기
        python_executable = sys.executable
        script_path = os.path.abspath(__file__)
        
        self.get_logger().info(f"재시작 실행: {python_executable} {script_path}")
        print("="*50)
        print("프로그램 자동 재시작 중...")
        print("="*50)
        
        # 프로그램 재시작 (현재 프로세스를 새로운 프로세스로 교체)
        os.execv(python_executable, [python_executable, script_path])
        
    def initialize_motors_with_protection(self):
        """모터 초기화 및 개별 보호 설정"""
        self.get_logger().info("개별 보호 설정으로 모터를 초기화하는 중...")
        
        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                time.sleep(0.01)
                
                # 개별 전류 제한 설정
                individual_current_limit = MOTOR_CURRENT_LIMITS[i]
                result, error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, dxl_id, ADDR_MAX_CURRENT_LIMIT, individual_current_limit
                )
                if result == COMM_SUCCESS:
                    motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 전류 제한값을 {individual_current_limit} mA로 설정")
                
                # 개별 온도 제한 설정
                individual_temp_limit = MOTOR_TEMPERATURE_LIMITS[i]
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_TEMPERATURE_LIMIT, individual_temp_limit
                )
                
                self.check_motor_error_status(dxl_id, i)
                self.motor_status[i] = '정상'
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f"모터 {dxl_id} 초기화 실패: {e}")
                self.motor_status[i] = '오류'
    
    def check_motor_error_status(self, dxl_id, motor_index):
        """모터 하드웨어 오류 상태 확인 및 오버로드 처리"""
        error_status, result, error = self.packetHandler.read1ByteTxRx(
            self.portHandler, dxl_id, ADDR_HARDWARE_ERROR_STATUS
        )
        
        if result != COMM_SUCCESS:
            return False
        
        error_types = []
        overload_detected = False
        
        if error_status & 0x01:
            error_types.append("입력 전압 오류")
        if error_status & 0x02:
            error_types.append("홀 센서 오류")
        if error_status & 0x04:
            error_types.append("전기적 충격 오류")
        if error_status & 0x08:
            error_types.append("엔코더 오류")
        if error_status & 0x10:
            error_types.append("전자적 보호 오류")
        if error_status & 0x20:
            error_types.append("오버로드")
            overload_detected = True
        if error_status & 0x40:
            error_types.append("과열")
        
        if error_types:
            motor_type = "그리퍼" if motor_index == 5 else f"관절 {motor_index+1}"
            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 하드웨어 오류 감지: {error_types}")
            
            # 오버로드 오류 처리
            if overload_detected:
                current_time = time.time()
                self.overload_error_count[motor_index] += 1
                self.last_overload_time[motor_index] = current_time
                
                # 연속 오버로드 체크
                if (current_time - self.consecutive_overload_time[motor_index]) < 3.0:
                    # 3초 이내 연속 오버로드
                    if self.overload_error_count[motor_index] >= MAX_OVERLOAD_ERRORS:
                        self.get_logger().error(f"모터 {dxl_id} ({motor_type}): {MAX_OVERLOAD_ERRORS}회 연속 오버로드 오류 감지!")
                        self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 후 프로그램 자동 재시작을 시작합니다...")
                        
                        # 해당 모터 리부팅 후 프로그램 재시작
                        if self.reboot_motor_and_restart_program(dxl_id, motor_index):
                            # 이 부분은 실행되지 않음 (프로그램이 재시작되기 때문)
                            pass
                        else:
                            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 실패")
                else:
                    # 새로운 오버로드 시퀀스 시작
                    self.consecutive_overload_time[motor_index] = current_time
                    self.overload_error_count[motor_index] = 1
            
            return True
        
        return False
    
    def reboot_motor_and_restart_program(self, dxl_id, motor_index):
        """오버로드 오류로 인한 모터 리부팅 후 프로그램 재시작"""
        motor_type = "그리퍼" if motor_index == 5 else f"관절 {motor_index+1}"
        self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 오버로드 복구를 위한 리부팅 시작...")
        
        self.motor_status[motor_index] = '복구중'
        
        try:
            # 1. 토크 비활성화
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.2)
            
            # 2. 리부팅 실행
            result, error = self.packetHandler.reboot(self.portHandler, dxl_id)
            
            if result != COMM_SUCCESS:
                self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 명령 실패")
                self.motor_status[motor_index] = '오류'
                return False
            
            # 3. 리부팅 완료 대기
            self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 명령 전송됨, 복구 대기중...")
            time.sleep(3.0)
            
            # 4. 연결 확인
            for attempt in range(5):
                position, result, error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION
                )
                
                if result == COMM_SUCCESS:
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 성공, 현재 위치: {position}")
                    
                    # 5. 프로그램 자동 재시작
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 완료 - 프로그램을 자동 재시작합니다")
                    self.restart_program()  # 이 함수에서 프로그램이 종료되고 재시작됨
                    return True  # 이 라인은 실행되지 않음
                
                time.sleep(0.7)
            
            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 후 응답 없음")
            self.motor_status[motor_index] = '실패'
            return False
            
        except Exception as e:
            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 중 예외 발생: {e}")
            self.motor_status[motor_index] = '오류'
            return False
    
    def reboot_motor(self, dxl_id, motor_index):
        """일반적인 모터 리부팅 (통신 오류용) + 포트 재연결 + 토크 활성화"""
        motor_type = "그리퍼" if motor_index == 5 else f"관절 {motor_index+1}"
        self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 통신 오류 복구를 위한 리부팅 시작...")
        
        self.motor_status[motor_index] = '복구중'
        
        try:
            # 1. 토크 비활성화
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.1)
            
            # 2. 리부팅 실행
            result, error = self.packetHandler.reboot(self.portHandler, dxl_id)
            
            if result != COMM_SUCCESS:
                self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 명령 실패")
                self.motor_status[motor_index] = '오류'
                return False
            
            # 3. 리부팅 완료 대기
            self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 명령 전송됨, 복구 대기중...")
            time.sleep(2.0)
            
            # 4. 연결 확인 및 포트 재연결
            connection_successful = False
            for attempt in range(5):
                position, result, error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_PRESENT_POSITION
                )
                
                if result == COMM_SUCCESS:
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅 성공, 현재 위치: {position}")
                    connection_successful = True
                    break
                else:
                    self.get_logger().warn(f"모터 {dxl_id} ({motor_type}): 리부팅 후 연결 실패 (시도 {attempt+1}/5)")
                    
                    # 첫 번째 실패 후 포트 재연결 시도
                    if attempt == 1:
                        self.get_logger().info("포트 재연결을 시도합니다...")
                        if self.reconnect_port():
                            self.get_logger().info("포트 재연결 성공")
                            # 모든 모터의 보호 설정 재적용
                            self.reapply_all_motor_settings()
                            # 모든 모터의 토크 재활성화
                            self.reactivate_all_motor_torque()
                        else:
                            self.get_logger().error("포트 재연결 실패")
                    
                    time.sleep(0.5)
            
            if connection_successful:
                # 5. 개별 모터 보호 설정 재적용
                self.setup_individual_motor_protection(dxl_id, motor_index)
                
                # 6. 해당 모터 토크 활성화 *** 중요! ***
                result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result == COMM_SUCCESS:
                    self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 토크 재활성화 완료")
                else:
                    self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 토크 재활성화 실패")
                
                # 7. 상태 복구
                self.motor_status[motor_index] = '정상'
                self.motor_error_count[motor_index] = 0
                self.motor_recovery_attempts[motor_index] += 1
                
                self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 리부팅, 포트 재연결, 토크 활성화 완료")
                return True
            else:
                self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 후 연결 복구 실패")
                self.motor_status[motor_index] = '실패'
                return False
            
        except Exception as e:
            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 리부팅 중 예외 발생: {e}")
            self.motor_status[motor_index] = '오류'
            return False
    
    def reapply_all_motor_settings(self):
        """모든 모터의 보호 설정 재적용"""
        self.get_logger().info("포트 재연결 후 모든 모터의 보호 설정을 재적용합니다...")
        
        for i, dxl_id in enumerate(self.dxl_ids):
            if self.motor_status[i] != '실패':
                try:
                    self.setup_individual_motor_protection(dxl_id, i)
                    time.sleep(0.01)
                except Exception as e:
                    motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                    self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 설정 재적용 실패: {e}")
        
        self.get_logger().info("모든 모터 보호 설정 재적용 완료")
    
    def reactivate_all_motor_torque(self):
        """모든 정상 모터의 토크 재활성화"""
        self.get_logger().info("포트 재연결 후 모든 정상 모터의 토크를 재활성화합니다...")
        
        success_count = 0
        failed_motors = []
        
        for i, dxl_id in enumerate(self.dxl_ids):
            if self.motor_status[i] in ['정상', '복구중']:  # 실패하지 않은 모터들만
                try:
                    result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
                    if result == COMM_SUCCESS:
                        success_count += 1
                        motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                        self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 토크 재활성화 성공")
                    else:
                        failed_motors.append(dxl_id)
                        motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                        self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 토크 재활성화 실패")
                    
                    time.sleep(0.01)
                    
                except Exception as e:
                    motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                    self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 토크 재활성화 중 오류: {e}")
                    failed_motors.append(dxl_id)
        
        if success_count > 0:
            self.torque_enabled = True
            self.get_logger().info(f"{success_count}개 모터 토크 재활성화 완료")
        
        if failed_motors:
            self.get_logger().warn(f"토크 재활성화 실패 모터들: {failed_motors}")
    
    def setup_individual_motor_protection(self, dxl_id, motor_index):
        """리부팅 후 개별 모터 보호 설정 재적용"""
        motor_type = "그리퍼" if motor_index == 5 else f"관절 {motor_index+1}"
        
        try:
            # 토크 비활성화 (설정 변경을 위해)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.01)
            
            # 개별 전류 제한 재설정
            individual_current_limit = MOTOR_CURRENT_LIMITS[motor_index]
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_MAX_CURRENT_LIMIT, individual_current_limit
            )
            if result == COMM_SUCCESS:
                self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 전류 제한값을 {individual_current_limit} mA로 복원")
            
            # 개별 온도 제한 재설정
            individual_temp_limit = MOTOR_TEMPERATURE_LIMITS[motor_index]
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TEMPERATURE_LIMIT, individual_temp_limit
            )
            if result == COMM_SUCCESS:
                self.get_logger().info(f"모터 {dxl_id} ({motor_type}): 온도 제한값을 {individual_temp_limit} °C로 복원")
            
            # 주의: 여기서는 토크를 활성화하지 않음 (호출하는 함수에서 처리)
            
        except Exception as e:
            self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 개별 보호 설정 재적용 실패: {e}")
    
    def monitor_motor_health(self):
        """모터 상태 모니터링"""
        current_time = time.time()
        
        for i, dxl_id in enumerate(self.dxl_ids):
            if self.motor_status[i] == '복구중':
                continue
                
            individual_temp_limit = MOTOR_TEMPERATURE_LIMITS[i]
            temp, result, error = self.packetHandler.read1ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_TEMPERATURE
            )
            
            motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
            
            if result == COMM_SUCCESS and temp > individual_temp_limit - 5:
                self.get_logger().warn(f"모터 {dxl_id} ({motor_type}): 고온 경고: {temp}°C (제한값: {individual_temp_limit}°C)")
            
            if self.check_motor_error_status(dxl_id, i):
                self.motor_error_count[i] += 1
                self.motor_last_error_time[i] = current_time
                
                if self.motor_error_count[i] >= MAX_CONSECUTIVE_ERRORS:
                    self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 임계 오류 횟수 도달")
                    
                    if self.motor_recovery_attempts[i] < 3:
                        self.reboot_motor(dxl_id, i)  # 포트 재연결 및 토크 활성화 포함된 리부팅
                    else:
                        self.motor_status[i] = '실패'
                        self.get_logger().error(f"모터 {dxl_id} ({motor_type}): 3회 복구 시도 후에도 실패")
    
    def position_callback(self, msg):
        """마스터에서 받은 위치 데이터 처리"""
        self.last_message_time = time.time()
        self.message_count += 1
        
        if self.communication_lost:
            self.communication_lost = False
            self.get_logger().info("통신 복구됨 - 정상 동작 재개")
        
        if len(msg.data) != 6:
            self.get_logger().warn(f"잘못된 데이터 길이: {len(msg.data)}")
            return
        
        if not self.torque_enabled:
            self.enable_torque()
        
        self.last_positions = list(msg.data)
        self.mirror_positions_with_error_handling(self.last_positions)
        
        if self.message_count % 200 == 0:
            self.get_logger().info(f"통신 상태 양호: {self.message_count}개 메시지 수신")
            self.log_motor_status()
            
    def mirror_positions_with_error_handling(self, master_positions):
        """오류 처리가 포함된 위치 미러링"""
        try:
            success_count = 0
            failed_motors = []
            
            for i, dxl_id in enumerate(self.dxl_ids):
                if self.motor_status[i] in ['실패', '복구중']:
                    continue
                
                target_position = master_positions[i]
                result, error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_GOAL_POSITION, target_position
                )
                
                if result != COMM_SUCCESS:
                    failed_motors.append(dxl_id)
                    self.motor_error_count[i] += 1
                    
                    if self.motor_error_count[i] >= 3:
                        motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                        self.get_logger().warn(f"모터 {dxl_id} ({motor_type}): 다중 통신 실패 - 리부팅 예약")
                        self.reboot_motor(dxl_id, i)  # 포트 재연결 및 토크 활성화 포함
                else:
                    success_count += 1
                    self.motor_error_count[i] = max(0, self.motor_error_count[i] - 1)
            
            if len(failed_motors) > len(self.dxl_ids) // 2:
                self.get_logger().warn(f"위치 설정 실패 모터들: {failed_motors}")
                    
        except Exception as e:
            self.get_logger().error(f"미러링 오류: {e}")
    
    def log_motor_status(self):
        """모터 상태 로깅"""
        status_summary = {}
        for status in self.motor_status:
            status_summary[status] = status_summary.get(status, 0) + 1
        
        self.get_logger().info(f"모터 상태 요약: {status_summary}")
        
        for i, (dxl_id, status) in enumerate(zip(self.dxl_ids, self.motor_status)):
            motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
            if status != '정상' or self.overload_error_count[i] > 0:
                self.get_logger().info(
                    f"모터 {dxl_id} ({motor_type}): 상태={status}, "
                    f"오버로드 횟수={self.overload_error_count[i]}, "
                    f"오류 횟수={self.motor_error_count[i]}, "
                    f"복구 시도={self.motor_recovery_attempts[i]}"
                )
    
    def check_communication(self):
        """통신 상태 확인"""
        if self.last_message_time is None:
            return
            
        elapsed_time = time.time() - self.last_message_time
        
        if elapsed_time > COMMUNICATION_CHECK_SECONDS and not self.communication_lost:
            self.communication_lost = True
            self.get_logger().warn(f"통신 끊어짐 (경과 시간: {elapsed_time:.1f}초)")
            self.get_logger().info("토크 유지됨 - 마지막 위치 유지")
            
        elif elapsed_time > 10.0 and int(elapsed_time) % 10 == 0:
            self.get_logger().info(f"통신 끊어진 지 {elapsed_time:.0f}초 - 위치 유지 중")
    
    def enable_torque(self):
        """토크 활성화 (개선된 버전)"""
        success_count = 0
        failed_motors = []
        
        for i, dxl_id in enumerate(self.dxl_ids):
            if self.motor_status[i] == '실패':
                continue
                
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            if result == COMM_SUCCESS:
                success_count += 1
            else:
                failed_motors.append(dxl_id)
                motor_type = "그리퍼" if i == 5 else f"관절 {i+1}"
                self.get_logger().warn(f"모터 {dxl_id} ({motor_type}): 토크 활성화 실패")
                
        if success_count > 0:
            self.torque_enabled = True
            self.get_logger().info(f"{success_count}개 모터 토크 활성화됨")
        
        if failed_motors:
            self.get_logger().warn(f"토크 활성화 실패 모터들: {failed_motors}")
        
        return success_count > 0
        
    def disable_torque(self):
        """토크 비활성화 (수동 종료시에만)"""
        if not self.torque_enabled:
            return
            
        self.get_logger().info("수동 종료 - 모든 모터 토크 비활성화 중...")
        
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                
        self.torque_enabled = False
            
    def signal_handler(self, signum, frame):
        """안전한 수동 종료 처리"""
        self.shutdown_requested = True
        
        self.get_logger().info("수동 종료 감지됨 (Ctrl+C) - 안전하게 종료하는 중...")
        
        if hasattr(self, 'communication_timer'):
            self.communication_timer.cancel()
        if hasattr(self, 'motor_health_check_timer'):
            self.motor_health_check_timer.cancel()
        
        # 수동 종료시에만 토크 끄고 설정 복원
        self.disable_torque()
        self.restore_original_settings()
        
        if self.portHandler and self.portHandler.is_open:
            self.portHandler.closePort()
            self.get_logger().info("포트 연결 종료")
        
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        """노드 소멸자"""
        if not self.shutdown_requested:
            self.disable_torque()
            self.restore_original_settings()
        
        if hasattr(self, 'portHandler') and self.portHandler and self.portHandler.is_open:
            self.portHandler.closePort()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        slave_node = EnhancedSlaveNode()
        print("향상된 로봇 팔 슬레이브 노드 시작됨 (완전 자동 복구 시스템)")
        print(f"포트: {PORT_CON}")
        print("주요 기능:")
        print("  - 오버로드 발생 시 해당 모터 리부팅 후 프로그램 재시작")
        print("  - 통신 오류 발생 시 해당 모터 리부팅 후 포트 자동 재연결")
        print("  - 리부팅 후 보호 설정 자동 재적용")
        print("  - 리부팅 후 토크 자동 활성화")
        print("  - 모든 모터 연결 상태 실시간 확인")
        print("  - 토크 유지한 채로 동작 계속")
        
        rclpy.spin(slave_node)
        
    except KeyboardInterrupt:
        print("키보드 인터럽트 수신됨")
    except Exception as e:
        print(f"오류 발생: {e}")
        try:
            if 'slave_node' in locals():
                slave_node.disable_torque()
                slave_node.restore_original_settings()
        except:
            pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("프로그램 종료됨")

if __name__ == '__main__':
    main()
