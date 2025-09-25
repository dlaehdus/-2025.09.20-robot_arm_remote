#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from geometry_msgs.msg import Twist
import math

import time
from Class_BLDC import BLDC_MotorControl, CMD_MODE, FEEDBACK_MODE, FEEDBACK_NUM_TO_STR, BAUDHEX_TO_BPS, BAUDBPS_TO_HEX

# === 당신이 만든 RS485 BLDC 클래스 임포트 ===
# from blc200 import BLC200  # (또는 BLDC_MotorControl) 파일 경로에 맞게 수정

# 모터 관련
VEL_MAX             = 1000                  # 모터 최대 속도
VEL_MIN             = -900                  # 모터 최소 속도
VEL_MAPING_RATIO    = 1.5                   # 조이스틱-모터 간 속도 매핑 상수

TRACK               = 0.347                               # 좌우 바퀴 간격 [m]
WHEEL_RADIUS        = 0.05035                      # 바퀴 반지름 [m]
# 변환 상수
RPM_PER_MPS         = 60.0 / (2.0 * math.pi * WHEEL_RADIUS)

UNIT_TENTHS         = 10

class BLC200SpeedNode(Node):
    def __init__(self,
                 left_port="/dev/ttyACM3",
                 right_port="/dev/ttyACM1",
                 baud=9600):
        super().__init__("blc200_speed_node")

        # QoS: Reliable + depth 10 (속도명령은 최신 것만 쓰지만, 신뢰성 확보)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Int32MultiArray, "controller", self.callback, qos)

        # === RS-485 드라이버 초기화 ===
        # BLDC_MotorControl 버전:
        self.left  = BLDC_MotorControl(port_dir=left_port, baudrate_bps=baud, motor_cw_is_moving_forward=False)
        self.right = BLDC_MotorControl(port_dir=right_port, baudrate_bps=baud, motor_cw_is_moving_forward=True)

        # 20Hz(0.05s) 주기 상태표시(선택)
        self.timer = self.create_timer(0.02, self.tick)  # 50ms 주기
        self.last_cmd = (0, 0)
        self.get_logger().info("BLC200 speed subscriber started (0.05 s)")
    
    # 안전을 위해 한계 클램프
    @staticmethod
    def clamp(v, lo, hi): return max(lo, min(hi, v))

    def callback(self, msg: Int32MultiArray | Twist):
        if type(msg) == Twist:
            v = float(msg.linear.x)          # m/s
            omega = float(msg.angular.z)     # rad/s

            # 차동 구동 선속도 계산
            v_r = v + (omega * TRACK / 2.0)  # m/s
            v_l = v - (omega * TRACK / 2.0)  # m/s

            # m/s -> RPM
            rpm_r = v_r * RPM_PER_MPS
            rpm_l = v_l * RPM_PER_MPS

            right_forward = (rpm_r >= 0.0)
            left_forward = (rpm_r >= 0.0)

            # fr = int(-rpm_r)
            right_rpm = abs(int(rpm_r * UNIT_TENTHS))
            # fl = int(rpm_l)
            left_rpm = abs(int(rpm_l * UNIT_TENTHS))

            self.left.set_speed_per_time(GoForward_true=left_forward,  target_Speed_RPM_tenths=left_rpm,  Accel_time_0p1s=1)
            self.right.set_speed_per_time(GoForward_true=right_forward, target_Speed_RPM_tenths=right_rpm, Accel_time_0p1s=1)

        elif type(msg) == Int32MultiArray:
            # 기대: data[0]=Left, data[1]=Right (controller.py가 Int32MultiArray를 발행 중) :contentReference[oaicite:4]{index=4}
            if not msg.data or len(msg.data) < 2:
                return
            left_vel, right_vel = int(msg.data[0]), int(msg.data[1])
            self.last_cmd = (left_vel, right_vel)

            # === 속도 스케일링/방향 정의 ===
            # 예시) 컨트롤러가 -330~+330 범위라면 이를 RPM으로 매핑
            left_vel  = self.clamp(left_vel, VEL_MIN, VEL_MAX)
            right_vel = self.clamp(right_vel, VEL_MIN, VEL_MAX)

            # 단순 매핑 예: 330 -> 3000RPM 정격의 1/10 등, 실제 기어/정격에 맞춰 조정
            # 여기서는 예시로 1:1 매핑을 가정 (필요하면 scale를 바꾸세요)
            left_rpm  = round(abs(left_vel) * VEL_MAPING_RATIO)     # 예: 330 -> 3300 (과하면 정격으로 클램프)
            right_rpm = round(abs(right_vel) * VEL_MAPING_RATIO)    # 확인 결과, 10 곱하는 것은 조이스틱 값에 대한 민감도 정도로 보면 됨. 값이 클수록 조이스틱을 조금 움직여도 속도가 매우 큼.

            left_rpm  = self.clamp(left_rpm,  0, 3000)   # 정격 3000RPM 가정
            right_rpm = self.clamp(right_rpm, 0, 3000)

            left_forward  = (left_vel  >= 0)# * (2 * int(self.left.control_Rside) - 1)
            right_forward = (right_vel >= 0)# * (2 * int(self.right.control_Rside) - 1)

            # BLDC_MotorControl 버전:
            self.left.set_speed_per_time(GoForward_true=left_forward,  target_Speed_RPM_tenths=left_rpm,  Accel_time_0p1s=1)
            self.right.set_speed_per_time(GoForward_true=right_forward, target_Speed_RPM_tenths=right_rpm, Accel_time_0p1s=1)
        else:
            pass

    def tick(self):
        # 상태 모니터링/워치독 용도 (선택)
        lv, rv = self.last_cmd
        NOW_L_vel = self.left.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["speed"])
        NOW_R_vel = self.right.ask_feedback(feedback_mode_chosen=FEEDBACK_MODE["speed"])

        if NOW_L_vel[2] == 0x00: NOW_L_dir = "CCW"
        elif NOW_L_vel[2] == 0x01: NOW_L_dir = "CW"
        else: NOW_L_dir = "Error"

        if self.last_cmd[0] < 0: NOW_L_sign = "-"
        else: NOW_L_sign = ""
        
        if NOW_R_vel[2] == 0x00: NOW_R_dir = "CCW"
        elif NOW_R_vel[2] == 0x01: NOW_R_dir = "CW"
        else: NOW_R_dir = "Error"

        if self.last_cmd[1] < 0: NOW_R_sign = "-"
        else: NOW_R_sign = ""

        self.get_logger().debug(f"last cmd L:{lv} R:{rv}")
        self.get_logger().info(f"\n[Target Vel]\tleft: {lv},\tright: {rv}\n"
                               f"[Present Vel]\t"
                               f"Left: {NOW_L_sign}{NOW_L_vel[3]}({NOW_L_dir}),\t"
                               f"Right: {NOW_R_sign}{NOW_R_vel[3]}({NOW_R_dir})")

def main():
    rclpy.init()
    node = BLC200SpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # try:
        #     node.left.Brake(); node.right.Brake()
        #     node.left.con_ser.close(); node.right.con_ser.close()
        # except Exception:
        #     pass
        for i in range(5):
            print(f"Turn off now. {i+1} time(s).")
            node.left.set_control_onoff(control_enable=False)
            node.right.set_control_onoff(control_enable=False)
            node.left.set_control_onoff(control_enable=True)
            node.right.set_control_onoff(control_enable=True)
            time.sleep(0.05)
        for _ in range(5):
            node.left.con_ser.close()
            node.right.con_ser.close()
        print("\nPort closed\n")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()