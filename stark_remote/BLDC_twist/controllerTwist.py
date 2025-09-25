#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import PortHandler, PacketHandler
import time
import math

# === 기존 상수/주소는 그대로 ===
PORT_CON        = '/dev/ttyACM5'

DI_ID_LEFT      = 21
DI_ID_RIGHT     = 22
GOAL_CURRENT    = 10
READ_MIN_POS    = 1600
READ_MAX_POS    = 2700

DI_MIN_POS      = 1500
ST_MIN_POS      = 2174
MIN_ZERO_POS    = 2200
FIRST_POS       = 2275
MAX_ZERO_POS    = 2300
ST_MAX_POS      = 2376
DI_MAX_POS      = 2800

# 모터 원 단위속도 범위(기준치)
VEL_MAX         = 1000
VEL_MIN         = -900

def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

class ControllerTwist(Node):
    """
    컨트롤러(다이나믹셀 포지션) -> 좌/우 '원 단위' 속도 -> [스케일] m/s 변환 -> Twist(/cmd_vel_in) 발행
    이 노드는 바퀴 제어를 하지 않고, 오직 '사용자 의도 속도'를 생성합니다.
    실제 바퀴 구동은 /cmd_vel을 구독하는 드라이브 노드가 담당합니다.
    """
    def __init__(self):
        super().__init__('controller_twist')

        # ---- 파라미터 ----
        self.declare_parameter('cmd_out_topic', '/cmd_vel_in')  # 자세 보정 노드 입력
        self.declare_parameter('track_width', 0.40)             # 바퀴 간 거리[m]
        self.declare_parameter('left_vel_scale',  0.001)        # 모터단위 -> m/s
        self.declare_parameter('right_vel_scale', 0.001)        # 모터단위 -> m/s
        self.declare_parameter('max_lin', 1.2)                  # m/s
        self.declare_parameter('max_ang', 1.5)                  # rad/s
        self.declare_parameter('publish_hz', 50.0)              # 발행 주기

        gp = self.get_parameter
        self.cmd_out_topic = gp('cmd_out_topic').value
        self.track_width   = float(gp('track_width').value)
        self.l_scale       = float(gp('left_vel_scale').value)
        self.r_scale       = float(gp('right_vel_scale').value)
        self.max_lin       = float(gp('max_lin').value)
        self.max_ang       = float(gp('max_ang').value)
        hz = float(gp('publish_hz').value)
        period = 1.0 / max(1e-3, hz)

        # ---- 퍼블리셔 ----
        self.pub = self.create_publisher(Twist, self.cmd_out_topic, 10)

        # ---- 다이나믹셀 I/O (기존과 동일) ----
        self.dxl_ids = [DI_ID_LEFT, DI_ID_RIGHT]
        self.portHandler = PortHandler(PORT_CON)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(57600):
            self.get_logger().error("보드레이트 설정 실패.")
            raise RuntimeError("Failed to set baudrate")

        # 우측 회전 방향 반전(기존 코드 유지)
        self.packetHandler.write1ByteTxRx(self.portHandler, DI_ID_RIGHT, 10, 1)

        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 48, READ_MAX_POS)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 52, READ_MIN_POS)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 116, FIRST_POS)
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102, GOAL_CURRENT)

        time.sleep(0.5)

        self.timer = self.create_timer(period, self.control_loop)
        self.get_logger().info(f"[INFO] ControllerTwist started -> {self.cmd_out_topic}")

    # --- 기존 로직 그대로: 포지션 읽기 & 포지션→원단위 속도 맵핑 ---
    def read_position(self, dxl_id):
        position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
        position = position if position < (1 << 31) else position - (1 << 32)
        return position

    def map_position_to_velocity(self, pos: int) -> int:
        if pos <= ST_MIN_POS:
            if pos <= DI_MIN_POS:
                return VEL_MIN
            return int(-1 - (ST_MIN_POS - pos) * ((VEL_MAX - 1) / (ST_MIN_POS - DI_MIN_POS)))
        elif MIN_ZERO_POS <= pos <= MAX_ZERO_POS:
            return 0
        elif pos >= ST_MAX_POS:
            if pos >= DI_MAX_POS:
                return VEL_MAX
            return int(1 + (pos - ST_MAX_POS) * ((VEL_MAX - 1) / (DI_MAX_POS - ST_MAX_POS)))
        return 0

    def control_loop(self):
        # 좌/우 원단위 속도 계산
        v_raw = []
        for dxl_id in self.dxl_ids:
            pos = self.read_position(dxl_id)
            if pos > DI_MAX_POS or pos < DI_MIN_POS:
                pos = FIRST_POS
            elif pos > READ_MAX_POS:
                pos = READ_MAX_POS
            elif pos < READ_MIN_POS:
                pos = READ_MIN_POS
            v_raw.append(self.map_position_to_velocity(pos))  # int 단위 속도

        # 스케일링 -> m/s
        v_l = float(v_raw[0]) * self.l_scale
        v_r = float(v_raw[1]) * self.r_scale

        # 차동구동 역기구학: v, omega
        track = max(1e-6, self.track_width)
        v_lin = 0.5 * (v_r + v_l)
        v_ang = (v_r - v_l) / track

        # 안전 제한
        v_lin = clamp(v_lin, -self.max_lin, self.max_lin)
        v_ang = clamp(v_ang, -self.max_ang, self.max_ang)

        # Twist 발행 (/cmd_vel_in)
        t = Twist()
        t.linear.x  = v_lin
        t.angular.z = v_ang
        self.pub.publish(t)

        # 로그(선택)
        self.get_logger().info(f"L_raw:{v_raw[0]:4d}, R_raw:{v_raw[1]:4d} | v={v_lin:.3f} m/s, w={v_ang:.3f} rad/s")

    def __del__(self):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
        self.portHandler.closePort()

def main():
    rclpy.init()
    node = ControllerTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
