#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math    
import time     
import serial    
import rclpy     
from rclpy.node import Node  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header      
from sensor_msgs.msg import Imu     

SERIAL_PORT = '/dev/usb-right-bottom'     
                               
BAUDRATE    = 115200              
FRAME_ID    = 'imu_link'        

# 공분산 행렬: 센서 측정값의 불확실성
# 지금은 대각 성분만 간단히 설정하여 축 간의 오차 상관관계는 없다고 가정
# 값이 작을수록 해당 측정값을 더 신뢰한다는 의미
COV_ORIENT = [0.0001, 0, 0,  0, 0.0001, 0,  0, 0, 0.0001] # 방향(Orientation) 공분산

COV_GYRO   = [0.0001, 0, 0,  0, 0.0001, 0,  0, 0, 0.0001] # 각속도(Angular Velocity) 공분산

COV_ACCEL  = [0.025,  0, 0,  0, 0.025,  0,  0, 0, 0.025]  # 선형 가속도(Linear Acceleration) 공분산

G_TO_MS2    = 9.80665              # 중력가속도
DEG_TO_RAD  = math.pi / 180.0      # 각도(degree)를 라디안(radian)으로 변환하기 위한 상수
WT_PKT_LEN  = 11                   # WitMotion 센서의 표준 패킷 길이 (11바이트)
ACC_ID, GYRO_ID, ANGLE_ID = 0x51, 0x52, 0x53  # 각 데이터 타입(가속도, 각속도, 각도)을 식별하는 ID

# Helper
def le_i16(lo: int, hi: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v

def checksum_ok(pkt: bytes) -> bool:
    return (sum(pkt[:10]) & 0xFF) == pkt[10]

def rpy_deg_to_quat(roll_d: float, pitch_d: float, yaw_d: float):
    r = roll_d * DEG_TO_RAD * 0.5
    p = pitch_d * DEG_TO_RAD * 0.5
    y = yaw_d  * DEG_TO_RAD * 0.5
    
    # 오일러 각 -> 쿼터니언 변환 공식
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yy= cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return w, x, yy, z

# ==============================================================================
# ROS 2 노드 클래스
# ==============================================================================

class WT901CNode(Node):
    def __init__(self):
        super().__init__('wt901c_imu_node')

 
        # '/imu/data' 토픽 발행
        self.pub = self.create_publisher(Imu, '/imu_data', 10)

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        self.buf = bytearray()
        time.sleep(0.1) # 포트가 안정적으로 열릴 때까지 잠시 대기
        self.ser.reset_input_buffer()

        self.acc = [0.0, 0.0, 0.0]       # 선형 가속도 (m/s^2)
        self.gyr = [0.0, 0.0, 0.0]       # 각속도 (rad/s)
        self.euler_deg = [0.0, 0.0, 0.0] # 오일러 각도 (roll, pitch, yaw in degrees)
        self.have_acc = self.have_gyro = self.have_ang = False


        self.timer = self.create_timer(0.02, self.poll)

        self.get_logger().info(f'[IMU] 시리얼 포트 {SERIAL_PORT} @ {BAUDRATE}bps, frame_id={FRAME_ID}')

    def poll(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buf += data
                self._parse_buffer()
        except Exception as e:
            self.get_logger().error(f'[IMU] 시리얼 읽기 오류: {e}')

    def _parse_buffer(self):
        b = self.buf
        while True:
            idx = b.find(b'\x55')
            if idx < 0:
                if len(b) > 2048:
                    del b[:-1]
                break
            if idx > 0:
                del b[:idx]

            if len(b) < WT_PKT_LEN:
                break

            pkt = bytes(b[:WT_PKT_LEN])
            if not checksum_ok(pkt):
                del b[0:1]
                continue

            del b[:WT_PKT_LEN]
            self._parse_packet(pkt)

            if self.have_acc and self.have_gyro and self.have_ang:
                self.publish()
                self.have_acc = self.have_gyro = self.have_ang = False

    def _parse_packet(self, pkt: bytes):
        typ = pkt[1]
        d0,d1,d2,d3,d4,d5,d6,d7 = pkt[2:10]
        x = le_i16(d0, d1)
        y = le_i16(d2, d3)
        z = le_i16(d4, d5)


        if typ == ACC_ID: # 가속도 데이터
            self.acc[0] = (x / 32768.0 * 16.0) * G_TO_MS2
            self.acc[1] = (y / 32768.0 * 16.0) * G_TO_MS2
            self.acc[2] = (z / 32768.0 * 16.0) * G_TO_MS2
            self.have_acc = True

        elif typ == GYRO_ID: # 각속도 데이터
            self.gyr[0] = (x / 32768.0 * 2000.0) * DEG_TO_RAD
            self.gyr[1] = (y / 32768.0 * 2000.0) * DEG_TO_RAD
            self.gyr[2] = (z / 32768.0 * 2000.0) * DEG_TO_RAD
            self.have_gyro = True

        elif typ == ANGLE_ID: # 각도 데이터
            self.euler_deg[0] = x / 32768.0 * 180.0 # Roll
            self.euler_deg[1] = y / 32768.0 * 180.0 # Pitch
            self.euler_deg[2] = z / 32768.0 * 180.0 # Yaw
            self.have_ang = True

    def publish(self):
        now = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header = Header(stamp=now, frame_id=FRAME_ID)

        w,x,y,z = rpy_deg_to_quat(*self.euler_deg)
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation_covariance = COV_ORIENT

        msg.angular_velocity.x = self.gyr[0]
        msg.angular_velocity.y = self.gyr[1]
        msg.angular_velocity.z = self.gyr[2]
        msg.angular_velocity_covariance = COV_GYRO

        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.linear_acceleration_covariance = COV_ACCEL

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = WT901CNode()
    try:

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()