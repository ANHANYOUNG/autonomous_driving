#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

SERIAL_PORT = '/dev/usb-right-bottom'
BAUDRATE = 115200
FRAME_ID = 'imu_link'

# 공분산
COV_ORIENT = [0.0001,0,0, 0,0.0001,0, 0,0,0.0001]
COV_GYRO   = [0.0001,0,0, 0,0.0001,0, 0,0,0.0001]
COV_ACCEL  = [0.025,0,0, 0,0.025,0, 0,0,0.025]

# IMU 상수
G_TO_MS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0
WT_PKT_LEN = 11
ACC_ID, GYRO_ID, ANGLE_ID = 0x51, 0x52, 0x53


# ------------------------
# 유틸
# ------------------------
def le_i16(lo, hi):
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v

def checksum_ok(pkt):
    return (sum(pkt[:10]) & 0xFF) == pkt[10]

def rpy_deg_to_quat(roll_d, pitch_d, yaw_d):
    r = roll_d * DEG_TO_RAD * 0.5
    p = pitch_d * DEG_TO_RAD * 0.5
    y = yaw_d * DEG_TO_RAD * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yy= cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return w, x, yy, z


# ================================================
# ROS2 IMU Node
# ================================================
class WT901CNode(Node):
    def __init__(self):
        super().__init__('wt901c_imu_node')

        self.pub = self.create_publisher(Imu, '/imu_data', 10)

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        self.buf = bytearray()
        time.sleep(0.1)
        self.ser.reset_input_buffer()

        self.acc = [0,0,0]
        self.gyr = [0,0,0]
        self.euler_deg = [0,0,0]
        self.have_acc = self.have_gyro = self.have_ang = False

        # 원하는 출력 주파수 설정 (예: 100Hz)
        self.set_return_rate(50)

        # 50Hz poll
        self.timer = self.create_timer(0.02, self.poll)

        self.get_logger().info(f'[IMU] {SERIAL_PORT} @ {BAUDRATE}bps, frame_id={FRAME_ID}')


    # -----------------------------
    # 실제 센서 출력 주파수 설정
    # -----------------------------
    def set_return_rate(self, hz):
        rate_map = {
            1:0x03, 2:0x04, 5:0x05, 10:0x06, 20:0x07,
            50:0x08, 100:0x09, 125:0x0A, 200:0x0B,
            0:0x0D
        }
        if hz not in rate_map:
            self.get_logger().warn(f"Unsupported rate {hz}Hz")
            return

        code = rate_map[hz]
        # unlock
        self.ser.write(bytes([0xFF,0xAA,0x69,0x88,0xB5]))
        time.sleep(0.05)
        # set frequency
        self.ser.write(bytes([0xFF,0xAA,0x03,code,0x00]))
        time.sleep(0.05)
        # save
        self.ser.write(bytes([0xFF,0xAA,0x00,0x00,0x00]))
        time.sleep(0.05)
        self.get_logger().info(f"[IMU] Return rate set to {hz} Hz")


    # -----------------------------
    # 버퍼 읽기
    # -----------------------------
    def poll(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buf += data
                self._parse_buffer()
        except Exception as e:
            self.get_logger().error(f'[IMU] read err: {e}')


    # -----------------------------
    # 패킷 파싱
    # -----------------------------
    def _parse_buffer(self):
        b = self.buf
        while True:
            idx = b.find(b'\x55')
            if idx < 0:
                if len(b) > 2048: del b[:-1]
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


    # -----------------------------
    # 1개 패킷 처리
    # -----------------------------
    def _parse_packet(self, pkt):
        typ = pkt[1]
        d0,d1,d2,d3,d4,d5,_,_ = pkt[2:10]
        x = le_i16(d0,d1)
        y = le_i16(d2,d3)
        z = le_i16(d4,d5)

        if typ == ACC_ID:
            self.acc[0] = (x/32768*16)*G_TO_MS2
            self.acc[1] = (y/32768*16)*G_TO_MS2
            self.acc[2] = (z/32768*16)*G_TO_MS2
            self.have_acc = True

        elif typ == GYRO_ID:
            self.gyr[0] = (x/32768*2000)*DEG_TO_RAD
            self.gyr[1] = (y/32768*2000)*DEG_TO_RAD
            self.gyr[2] = (z/32768*2000)*DEG_TO_RAD
            self.have_gyro = True

        elif typ == ANGLE_ID:
            self.euler_deg[0] = x/32768*180
            self.euler_deg[1] = y/32768*180
            self.euler_deg[2] = z/32768*180
            self.have_ang = True


    # -----------------------------
    # ROS IMU 메시지 발행
    # -----------------------------
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


# ================================================
# MAIN
# ================================================
def main():
    rclpy.init()
    node = WT901CNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.ser.close()
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
