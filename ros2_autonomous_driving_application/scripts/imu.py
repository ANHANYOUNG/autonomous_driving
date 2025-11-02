#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 위 두 줄은 이 스크립트가 python3로 실행되어야 하며, UTF-8 인코딩을 사용함을 명시합니다.

# 필요한 라이브러리들을 가져옵니다.
import math      # 수학 연산(pi, cos, sin 등)을 위해 사용
import time      # time.sleep() 등 시간 관련 함수를 위해 사용
import serial    # 시리얼 통신을 위해 pyserial 라이브러리를 사용 (pip install pyserial)
import rclpy     # ROS 2의 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 노드를 생성하기 위한 기본 클래스
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # 통신 품질(QoS) 설정을 위해 사용
from std_msgs.msg import Header      # 메시지에 포함될 표준 헤더(타임스탬프, 좌표계 ID)
from sensor_msgs.msg import Imu      # IMU 센서 데이터를 위한 표준 메시지 타입

# ==============================================================================
# 사용자 설정: 이 부분의 값들을 자신의 환경에 맞게 수정하세요.
# ==============================================================================
SERIAL_PORT = '/dev/usb-right-bottom'      # IMU 센서가 연결된 시리얼 포트 경로
                                  # USB-UART 변환기는 보통 /dev/ttyUSBx 형식입니다.
BAUDRATE    = 115200              # IMU 센서와 통신할 보드레이트(통신 속도). 장치 설정과 일치해야 합니다.
FRAME_ID    = 'imu_link'          # 발행될 Imu 메시지의 header.frame_id. 로봇의 TF 트리와 일치시키는 것이 좋습니다.

# 공분산 행렬: 센서 측정값의 불확실성을 나타냅니다.
# 지금은 대각 성분만 간단히 설정하여 축 간의 오차 상관관계는 없다고 가정합니다.
# 값이 작을수록 해당 측정값을 더 신뢰한다는 의미입니다.
COV_ORIENT = [0.0001, 0, 0,  0, 0.0001, 0,  0, 0, 0.0001] # 방향(Orientation) 공분산

COV_GYRO   = [0.0001, 0, 0,  0, 0.0001, 0,  0, 0, 0.0001] # 각속도(Angular Velocity) 공분산

COV_ACCEL  = [0.025,  0, 0,  0, 0.025,  0,  0, 0, 0.025]  # 선형 가속도(Linear Acceleration) 공분산
# ==============================================================================
# 내부 상수: WitMotion 프로토콜과 물리 단위 변환을 위한 값들입니다.
# ==============================================================================
G_TO_MS2    = 9.80665              # 중력가속도(g)를 m/s^2로 변환하기 위한 상수
DEG_TO_RAD  = math.pi / 180.0      # 각도(degree)를 라디안(radian)으로 변환하기 위한 상수
WT_PKT_LEN  = 11                   # WitMotion 센서의 표준 패킷 길이 (11바이트)
ACC_ID, GYRO_ID, ANGLE_ID = 0x51, 0x52, 0x53  # 각 데이터 타입(가속도, 각속도, 각도)을 식별하는 ID

# ==============================================================================
# 도우미 함수들
# ==============================================================================

def le_i16(lo: int, hi: int) -> int:
    """리틀 엔디안 형식의 2바이트(low, high)를 16비트 부호 있는 정수로 변환합니다."""
    # high 바이트를 8비트 왼쪽으로 시프트하고 low 바이트와 OR 연산을 하여 16비트 값을 만듭니다.
    v = (hi << 8) | lo
    # 최상위 비트(MSB)가 1이면 음수이므로, 2의 보수법에 따라 실제 음수 값으로 변환합니다.
    return v - 0x10000 if v & 0x8000 else v

def checksum_ok(pkt: bytes) -> bool:
    """수신된 패킷의 체크섬이 유효한지 확인합니다."""
    # 프로토콜 명세에 따라, 처음 10바이트의 합을 255로 나눈 나머지가 마지막 11번째 바이트(체크섬)와 같아야 합니다.
    return (sum(pkt[:10]) & 0xFF) == pkt[10]

def rpy_deg_to_quat(roll_d: float, pitch_d: float, yaw_d: float):
    """오일러 각도(Roll, Pitch, Yaw)를 쿼터니언으로 변환합니다."""
    # ROS의 Imu 메시지는 방향을 쿼터니언으로 표현하므로 변환이 필요합니다.
    # 쿼터니언은 짐벌락(Gimbal Lock) 현상이 없어 3D 회전 표현에 더 안정적입니다.
    # 먼저 각도를 라디안으로 변환하고 계산을 간단히 하기 위해 2로 나눕니다.
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
    """
    WT901C IMU 센서로부터 데이터를 읽어 파싱하고, ROS 2의 Imu 메시지 형식으로 발행하는 노드.
    가장 단순한 형태로, 가속도(ACC), 각속도(GYRO), 각도(ANGLE) 데이터가 한 세트로 모두 수신되면
    하나의 Imu 메시지를 발행하는 구조입니다.
    """
    def __init__(self):
        # rclpy.node.Node 클래스의 생성자를 호출하고 노드 이름을 'wt901c_imu_node'로 설정합니다.
        super().__init__('wt901c_imu_node')

 
        # '/imu/data' 토픽으로 Imu 메시지를 발행할 퍼블리셔를 생성합니다.
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # 시리얼 포트를 엽니다. timeout을 짧게 주어 read가 오랫동안 블로킹되지 않도록 합니다.
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        # 시리얼 데이터는 한번에 전체 패킷이 들어오지 않을 수 있으므로, 임시 저장할 버퍼를 생성합니다.
        self.buf = bytearray()
        time.sleep(0.1) # 포트가 안정적으로 열릴 때까지 잠시 대기
        self.ser.reset_input_buffer() # 시리얼 포트의 입력 버퍼를 비웁니다.

        # 센서로부터 받은 최신 데이터를 저장할 변수들
        self.acc = [0.0, 0.0, 0.0]       # 선형 가속도 (m/s^2)
        self.gyr = [0.0, 0.0, 0.0]       # 각속도 (rad/s)
        self.euler_deg = [0.0, 0.0, 0.0] # 오일러 각도 (roll, pitch, yaw in degrees)
        # 데이터 세트가 완성되었는지 추적하기 위한 플래그
        self.have_acc = self.have_gyro = self.have_ang = False

        # 1ms 주기로 poll 함수를 호출하는 타이머를 생성합니다.
        # 이 타이머는 시리얼 포트를 지속적으로 확인하여 데이터 수신을 처리합니다.
        self.timer = self.create_timer(0.0005, self.poll)

        self.get_logger().info(f'시리얼 포트 {SERIAL_PORT} @ {BAUDRATE}bps, frame_id={FRAME_ID}')

    def poll(self):
        """타이머에 의해 주기적으로 호출되어 시리얼 포트에서 데이터를 읽고 버퍼에 추가합니다."""
        try:
            # 시리얼 버퍼에 수신 대기 중인 모든 데이터(in_waiting)를 읽어옵니다. 데이터가 없으면 1바이트만 읽으려 시도합니다.
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                # 읽어온 데이터가 있으면 내부 버퍼(self.buf)에 추가합니다.
                self.buf += data
                # 버퍼에 데이터가 추가되었으므로, 패킷 파싱을 시도합니다.
                self._parse_buffer()
        except Exception as e:
            # 장치 연결 해제 등 시리얼 통신 중 발생할 수 있는 오류를 로깅합니다.
            self.get_logger().error(f'시리얼 읽기 오류: {e}')

    def _parse_buffer(self):
        """내부 버퍼(self.buf)를 파싱하여 유효한 11바이트 패킷들을 처리합니다."""
        b = self.buf
        while True:
            # 패킷 시작을 알리는 헤더(0x55)를 찾습니다.
            idx = b.find(b'\x55')
            if idx < 0:
                # 헤더가 없으면 파싱할 데이터가 없는 것이므로 루프를 종료합니다.
                # 버퍼가 너무 커지는 것을 방지하기 위해 오래된 데이터를 일부 삭제할 수 있습니다.
                if len(b) > 2048:
                    del b[:-1]
                break
            # 헤더 이전에 있는 불필요한 데이터는 삭제합니다.
            if idx > 0:
                del b[:idx]

            # 버퍼의 길이가 전체 패킷 길이(11바이트)보다 짧으면, 더 많은 데이터가 필요하므로 대기합니다.
            if len(b) < WT_PKT_LEN:
                break

            # 11바이트 패킷을 추출합니다.
            pkt = bytes(b[:WT_PKT_LEN])
            # 체크섬을 확인하여 데이터가 깨지지 않았는지 검증합니다.
            if not checksum_ok(pkt):
                # 체크섬이 틀리면, 현재 헤더(0x55)는 유효하지 않은 데이터일 수 있습니다.
                # 헤더 1바이트만 버리고 다음 바이트부터 다시 헤더를 탐색합니다.
                del b[0:1]
                continue

            # 체크섬까지 통과한 유효한 패킷이므로, 버퍼에서 해당 패킷을 제거(소비)합니다.
            del b[:WT_PKT_LEN]
            # 패킷의 내용을 파싱하여 데이터를 저장합니다.
            self._parse_packet(pkt)

            # 가속도, 각속도, 각도 데이터가 모두 수신되었다면
            if self.have_acc and self.have_gyro and self.have_ang:
                # 하나의 Imu 메시지로 묶어 발행합니다.
                self.publish()
                # 다음 메시지를 위해 플래그를 리셋합니다.
                self.have_acc = self.have_gyro = self.have_ang = False

    def _parse_packet(self, pkt: bytes):
        """하나의 유효한 패킷을 파싱하여 클래스 변수에 데이터를 저장합니다."""
        typ = pkt[1]  # 패킷의 두 번째 바이트는 데이터 타입을 나타냅니다.
        d0,d1,d2,d3,d4,d5,d6,d7 = pkt[2:10] # 실제 데이터 부분
        x = le_i16(d0, d1)
        y = le_i16(d2, d3)
        z = le_i16(d4, d5)

        # 데이터 타입에 따라 적절한 변환 공식을 적용하여 물리 단위로 변환합니다.
        # (센서의 원시 값 / 32768.0 * 측정 범위) * 단위 변환 상수
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

        # 그 외 자력계 등 다른 타입의 패킷은 이 예제에서 무시합니다.

    def publish(self):
        """파싱된 최신 데이터를 사용하여 Imu 메시지를 생성하고 발행합니다."""
        now = self.get_clock().now().to_msg() # 현재 ROS 시간을 가져옵니다.
        msg = Imu()
        msg.header = Header(stamp=now, frame_id=FRAME_ID)

        # 방향(Orientation) 데이터: 오일러 각도(RPY)를 쿼터니언으로 변환하여 채웁니다.
        w,x,y,z = rpy_deg_to_quat(*self.euler_deg)
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation_covariance = COV_ORIENT

        # 각속도(Angular Velocity) 데이터 (단위: rad/s)
        msg.angular_velocity.x = self.gyr[0]
        msg.angular_velocity.y = self.gyr[1]
        msg.angular_velocity.z = self.gyr[2]
        msg.angular_velocity_covariance = COV_GYRO

        # 선형 가속도(Linear Acceleration) 데이터 (단위: m/s^2)
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.linear_acceleration_covariance = COV_ACCEL

        # 완성된 메시지를 발행합니다.
        self.pub.publish(msg)

def main():
    # ROS 2 시스템을 초기화합니다.
    rclpy.init()
    # WT901CNode 클래스의 인스턴스를 생성합니다.
    node = WT901CNode()
    try:
        # rclpy.spin()은 노드를 계속 실행 상태로 유지하며, 타이머와 같은 콜백들을 처리합니다.
        # Ctrl+C 와 같은 종료 신호를 받을 때까지 여기서 대기합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 사용자가 Ctrl+C를 누르면 KeyboardInterrupt 예외가 발생하며, 여기서 처리합니다.
        pass
    finally:
        # 노드가 종료될 때(정상 종료든 예외 발생이든) 반드시 실행되는 부분입니다.
        try:
            # 열었던 시리얼 포트를 닫아 리소스를 해제합니다.
            node.ser.close()
        except Exception:
            pass
        # 노드를 명시적으로 파괴합니다.
        node.destroy_node()
        # ROS 2 시스템을 종료합니다.
        rclpy.shutdown()

if __name__ == '__main__':
    # 이 스크립트가 직접 실행되었을 때 main() 함수를 호출합니다.
    main()