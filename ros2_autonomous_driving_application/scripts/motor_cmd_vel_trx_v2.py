#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import threading
import math 

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MotorCmdVelTrx(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_trx')
        
        # === 1. 시리얼 포트 설정 (기존과 동일) ===
        self.declare_parameter('port', '/dev/usb-right-top')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # --- [수정된 부분 시작] ---
        
        # [신규] 로봇 기구학(Kinematics) 파라미터 (무한궤도형)
        self.declare_parameter('wheel_radius', 0.1)  # Based on GAZEBO unit: [m]
        self.declare_parameter('wheel_base', 0.57)    # Based on GAZEBO unit: [m]
        self.declare_parameter('gear_ratio', 15.0)  
        self.declare_parameter('max_rpm', 1000.0)     # 예: 모터의 최대 RPM (안전 제한용)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().double_value
        
        # 시리얼 포트 열기 (기존과 동일)
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f'Opened serial port: {port} at {baudrate} baud, motor_cmd_vel_trx_v2')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None
        
        # === 2. ROS2 구독자 생성 (기존과 동일) ===
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            state_qos
        )
        
        # === 3. 상태 변수들 [수정] ===
        self.current_robot_state = "STOP"  # [수정] 초기 상태를 "STOP"으로 변경 (IDLE -> STOP)
        self.motor_command = [0, 0]        # 현재 모터 지령값 (RPM)
        self.received_motor_data = [0, 0]  # 수신된 모터 속도 (RPM)
        
        # === 4. 시리얼 통신 스레드 (기존과 동일) ===
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
            self.serial_thread.start()
        
        self.get_logger().info('Motor Command Velocity Transceiver started (ROS Mode)')
    
    # === 5. 시리얼 핸들러 (기존과 동일) ===
    
    def serial_handler(self):
        """시리얼 수신-송신 핸들러 (응답식)"""
        while self.running and self.ser and self.ser.is_open:
            try:
                    data = self.ser.read(4)
                    if len(data) == 4:
                        motor1_rpm, motor2_rpm = struct.unpack('<hh', data)
                        self.received_motor_data = [motor1_rpm, motor2_rpm]
                        self.get_logger().info(
                            f'Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm',
                            throttle_duration_sec=1.0
                        )
                        self.send_motor_response()
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Serial communication error: {e}')
                break
    
    def send_motor_response(self):
        """모터 지령 응답 송신"""
        try:
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)
            self.get_logger().info(
                f'Sent command: M1={self.motor_command[0]}, M2={self.motor_command[1]}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    # === 6. ROS2 콜백 함수 [대폭 수정] ===

    def robot_state_callback(self, msg: String):
        """[수정] /robot_state 토픽을 수신하여 현재 상태를 업데이트합니다."""
        self.current_robot_state = msg.data
        
        # STOP 상태가 되면 모터를 즉시 정지시킵니다.
        if self.current_robot_state == "STOP":
            self.motor_command = [0, 0]
            self.get_logger().info(f'Entering {self.current_robot_state} state. Stopping motors.')

    def cmd_vel_callback(self, msg: Twist):
        """
        [수정] /cmd_vel (Twist) 메시지를 수신하여 RPM으로 변환합니다.
        (m/s, rad/s) -> (rpm_left, rpm_right)
        """
        
        # [수정] 주행 가능 상태가 아니면 명령을 무시합니다.
        # (MANUAL -> KEY, AUTO -> RUN)
        if self.current_robot_state not in ["KEY", "RUN", "CAL", "ALIGN"]:
            return
            
        # 1. /cmd_vel에서 선속도(v)와 각속도(w)를 가져옵니다.
        v = msg.linear.x  # (m/s)
        w = msg.angular.z # (rad/s)
        
        # 2. 차동 구동 로봇(무한궤도형 포함)의 역기구학 (Inverse Kinematics) 계산
        # v_left  = v - (w * L / 2)
        # v_right = v + (w * L / 2)
        # (L = self.wheel_base = 궤도 중심간 거리)
        
        v_left_ms = v - (w * self.wheel_base / 2.0)
        v_right_ms = v + (w * self.wheel_base / 2.0)
        
        # 3. 바퀴(궤도) 속도(m/s)를 RPM으로 변환
        # RPM = (v_ms * 60) / (2 * pi * wheel_radius)
        # (wheel_radius = 구동 스프로킷 반경)
        
        circumference = 2 * math.pi * self.wheel_radius 
        
        if circumference == 0:
            self.get_logger().error('Wheel(Sprocket) radius is zero, cannot calculate RPM.')
            return

        rpm_left = (v_left_ms * 60.0 * self.gear_ratio) / circumference
        rpm_right = (v_right_ms * 60.0 * self.gear_ratio) / circumference
        
        # 4. [안전] 최대 RPM 제한 (Clamping)
        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

        # 6. 최종 모터 지령값(self.motor_command) 업데이트
        # [중요] M1이 왼쪽, M2가 오른쪽이라고 가정합니다.
        # 만약 로봇이 반대로 움직이면 [rpm_right, rpm_left]로 순서를 바꿔야 합니다.
        self.motor_command = [int(rpm_left), int(rpm_right)]    
        self.get_logger().info(f'CmdVel (v={v:.2f}, w={w:.2f}) -> Wheel RPM (L={rpm_left:.0f}, R={rpm_right:.0f})')
    
    
    # --- [수정된 부분 끝] ---

    # === 7. 노드 종료 (기존과 동일) ===
    def destroy_node(self):
        self.get_logger().info('Shutting down Motor Command Velocity Transceiver...')
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('Sent final stop command')
            except:
                pass
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        super().destroy_node()

# === 8. 메인 함수 (기존과 동일) ===
def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelTrx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
