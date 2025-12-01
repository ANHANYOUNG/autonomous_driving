#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import threading


class MotorCmdVelTrx(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_trx')
        
        # 시리얼 포트 설정
        self.declare_parameter('port', '/dev/usb-left-top')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 시리얼 포트 열기
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f'Opened serial port: {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None
        
        # ROS2 구독자 생성
        self.key_bits_sub = self.create_subscription(
            String,
            '/app/key_bits',
            self.key_bits_callback,
            1
        )
        
        self.sw_bits_sub = self.create_subscription(
            String,
            '/app/sw_bits',
            self.sw_bits_callback,
            1
        )
        
        # 상태 변수들
        self.current_key_bits = [0, 0, 0, 0]  # front, back, left, right
        self.current_sw_bits = [0, 0, 0, 0, 0]  # stop, key, calib, align, run
        self.motor_command = [0, 0]  # 현재 모터 지령값
        self.received_motor_data = [0, 0]  # 수신된 모터 속도
        
        # 시리얼 통신 스레드 시작 (수신-송신 응답식)
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
            self.serial_thread.start()
        
        self.get_logger().info('Motor Command Velocity Transceiver started (Request-Response mode)')
    
    def serial_handler(self):
        """시리얼 수신-송신 핸들러 (응답식)"""
        while self.running and self.ser and self.ser.is_open:
            try:
                # 1. 데이터 수신 대기 (int16 data[2] = 4바이트)
                if self.ser.in_waiting >= 4:
                    data = self.ser.read(4)
                    if len(data) == 4:
                        # 리틀 엔디안으로 int16 2개 언팩
                        motor1_rpm, motor2_rpm = struct.unpack('<hh', data)
                        self.received_motor_data = [motor1_rpm, motor2_rpm]
                        
                        self.get_logger().debug(
                            f'Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm'
                        )
                        
                        # 2. 즉시 모터 지령 응답 송신
                        self.send_motor_response()
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Serial communication error: {e}')
                break
    
    def send_motor_response(self):
        """모터 지령 응답 송신"""
        try:
            # 현재 모터 지령을 int16 data[2]로 송신
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)
            
            self.get_logger().debug(
                f'Sent command: M1={self.motor_command[0]}, M2={self.motor_command[1]}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def key_bits_callback(self, msg):
        """키 비트 데이터 처리"""
        try:
            # '"0""0""0""0"' 형태의 문자열 파싱
            bits_str = msg.data.replace('"', '')  # 따옴표 제거
            self.current_key_bits = [int(bit) for bit in bits_str]
            
            self.get_logger().debug(f'Key bits: {self.current_key_bits}')
            
            # 키 조작 모드일 때만 동작
            if self.current_sw_bits[1] == 1:  # key 조작 모드
                self.update_motor_command()
            
        except Exception as e:
            self.get_logger().warn(f'Failed to parse key bits: {msg.data}, error: {e}')
    
    def sw_bits_callback(self, msg):
        """스위치 비트 데이터 처리"""
        try:
            # '"0""0""0""0""1"' 형태의 문자열 파싱
            bits_str = msg.data.replace('"', '')  # 따옴표 제거
            self.current_sw_bits = [int(bit) for bit in bits_str]
            
            self.get_logger().debug(f'SW bits: {self.current_sw_bits}')
            
            # 상태에 따른 모터 지령 업데이트
            self.update_motor_command()
            
        except Exception as e:
            self.get_logger().warn(f'Failed to parse sw bits: {msg.data}, error: {e}')
    
    def update_motor_command(self):
        """현재 상태에 따라 모터 지령 업데이트"""
        # stop 상태 처리 (최우선)
        if self.current_sw_bits[0] == 1:  # stop 활성
            self.motor_command = [0, 0]
            self.get_logger().info('STOP mode - Command: [0, 0]')
            return
        
        # key 조작 모드
        if self.current_sw_bits[1] == 1:  # key 조작 모드
            self.process_key_commands()
            return
        
        # calibration 모드
        if self.current_sw_bits[2] == 1:  # calibration 모드
            self.process_calibration_mode()
            return
        
        # align 모드
        if self.current_sw_bits[3] == 1:  # align 모드
            self.process_align_mode()
            return
        
        # run 모드
        if self.current_sw_bits[4] == 1:  # run 모드
            self.process_run_mode()
            return
        
        # 모든 모드가 비활성화된 경우
        self.motor_command = [0, 0]
        self.get_logger().debug('No active mode - Command: [0, 0]')
    
    def process_key_commands(self):
        """키 명령 처리"""
        # front, back, left, right 순서
        front = self.current_key_bits[0]
        back = self.current_key_bits[1]
        left = self.current_key_bits[2]
        right = self.current_key_bits[3]
        
        if front == 1:
            # 전진: 500, 500
            self.motor_command = [500, 500]
            self.get_logger().info('Key: FRONT - Command: [500, 500]')
            
        elif back == 1:
            # 후진: -500, -500
            self.motor_command = [-500, -500]
            self.get_logger().info('Key: BACK - Command: [-500, -500]')
            
        elif left == 1:
            # 좌회전: 250, -250
            self.motor_command = [250, -250]
            self.get_logger().info('Key: LEFT - Command: [250, -250]')

        elif right == 1:
            # 우회전: -250, 250
            self.motor_command = [-250, 250]
            self.get_logger().info('Key: RIGHT - Command: [-250, 250]')
            
        else:
            # 키가 눌리지 않음
            self.motor_command = [0, 0]
            self.get_logger().debug('No key pressed - Command: [0, 0]')
    
    def process_calibration_mode(self):
        """캘리브레이션 모드 처리 (템플릿)"""
        # TODO: 캘리브레이션 로직 구현
        self.motor_command = [0, 0]  # 일단 정지
        self.get_logger().info('Calibration mode - Command: [0, 0] (Not implemented)')
    
    def process_align_mode(self):
        """정렬 모드 처리 (템플릿)"""
        # TODO: 정렬 로직 구현
        self.motor_command = [0, 0]  # 일단 정지
        self.get_logger().info('Align mode - Command: [0, 0] (Not implemented)')
    
    def process_run_mode(self):
        """자동 주행 모드 처리 (템플릿)"""
        # TODO: 자동 주행 로직 구현
        # 예: Pure Pursuit 등의 주행 알고리즘 결과를 motor_command에 설정
        self.motor_command = [0, 0]  # 일단 정지
        self.get_logger().info('Run mode - Command: [0, 0] (Not implemented)')
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('Shutting down Motor Command Velocity Transceiver...')
        
        self.running = False
        
        # 최종 정지 명령 송신
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('Sent final stop command')
            except:
                pass
        
        # 시리얼 포트 닫기
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        
        super().destroy_node()


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