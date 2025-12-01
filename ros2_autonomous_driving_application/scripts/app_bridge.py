#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AppBridge(Node):
    def __init__(self):
        super().__init__('app_bridge_node')
        self.get_logger().info('App Bridge node has been started.')

        # 1. Parameters
        self.declare_parameter('low_speed', 0.1)
        self.declare_parameter('medium_speed', 0.2)
        self.declare_parameter('high_speed', 0.3)
        self.declare_parameter('turn_speed', 0.3)
        self.low_speed = self.get_parameter('low_speed').get_parameter_value().double_value
        self.medium_speed = self.get_parameter('medium_speed').get_parameter_value().double_value
        self.high_speed = self.get_parameter('high_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.current_speed = self.medium_speed  # 초기 속도 설정

        # 2. Publishers
        self.state_command_pub = self.create_publisher(String, '/state_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Int32, '/safety', 10)
        self.video_pub = self.create_publisher(Int32, '/video_enable', 10)

        # 3. Subscribers
        self.create_subscription(String, '/app/sw_bits', self.sw_bits_callback, 10)
        self.create_subscription(String, '/app/key_bits', self.key_bits_callback, 10)
        self.create_subscription(String, '/app/speed_bits', self.speed_bits_callback, 10)
        self.create_subscription(String, '/app/video_bit', self.video_bit_callback, 10)
        self.create_subscription(String, '/app/safe_bit', self.safe_bit_callback, 10)
        
        # state_manager가 발행하는 마스터 상태를 구독
        self.create_subscription(String, '/robot_state', self.robot_state_callback, 10)

        # 4. 내부 상태 및 중복 발행 방지 변수
        self.last_mode_command = None
        self.current_robot_state = "STOP"  #초기 상태를 STOP

        self.get_logger().info('[AppBridge] App Bridge is ready to translate app commands.')

        # 5. 스위치 비트와 명령 매핑 딕셔너리
        self.sw_command_map = {
                    "10000": "STOP",
                    "01000": "KEY",
                    "00100": "CAL",
                    "00010": "ALIGN",
                    "00001": "RUN"
                }
        
        # 비디오/안전 비트 중복 로그 방지용 변수
        self.last_video_bit = None
        self.last_safe_bit = None

    # /robot_state 토픽을 수신하여 현재 상태를 업데이트하는 콜백 함수
    def robot_state_callback(self, msg):
        self.current_robot_state = msg.data

    # /app/sw_bits 토픽을 수신하여 모드 명령으로 변환하는 콜백 함수
    def sw_bits_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '')
        
        # 미리 만들어둔 딕셔너리 사용
        if cleaned_bits in self.sw_command_map:
            command = self.sw_command_map[cleaned_bits]
            if command != self.last_mode_command:
                self.get_logger().info(f'[AppBridge] Received sw_bits "{cleaned_bits}", publishing command: "{command}"')
                command_msg = String()
                command_msg.data = command
                self.state_command_pub.publish(command_msg)
                self.last_mode_command = command
        else:
            self.get_logger().warn(f'[AppBridge] Unknown sw_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)

    # /app/key_bits 토픽을 수신하여 Twist 속도 명령으로 변환하는 콜백 함수
    def key_bits_callback(self, msg):
        # [수정] 'KEY' 상태에서는 모든 조작을, 'STOP' 상태에서는 '정지' 비트만 처리

        cleaned_bits = msg.data.replace('"', '')
        twist_msg = Twist()

        # 1. 정지 명령("0000")인 경우
        if cleaned_bits == "0000":
            # 'KEY' 상태이거나 'STOP' 상태일 때만 정지 명령을 발행
            if self.current_robot_state in ['KEY', 'STOP']:
                self.cmd_vel_pub.publish(twist_msg)
            # RUN이나 CAL 상태라면, 수동 정지 명령을 무시
            return
        
        # 2. 이동 명령("0000"이 아닌)인 경우
        # [수정] 이동 명령은 'KEY' 상태일 때만 처리
        if self.current_robot_state != 'KEY':
            self.get_logger().warn(
                f'[AppBridge] Ignoring movement key_bits in "{self.current_robot_state}" state.', 
                throttle_duration_sec=5.0)
            return
        
        # 3. 'KEY' 상태일 때 이동 명령 매핑
        if cleaned_bits == "1000":  # 앞
            twist_msg.linear.x = self.current_speed
        elif cleaned_bits == "0100":  # 뒤
            twist_msg.linear.x = -self.current_speed
        elif cleaned_bits == "0010":  # 좌
            twist_msg.angular.z = self.turn_speed
        elif cleaned_bits == "0001":  # 우
            twist_msg.angular.z = -self.turn_speed
        else:
            self.get_logger().warn(
                f'[AppBridge] Received unknown key_bits: "{cleaned_bits}"', 
                throttle_duration_sec=5.0)
            return
        self.cmd_vel_pub.publish(twist_msg)

    def speed_bits_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '')
        new_speed = self.medium_speed
        if cleaned_bits == "100":  # 저
            new_speed = self.low_speed
        elif cleaned_bits == "010":  # 중
            new_speed = self.medium_speed
        elif cleaned_bits == "001":  # 고
            new_speed = self.high_speed
        else:
            self.get_logger().warn(f'[AppBridge] Received unknown speed_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)
            
        if new_speed != self.current_speed:
            self.current_speed = new_speed
            self.get_logger().info(f'[AppBridge] Speed set to {self.current_speed:.2f} m/s')

    def video_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()
        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid video_bit: {msg.data}', throttle_duration_sec=5.0)
            return
        
        # 상태가 변했을 때만 로그 출력
        if cleaned_bits != self.last_video_bit:
            if cleaned_bits == "1":
                self.get_logger().info(f'[AppBridge] Video ON')
            else:
                self.get_logger().info(f'[AppBridge] Video OFF')
            self.last_video_bit = cleaned_bits
            
        out = Int32()
        out.data = int(cleaned_bits)
        self.video_pub.publish(out)

    def safe_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()

        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid safe_bit: {msg.data}', throttle_duration_sec=5.0)
            return

        # 상태가 변했을 때만 로그 출력
        if cleaned_bits != self.last_safe_bit:
            if cleaned_bits == "1":
                self.get_logger().info('[AppBridge] Safety ENABLED')
            self.last_safe_bit = cleaned_bits

        # 기존 로직 유지 (1일 때만 0 발행)
        if cleaned_bits == "1":
            out = Int32()
            out.data = 0
            self.safety_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    app_bridge = AppBridge()
    rclpy.spin(app_bridge)
    app_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

