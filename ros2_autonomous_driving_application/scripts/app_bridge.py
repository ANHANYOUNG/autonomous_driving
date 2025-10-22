#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# [신규] QoS 프로파일을 위해 필요한 클래스들을 임포트합니다.
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AppBridge(Node):
    """
    앱 인벤터의 신호('/app/...')를 ROS2 표준 토픽('/state_command', '/cmd_vel')으로
    변환(Bridge)하는 노드입니다.
    [수정] 이제 로봇의 현재 상태를 인지하여 MANUAL 상태에서만 조작 명령을 변환합니다.
    """
    def __init__(self):
        super().__init__('app_bridge_node')
        self.get_logger().info('App Bridge node has been started.')

        # 1. 파라미터 선언 (속도 값)
        self.declare_parameter('forward_speed', 0.5)
        self.declare_parameter('turn_speed', 0.3)
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        # 2. 발행 (Publishers)
        self.state_command_pub = self.create_publisher(String, '/state_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. 구독 (Subscribers)
        self.create_subscription(String, '/app/sw_bits', self.sw_bits_callback, 10)
        self.create_subscription(String, '/app/key_bits', self.key_bits_callback, 10)
        
        # [신규] state_manager가 발행하는 마스터 상태를 구독합니다.
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(String, '/robot_state', self.robot_state_callback, state_qos)

        # 4. 내부 상태 및 중복 발행 방지 변수
        self.last_mode_command = None
        self.current_robot_state = None  # [신규] 로봇의 현재 상태를 저장할 변수 초기화
        
        self.get_logger().info('App Bridge is ready to translate app commands.')

    # [신규] /robot_state 토픽을 수신하여 현재 상태를 업데이트하는 콜백 함수
    def robot_state_callback(self, msg):
        self.current_robot_state = msg.data

    def sw_bits_callback(self, msg):
        """ /app/sw_bits를 수신하여 모드(상태) 명령으로 변환합니다. """
        cleaned_bits = msg.data.replace('"', '')
        
        command_map = {
            "10000": "IDLE",
            "01000": "MANUAL",
            "00100": "CAL",
            "00010": "ALIGN",
            "00001": "AUTO"
        }
        
        if cleaned_bits in command_map:
            command = command_map[cleaned_bits]
            if command != self.last_mode_command:
                self.get_logger().info(f'Received sw_bits "{cleaned_bits}", publishing command: "{command}"')
                command_msg = String()
                command_msg.data = command
                self.state_command_pub.publish(command_msg)
                self.last_mode_command = command
        else:
            self.get_logger().warn(f'Received unknown sw_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)


    def key_bits_callback(self, msg):
        """ /app/key_bits를 수신하여 Twist 속도 명령으로 변환합니다. """
        # [수정] 현재 상태가 'MANUAL'이 아니면 아무것도 하지 않고 함수를 종료합니다.
        if self.current_robot_state != 'MANUAL':
            self.get_logger().warn(f'Ignoring key_bits in "{self.current_robot_state}" state.', throttle_duration_sec=5.0)
            return

        cleaned_bits = msg.data.replace('"', '')
        twist_msg = Twist()

        if cleaned_bits == "1000":  # 앞
            twist_msg.linear.x = self.forward_speed
        elif cleaned_bits == "0100":  # 뒤
            twist_msg.linear.x = -self.forward_speed
        elif cleaned_bits == "0010":  # 좌
            twist_msg.angular.z = self.turn_speed
        elif cleaned_bits == "0001":  # 우
            twist_msg.angular.z = -self.turn_speed
        elif cleaned_bits == "0000":  # 정지
            pass
        else:
            self.get_logger().warn(f'Received unknown key_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)
            return

        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    app_bridge = AppBridge()
    rclpy.spin(app_bridge)
    app_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

