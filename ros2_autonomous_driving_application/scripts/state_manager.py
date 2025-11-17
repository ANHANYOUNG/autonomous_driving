#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.get_logger().info('[STATE_MANAGER] State Manager (Brain) has been started.')

        # 1. 파라미터 및 내부 상태 변수
        self.declare_parameter('initial_state', 'STOP')
        self.state = self.get_parameter('initial_state').get_parameter_value().string_value

        # 2. 발행 (Publication)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        
        # 3. 구독 (Subscriptions)
        self.create_subscription(String, '/state_command', self.command_callback, 10)


        # 4. 초기 상태 발행 및 주기적 발행 타이머
        self.get_logger().info(f'[STATE_MANAGER] Initial state set to: {self.state}')
        self._update_state(self.state, force_publish=True)
        self.publish_timer = self.create_timer(1.0, self.publish_state_loop)

    # 상태 업데이트
    def _update_state(self, new_state, force_publish=False):
        """ 상태를 안전하게 업데이트하고 전파"""
        if new_state == self.state and not force_publish:
            return

        self.get_logger().info(f'[STATE_MANAGER] State transition: {self.state} -> {new_state}')
        self.state = new_state
        
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    # 상태 명령 처리
    def command_callback(self, msg):
        command = msg.data
        
        # E_STOP 또는 STOP 명령은 항상 STOP 상태로 전환
        if command == 'E_STOP' or command == 'STOP':
            self._update_state('STOP')
        
        elif command == 'RUN':
            self._update_state('RUN')

        elif command == 'KEY':
            self._update_state('KEY')
            
        elif command == 'CAL':
            self._update_state('CALIBRATION')
            
        elif command == 'ALIGN':
            self._update_state('ALIGN')
            return

    # robot_state 토픽을 주기적으로 발행
    def publish_state_loop(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self.get_logger().info(f'[STATE_MANAGER] Publishing state: {self.state}')


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

