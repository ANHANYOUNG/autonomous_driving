#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StateManager(Node):
    """
    - 사용자 명령, 수동 개입, 자율주행 완료 신호를 감지합니다.
    - 시스템의 마스터 상태(STOP, RUN, KEY, ALIGN, CALIBRATION)를 결정합니다.
    - 결정된 상태를 '/robot_state' 토픽으로 발행합니다.
    """
    def __init__(self):
        super().__init__('state_manager_node')
        self.get_logger().info('State Manager (Brain) has been started.')

        # 1. 파라미터 및 내부 상태 변수
        self.declare_parameter('initial_state', 'STOP')
        self.state = self.get_parameter('initial_state').get_parameter_value().string_value

        # 2. 발행 (Publication)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.state_pub = self.create_publisher(String, '/robot_state', state_qos)
        
        # 3. 구독 (Subscriptions)
        self.create_subscription(String, '/state_command', self.command_callback, 10)
        # self.create_subscription(Twist, '/cmd_vel', self.KEY_override_callback, 10)

        # 4. 초기 상태 발행 및 주기적 발행 타이머
        self.get_logger().info(f'Initial state set to: {self.state}')
        self._update_state(self.state, force_publish=True)
        self.publish_timer = self.create_timer(1.0, self.publish_state_loop)

    # 상태 업데이트
    def _update_state(self, new_state, force_publish=False):
        """ 상태를 안전하게 업데이트하고 전파합니다. """
        if new_state == self.state and not force_publish:
            return

        self.get_logger().info(f'State transition: {self.state} -> {new_state}')
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



    # # 수동 조작 감지
    # def KEY_override_callback(self, msg):
    #     if self.state in ['RUN', 'CALIBRATION', 'ALIGN']:
    #         self.get_logger().warn(f'KEY override detected in {self.state} state! Switching to KEY mode.')
    #         self._update_state('KEY')

    # robot_state 토픽을 주기적으로 발행
    def publish_state_loop(self):
        """
        1초마다 현재 상태를 '/robot_state' 토픽으로 계속 발행합니다.
        이를 통해 네트워크 지연이나 노드 재시작 시에도 상태 동기화를 보장합니다.
        """
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self.get_logger().info(f'Publishing state: {self.state}')


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

