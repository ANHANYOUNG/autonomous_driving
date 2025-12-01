#!/usr/bin/env python3
"""State Manager: state change and broadcast"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.get_logger().info('[STATE_MANAGER] State Manager (Brain) has been started.')

        # Initial state setting
        self.declare_parameter('initial_state', 'STOP')
        self.state = self.get_parameter('initial_state').get_parameter_value().string_value
        self.last_logged_state = None

        # Publication
        self.state_pub = self.create_publisher(String, '/robot_state', 10) # current state or changed state broadcast
        
        # Subscriptions
        self.create_subscription(String, '/state_command', self.command_callback, 10) # state change command

        # Initial state publish and periodic publish timer
        self.get_logger().info(f'[STATE_MANAGER] Initial state set to: {self.state}')
        self._update_state(self.state, force_publish=True)
        self.publish_timer = self.create_timer(1.0, self.publish_state_loop) # 1Hz

    # Change state and publish once
    def _update_state(self, new_state, force_publish=False):
        if new_state == self.state and not force_publish:
            return

        self.get_logger().info(f'[STATE_MANAGER] State transition: {self.state} -> {new_state}')
        self.state = new_state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    # Publish current state periodically for case of missing messages
    def publish_state_loop(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        if self.state != self.last_logged_state:
            self.get_logger().info(f'[STATE_MANAGER] Publishing state: {self.state}')
            self.last_logged_state = self.state
            
    # Handle state commands
    def command_callback(self, msg):
        command = msg.data
        
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



def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

