#!/usr/bin/env python3
"""App Bridge: Translate bits from the app into commands"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AppBridge(Node):
    def __init__(self):
        super().__init__('app_bridge_node')
        self.get_logger().info('App Bridge node has been started.')

        # Parameters
        # Speed Setup for KEY mode
        self.declare_parameter('low_speed', 0.1)
        self.declare_parameter('medium_speed', 0.2)
        self.declare_parameter('high_speed', 0.3)
        self.declare_parameter('turn_speed', 0.3)
        self.low_speed = self.get_parameter('low_speed').get_parameter_value().double_value
        self.medium_speed = self.get_parameter('medium_speed').get_parameter_value().double_value
        self.high_speed = self.get_parameter('high_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.current_speed = self.medium_speed  # Initial speed setting

        # Publishers
        self.state_command_pub = self.create_publisher(String, '/state_command', 10) # Mode change
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Velocity command
        self.safety_pub = self.create_publisher(Int32, '/safety', 10) # Safety button for resuming operation after person leaves camera screen
        self.video_pub = self.create_publisher(Int32, '/video_enable', 10) # Video screen on/off

        # Subscriptions
        # Bits from the app
        self.create_subscription(String, '/app/sw_bits', self.sw_bits_callback, 10) # Mode change
        self.create_subscription(String, '/app/key_bits', self.key_bits_callback, 10) # Direction keys in KEY mode
        self.create_subscription(String, '/app/speed_bits', self.speed_bits_callback, 10) # Speed control in KEY mode
        self.create_subscription(String, '/app/video_bit', self.video_bit_callback, 10) # Video screen on/off
        self.create_subscription(String, '/app/safe_bit', self.safe_bit_callback, 10) # Safety button
        
        # State from State Manager
        self.create_subscription(String, '/robot_state', self.robot_state_callback, 10)

        # Prevent repeated commands
        self.last_mode_command = None

        # Initial state: STOP
        self.current_robot_state = "STOP"  

        self.get_logger().info('[AppBridge] App Bridge is ready to translate app commands.')

        # Switch bits to command mapping
        self.sw_command_map = {
                    "10000": "STOP",
                    "01000": "KEY",
                    "00100": "CAL",
                    "00010": "ALIGN",
                    "00001": "RUN"
                }
        
        # Variables to prevent repeated logs for video/safety bits
        self.last_video_bit = None
        self.last_safe_bit = None

    # Update the current state by receiving the /robot_state topic
    def robot_state_callback(self, msg):
        self.current_robot_state = msg.data

    # Convert /app/sw_bits topic to state command
    def sw_bits_callback(self, msg):
        # clean the input
        cleaned_bits = msg.data.replace('"', '')
        
        # Use pre-made dictionary
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

    # Convert /app/key_bits topic to velocity command
    def key_bits_callback(self, msg):
        # In 'KEY' state, handle all key bits, in 'STOP' state, handle only 'stop' bit

        cleaned_bits = msg.data.replace('"', '')
        twist_msg = Twist()

        # Stop command
        if cleaned_bits == "0000":
            # Publish stop command only in 'KEY' or 'STOP' states
            if self.current_robot_state in ['KEY', 'STOP']:
                self.cmd_vel_pub.publish(twist_msg)
            # Ignore manual stop command in RUN or CAL states
            return
        
        # Movement command
        # Handle movement commands only in 'KEY' state
        if self.current_robot_state != 'KEY':
            self.get_logger().warn(
                f'[AppBridge] Ignoring movement key_bits in "{self.current_robot_state}" state.', 
                throttle_duration_sec=5.0)
            return
        
        # Map movement commands in 'KEY' state
        if cleaned_bits == "1000":  # Forward
            twist_msg.linear.x = self.current_speed
        elif cleaned_bits == "0100":  # Backward
            twist_msg.linear.x = -self.current_speed
        elif cleaned_bits == "0010":  # Left
            twist_msg.angular.z = self.turn_speed
        elif cleaned_bits == "0001":  # Right
            twist_msg.angular.z = -self.turn_speed
        else:
            self.get_logger().warn(
                f'[AppBridge] Received unknown key_bits: "{cleaned_bits}"', 
                throttle_duration_sec=5.0)
            return
        self.cmd_vel_pub.publish(twist_msg)

    # Convert /app/speed_bits topic to speed command
    def speed_bits_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '')
        new_speed = self.medium_speed
        if cleaned_bits == "100":  # Slow
            new_speed = self.low_speed
        elif cleaned_bits == "010":  # Medium
            new_speed = self.medium_speed
        elif cleaned_bits == "001":  # Fast
            new_speed = self.high_speed
        else:
            self.get_logger().warn(f'[AppBridge] Received unknown speed_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)
        
        # Update only if speed changed
        if new_speed != self.current_speed:
            self.current_speed = new_speed
            self.get_logger().info(f'[AppBridge] Speed set to {self.current_speed:.2f} m/s')

    # Convert /app/video_bit topic to video on/off command
    def video_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()
        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid video_bit: {msg.data}', throttle_duration_sec=5.0)
            return
        
        # Log only when state changes
        if cleaned_bits != self.last_video_bit:
            if cleaned_bits == "1":
                self.get_logger().info(f'[AppBridge] Video ON')
            else:
                self.get_logger().info(f'[AppBridge] Video OFF')
            self.last_video_bit = cleaned_bits
        out = Int32()
        out.data = int(cleaned_bits)
        self.video_pub.publish(out)

    # Convert /app/safe_bit topic to safety command
    def safe_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()

        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid safe_bit: {msg.data}', throttle_duration_sec=5.0)
            return

        # Log only when state changes
        if cleaned_bits != self.last_safe_bit:
            if cleaned_bits == "1":
                self.get_logger().info('[AppBridge] Safety ENABLED')
            self.last_safe_bit = cleaned_bits

        # When safe_bit is 1, publish 0 to resume operation
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

