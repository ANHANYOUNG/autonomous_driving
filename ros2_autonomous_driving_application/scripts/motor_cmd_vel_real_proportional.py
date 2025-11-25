#!/usr/bin/env python3
"""Real, Proportional Clipping"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import threading
import math 
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MotorCmdVelRealProportional(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_real_proportional')
        
        # ========== Serial Port Parameters ==========
        self.declare_parameter('port', '/dev/usb-right-top')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # ========== Kinematics Parameters ==========
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 1.5)
        self.declare_parameter('gear_ratio', 60.0)
        self.declare_parameter('max_motor_rpm', 3000.0)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        # ========== Serial Port Open ==========
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(
                f'[MOTOR_PROP] Opened serial port: {port} at {baudrate} baud\n'
                f'  Strategy: Proportional Clipping (Both v and w scaled equally)'
            )
        except Exception as e:
            self.get_logger().error(f'[MOTOR_PROP] Failed to open serial port {port}: {e}')
            self.ser = None

        # ========== Subscriptions ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        
        self.state_sub = self.create_subscription(String, '/robot_state', self.robot_state_callback, 10)
        
        # ========== Publishers ==========
        self.motor_rpm_pub = self.create_publisher(Twist, '/motor_rpm', 10)

        # ========== State Variables ==========
        self.current_robot_state = "STOP"
        self.motor_command = [0, 0]  # [rpm_left, rpm_right]
        self.received_motor_data = [0, 0]
        
        # Previous command for change detection
        self.prev_motor_command = None
        self.prev_v_input = None
        self.prev_w_input = None
        
        # ========== Serial Communication Thread ==========
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
            self.serial_thread.start()

        self.get_logger().info(
            f'[MOTOR_PROP] Motor driver started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}'
        )

    def serial_handler(self):
        """Serial receive-transmit handler"""
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(4)
                if len(data) == 4:
                    motor1_rpm, motor2_rpm = struct.unpack('<hh', data)
                    self.received_motor_data = [motor1_rpm, motor2_rpm]

                    # 데이터 저장용 아무도 안 듣는 motor_rpm 퍼블리셔
                    rpm_msg = Twist()
                    rpm_msg.angular.x = float(motor1_rpm) # Left RPM (임시로 angular.x 사용)
                    rpm_msg.angular.y = float(motor2_rpm) # Right RPM (임시로 angular.y 사용)
                    self.motor_rpm_pub.publish(rpm_msg)

                    self.get_logger().info(
                        f'[MOTOR_PROP] Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm',
                        throttle_duration_sec=1.0
                    )
                    self.send_motor_response()
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'[MOTOR_PROP] Serial communication error: {e}')
                break
    
    def send_motor_response(self):
        """Send motor command response"""
        try:
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)
            self.get_logger().info(
                f'[MOTOR_PROP] Sent command: M1={self.motor_command[0]}, M2={self.motor_command[1]}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'[MOTOR_PROP] Serial write error: {e}')

    def robot_state_callback(self, msg: String):
        """Update robot state and stop motors if entering STOP state"""
        self.current_robot_state = msg.data
        
        if self.current_robot_state == "STOP":
            self.motor_command = [0, 0]
            self.get_logger().info(f'[MOTOR_PROP] Entering {self.current_robot_state} state. Stopping motors.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert /cmd_vel to RPM with proportional clipping
        Strategy: If either wheel exceeds max RPM, scale both wheels equally
        """
        # Stop motors if not in driving state
        if self.current_robot_state not in ["KEY", "RUN", "CALIBRATION", "ALIGN"]:
            self.motor_command = [0, 0]
            return
        
        v = msg.linear.x  # m/s
        w = msg.angular.z # rad/s

        # Convert to wheel velocities
        v_left_ms = v - (w * self.wheel_base * 0.5)
        v_right_ms = v + (w * self.wheel_base * 0.5)

        # Convert to motor RPM
        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_PROP] Wheel circumference is zero!')
            return
        
        target_rpm_left = (v_left_ms * 60.0 * self.gear_ratio) / self.wheel_circumference
        target_rpm_right = (v_right_ms * 60.0 * self.gear_ratio) / self.wheel_circumference

        # ========== Proportional Clipping ==========
        max_rpm_abs = max(abs(target_rpm_left), abs(target_rpm_right))
        
        if max_rpm_abs > self.max_motor_rpm:
            # Calculate scale factor
            scale = self.max_motor_rpm / max_rpm_abs
            
            # Apply scale to both wheels
            actual_rpm_left = target_rpm_left * scale
            actual_rpm_right = target_rpm_right * scale
            
            self.get_logger().warn(
                f'[MOTOR_PROP] RPM CLIPPED\n'
                f'  Input: v={v:.3f}, w={w:.3f}\n'
                f'  Target RPM: L={target_rpm_left:.0f}, R={target_rpm_right:.0f}\n'
                f'  Scale: {scale:.2%}\n'
                f'  Actual RPM: L={actual_rpm_left:.0f}, R={actual_rpm_right:.0f}',
                throttle_duration_sec=1.0
            )
        else:
            actual_rpm_left = target_rpm_left
            actual_rpm_right = target_rpm_right

        # Update motor command
        self.motor_command = [int(actual_rpm_left), int(actual_rpm_right)]
        
        # Log only when values change
        if (self.prev_motor_command != self.motor_command or 
            self.prev_v_input != v or self.prev_w_input != w):
            self.get_logger().info(
                f'[MOTOR_PROP] CmdVel (v={v:.2f}, w={w:.2f}) -> '
                f'Target RPM (L={target_rpm_left:.0f}, R={target_rpm_right:.0f}) -> '
                f'Actual RPM (L={actual_rpm_left:.0f}, R={actual_rpm_right:.0f})'
            )
            self.prev_motor_command = self.motor_command.copy()
            self.prev_v_input = v
            self.prev_w_input = w

    def destroy_node(self):
        self.get_logger().info('[MOTOR_PROP] Shutting down...')
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('[MOTOR_PROP] Sent final stop command')
            except:
                pass
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelRealProportional()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
