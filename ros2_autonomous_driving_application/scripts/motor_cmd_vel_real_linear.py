#!/usr/bin/env python3
"""Real, Linear"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import threading
import math 
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MotorCmdVelRealLinear(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_real_linear')
        
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
        
        # ========== Max Velocities ==========
        self.max_linear_velocity = 0.35
        max_wheel_velocity = self.max_linear_velocity
        self.max_angular_velocity = (2.0 * max_wheel_velocity) / self.wheel_base
        
        # ========== Serial Port Open ==========
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(
                f'[MOTOR_LINEAR] Opened serial port: {port} at {baudrate} baud\n'
                f'  Strategy: Linear Priority'
            )
        except Exception as e:
            self.get_logger().error(f'[MOTOR_LINEAR] Failed to open serial port {port}: {e}')
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
            f'[MOTOR_LINEAR] Motor driver started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}\n'
            f'  - Max Linear Velocity: {self.max_linear_velocity:.3f} m/s\n'
            f'  - Max Angular Velocity: {self.max_angular_velocity:.3f} rad/s'
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
                        f'[MOTOR_LINEAR] Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm',
                        throttle_duration_sec=1.0
                    )
                    self.send_motor_response()
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'[MOTOR_LINEAR] Serial communication error: {e}')
                break
    
    def send_motor_response(self):
        """Send motor command response"""
        try:
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)
            self.get_logger().info(
                f'[MOTOR_LINEAR] Sent command: M1={self.motor_command[0]}, M2={self.motor_command[1]}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'[MOTOR_LINEAR] Serial write error: {e}')

    def robot_state_callback(self, msg: String):
        """Update robot state and stop motors if entering STOP state"""
        self.current_robot_state = msg.data
        
        if self.current_robot_state == "STOP":
            self.motor_command = [0, 0]
            self.get_logger().info(f'[MOTOR_LINEAR] Entering {self.current_robot_state} state. Stopping motors.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert /cmd_vel to RPM with linear priority
        Strategy: Secure v first, then damp w if RPM limit is exceeded
        """
        # Stop motors if not in driving state
        if self.current_robot_state not in ["KEY", "RUN", "CALIBRATION", "ALIGN"]:
            self.motor_command = [0, 0]
            return
        
        v_cmd = msg.linear.x  # m/s
        w_cmd = msg.angular.z # rad/s

        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_LINEAR] Wheel circumference is zero!')
            return

        # ========== Linear Velocity Clipping (Priority 1) ==========
        if abs(v_cmd) > self.max_linear_velocity:
            v_final = math.copysign(self.max_linear_velocity, v_cmd)
            self.get_logger().warn(
                f'[MOTOR_LINEAR] LINEAR CLIPPED\n'
                f'  Input v: {v_cmd:.3f} m/s\n'
                f'  Output v: {v_final:.3f} m/s (Max: {self.max_linear_velocity:.3f})',
                throttle_duration_sec=1.0
            )
        else:
            v_final = v_cmd

        # ========== Angular Velocity Damping (Priority 2) ==========
        # Calculate remaining RPM capacity after securing v
        wheel_velocity_for_v = abs(v_final)
        max_wheel_diff = self.max_linear_velocity - wheel_velocity_for_v
        
        if max_wheel_diff < 0:
            max_wheel_diff = 0.0
        
        # Convert to max allowable angular velocity
        max_w_allowed = (2.0 * max_wheel_diff) / self.wheel_base
        
        if abs(w_cmd) > max_w_allowed:
            w_final = math.copysign(max_w_allowed, w_cmd)
            
            # Calculate damping ratio for logging
            damping_ratio = (max_w_allowed / abs(w_cmd)) * 100 if w_cmd != 0 else 100
            
            self.get_logger().warn(
                f'[MOTOR_LINEAR] ANGULAR DAMPED\n'
                f'  Input: v={v_final:.3f}, w={w_cmd:.3f}\n'
                f'  Max w allowed: {max_w_allowed:.3f} rad/s (after v reservation)\n'
                f'  Damping ratio: {damping_ratio:.1f}% (w reduced to prevent oscillation)\n'
                f'  Output: v={v_final:.3f}, w={w_final:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            w_final = w_cmd

        # ========== Convert to Wheel Velocities and RPM ==========
        v_left = v_final - (w_final * self.wheel_base * 0.5)
        v_right = v_final + (w_final * self.wheel_base * 0.5)
        
        rpm_left = (v_left * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right = (v_right * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # ========== Safety Verification ==========
        if abs(rpm_left) > self.max_motor_rpm or abs(rpm_right) > self.max_motor_rpm:
            self.get_logger().error(
                f'[MOTOR_LINEAR] RPM EXCEEDED (LOGIC ERROR)\n'
                f'  Target RPM: L={rpm_left:.0f}, R={rpm_right:.0f}\n'
                f'  Max RPM: {self.max_motor_rpm:.0f}\n'
                f'  Emergency Stop Activated'
            )
            rpm_left = 0.0
            rpm_right = 0.0

        # Update motor command
        self.motor_command = [int(rpm_left), int(rpm_right)]
        
        # Log only when values change
        if (self.prev_motor_command != self.motor_command or 
            self.prev_v_input != v_cmd or self.prev_w_input != w_cmd):
            
            # Calculate target RPM for comparison
            v_left_target = v_cmd - (w_cmd * self.wheel_base * 0.5)
            v_right_target = v_cmd + (w_cmd * self.wheel_base * 0.5)
            rpm_left_target = (v_left_target * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_target = (v_right_target * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            self.get_logger().info(
                f'[MOTOR_REAL_LINEAR] CmdVel (v={v_cmd:.2f}, w={w_cmd:.2f}) -> '
                f'Target RPM (L={rpm_left_target:.0f}, R={rpm_right_target:.0f}) -> '
                f'Actual RPM (L={rpm_left:.0f}, R={rpm_right:.0f})'
            )
            self.prev_motor_command = self.motor_command.copy()
            self.prev_v_input = v_cmd
            self.prev_w_input = w_cmd

    def destroy_node(self):
        self.get_logger().info('[MOTOR_REAL_LINEAR] Shutting down...')
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('[MOTOR_REAL_LINEAR] Sent final stop command')
            except:
                pass
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelRealLinear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
