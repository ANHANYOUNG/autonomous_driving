#!/usr/bin/env python3
"""No motor clipping control, Real"""
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
        
        self.declare_parameter('port', '/dev/usb-left-top')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Kinematics Parameters
        self.declare_parameter('wheel_radius', 0.1)  # [m]
        self.declare_parameter('wheel_base', 1.5)    # [m]
        self.declare_parameter('gear_ratio', 60.0)  

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        
        # Open Serial Port
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f'[MOTOR_CMD_VEL] Opened serial port: {port} at {baudrate} baud, motor_cmd_vel_trx_v2')
        except Exception as e:
            self.get_logger().error(f'[MOTOR_CMD_VEL] Failed to open serial port {port}: {e}')
            self.ser = None

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        
        self.state_sub = self.create_subscription(String, '/robot_state', self.robot_state_callback, 10)
        
        # Publishers
        self.motor_rpm_pub = self.create_publisher(Twist, '/motor_rpm', 10)
        
        # State Variables
        self.current_robot_state = "STOP"  # 초기 상태를 "STOP"으로 변경
        self.motor_command = [0, 0]        # 현재 모터 지령값 (RPM)
        self.received_motor_data = [0, 0]  # 수신된 모터 속도 (RPM)
        
        # Previous command for change detection
        self.prev_motor_command = None
        self.prev_received_rpm = None  # 수신 RPM 변화 감지용
        self.prev_sent_rpm = None  # 송신 RPM 변화 감지용
        
        # Serial Communication Thread
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
            self.serial_thread.start()

        self.get_logger().info('[MOTOR_CMD_VEL] Motor Command Velocity Transceiver started (ROS Mode)')

    def serial_handler(self):
        """Serial receive-transmit handler"""
        while self.running and self.ser and self.ser.is_open:
            try:
                    data = self.ser.read(4)
                    if len(data) == 4:
                        motor1_rpm, motor2_rpm = struct.unpack('<hh', data)
                        self.received_motor_data = [motor1_rpm, motor2_rpm]
                        
                        # Data storage only, no subscribers to motor_rpm
                        rpm_msg = Twist()
                        rpm_msg.angular.x = float(motor1_rpm)  # Left RPM
                        rpm_msg.angular.y = float(motor2_rpm)  # Right RPM
                        self.motor_rpm_pub.publish(rpm_msg)
                        
                        # Log only when RPM values change
                        if self.prev_received_rpm != self.received_motor_data:
                            self.get_logger().info(
                                f'[MOTOR_CMD_VEL] Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm'
                            )
                            self.prev_received_rpm = self.received_motor_data.copy()
                        
                        self.send_motor_response()
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'[MOTOR_CMD_VEL] Serial communication error: {e}')
                break
    
    def send_motor_response(self):
        """모터 지령 응답 송신"""
        try:
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)
            
            # Log only when RPM command changes
            if self.prev_sent_rpm != self.motor_command:
                self.get_logger().info(
                    f'[MOTOR_CMD_VEL] Sent: M1={self.motor_command[0]} rpm, M2={self.motor_command[1]} rpm'
                )
                self.prev_sent_rpm = self.motor_command.copy()
        except Exception as e:
            self.get_logger().error(f'[MOTOR_CMD_VEL] Serial write error: {e}')

    def robot_state_callback(self, msg: String):
        """/robot_state 토픽을 수신하여 현재 상태를 업데이트합니다."""
        self.current_robot_state = msg.data
        
        # When entering STOP state, set motor commands to zero
        if self.current_robot_state == "STOP":
            self.motor_command = [0, 0]
            self.get_logger().info(f'[MOTOR_CMD_VEL] Entering {self.current_robot_state} state. Stopping motors.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert /cmd_vel (Twist) message to RPM.
        (m/s, rad/s) -> (rpm_left, rpm_right)
        """
        # Set motor commands to zero if not in a drivable state
        if self.current_robot_state not in ["KEY", "RUN", "CALIBRATION", "ALIGN"]:
            self.motor_command = [0, 0]
            return
        # Extract linear velocity (v) and angular velocity (w)
        v = msg.linear.x  # (m/s)
        w = msg.angular.z # (rad/s)

        # Convert linear and angular velocity to wheel (track) velocity
        v_left_ms = v - (w * self.wheel_base * 0.5)
        v_right_ms = v + (w * self.wheel_base * 0.5)

        # Convert wheel speed (m/s) to motor speed (RPM)
        circumference = 2 * math.pi * self.wheel_radius 
        if circumference == 0:
            self.get_logger().error('[MOTOR_CMD_VEL] Wheel(Sprocket) radius is zero, cannot calculate RPM.')
            return
        rpm_left = (v_left_ms * 60.0 * self.gear_ratio) / circumference
        rpm_right = (v_right_ms * 60.0 * self.gear_ratio) / circumference

        # Update final motor command (self.motor_command)
        self.motor_command = [int(rpm_left), int(rpm_right)]
        
        # Log only when change
        if self.prev_motor_command != self.motor_command:
            self.get_logger().info(
                f'[MOTOR_CMD_VEL] CmdVel (v={v:.2f}, w={w:.2f}) -> '
                f'Wheel RPM (L={rpm_left:.0f}, R={rpm_right:.0f})'
            )
            self.prev_motor_command = self.motor_command.copy()

    def destroy_node(self):
        self.get_logger().info('[MOTOR_CMD_VEL] Shutting down Motor Command Velocity Transceiver...')
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('[MOTOR_CMD_VEL] Sent final stop command')
            except:
                pass
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