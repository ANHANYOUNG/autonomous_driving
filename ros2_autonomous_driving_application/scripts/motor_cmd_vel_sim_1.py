#!/usr/bin/env python3
"""비례적 클리핑"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorCmdVelSim1(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_sim_1')
        
        # ========== Parameters ==========
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 1.5)
        self.declare_parameter('gear_ratio', 60.0)
        self.declare_parameter('max_motor_rpm', 2500.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        
        # ========== Subscribers & Publishers ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_ppc',
            self.cmd_vel_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        self.get_logger().info(
            f'[MOTOR_SIM_1] Virtual Motor Driver (Ratio Clipping) started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}\n'
        )
    
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        
        # 목표 RPM
        v_left_ms = v - (w * self.wheel_base * 0.5)
        v_right_ms = v + (w * self.wheel_base * 0.5)
        
        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_SIM_1] Wheel circumference is zero')
            return
        
        target_rpm_left = (v_left_ms * 60.0 * self.gear_ratio) / self.wheel_circumference
        target_rpm_right = (v_right_ms * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # 초과 비율 계산
        max_rpm_abs = max(abs(target_rpm_left), abs(target_rpm_right))
        
        if max_rpm_abs > self.max_motor_rpm:
            scale = self.max_motor_rpm / max_rpm_abs
            
            # RPM에 scale 적용 (실제 모터 동작)
            actual_rpm_left = target_rpm_left * scale
            actual_rpm_right = target_rpm_right * scale
            
            # 클리핑된 RPM → (v, w) 역변환 (Gazebo 전달용)
            # RPM → 휠 선속도 (m/s)
            v_left_ms_actual = (actual_rpm_left * self.wheel_circumference) / (60.0 * self.gear_ratio)
            v_right_ms_actual = (actual_rpm_right * self.wheel_circumference) / (60.0 * self.gear_ratio)
            
            # 차동구동 역변환: v_L, v_R → v, w
            v_actual = (v_left_ms_actual + v_right_ms_actual) / 2.0
            w_actual = (v_right_ms_actual - v_left_ms_actual) / self.wheel_base
            
            self.get_logger().warn(
                f'[MOTOR_SIM_1] RPM CLIPPED\n'
                f'  Input: v={v:.3f}, w={w:.3f}\n'
                f'  Target RPM: L={target_rpm_left:.0f}, R={target_rpm_right:.0f}\n'
                f'  Scale: {scale:.2%}\n'
                f'  Actual RPM: L={actual_rpm_left:.0f}, R={actual_rpm_right:.0f}\n'
                f'  Output: v={v_actual:.3f}, w={w_actual:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            # 한계 내 → 그대로 발행
            v_actual = v
            w_actual = w
            actual_rpm_left = target_rpm_left
            actual_rpm_right = target_rpm_right
        
        output_twist = Twist()
        output_twist.linear.x = v_actual
        output_twist.angular.z = w_actual
        self.cmd_vel_pub.publish(output_twist)
        
        self.get_logger().info(
            f'[MOTOR_SIM_1] Input: v={v:.3f}, w={w:.3f} | '
            f'Target RPM: L={target_rpm_left:.0f}, R={target_rpm_right:.0f} | '
            f'Actual RPM: L={actual_rpm_left:.0f}, R={actual_rpm_right:.0f} | '
            f'Output: v={v_actual:.3f}, w={w_actual:.3f}',
            throttle_duration_sec=0.5
        )
    
    def destroy_node(self):
        self.get_logger().info('[MOTOR_SIM_1] Shutting down...')
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelSim1()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
