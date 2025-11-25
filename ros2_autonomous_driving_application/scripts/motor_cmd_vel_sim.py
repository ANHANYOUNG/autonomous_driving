#!/usr/bin/env python3
"""Only Clipping"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorCmdVelSim(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_sim')
        
        # ========== Parameters ==========
        self.declare_parameter('wheel_radius', 0.1)     # 바퀴(스프로킷) 반경 [m]
        self.declare_parameter('wheel_base', 1.5)       # 양 궤도 간격 [m]
        self.declare_parameter('gear_ratio', 60.0)      # 기어비
        self.declare_parameter('max_motor_rpm', 2500.0) # 모터 최대 RPM
        
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
        
        # 클리핑된 실제 명령 (Gazebo용)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        
        # 바퀴 둘레
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        # 이전 명령 값 저장 (변화 감지용)
        self.prev_v_actual = None
        self.prev_w_actual = None
        
        self.get_logger().info(
            f'[MOTOR_SIM] Virtual Motor Driver started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}\n'
            f'  - Input: /cmd_vel_ppc (ideal)\n'
            f'  - Output: /cmd_vel_sim (clipped)'
        )
    
    def cmd_vel_callback(self, msg: Twist):
        # ========== Twist → 목표 RPM ==========
        v = msg.linear.x   # [m/s]
        w = msg.angular.z  # [rad/s]
        
        # 차동 구동 모델: 좌우 바퀴 속도 계산
        v_left_ms = v - (w * self.wheel_base * 0.5)
        v_right_ms = v + (w * self.wheel_base * 0.5)
        
        # 바퀴 속도 → 모터 RPM
        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_SIM] Wheel circumference is zero!')
            return
        
        target_rpm_left = (v_left_ms * 60.0 * self.gear_ratio) / self.wheel_circumference
        target_rpm_right = (v_right_ms * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # ========== RPM 클리핑 (±2500) ==========
        clipped_rpm_left = max(-self.max_motor_rpm, min(self.max_motor_rpm, target_rpm_left))
        clipped_rpm_right = max(-self.max_motor_rpm, min(self.max_motor_rpm, target_rpm_right))
        
        # 클리핑 발생 여부 확인 (디버깅용)
        clipped = (abs(target_rpm_left) > self.max_motor_rpm or 
                   abs(target_rpm_right) > self.max_motor_rpm)
        
        if clipped:
            self.get_logger().warn(
                f'[MOTOR_SIM] RPM CLIPPED\n'
                f'  Target: L={target_rpm_left:.0f}, R={target_rpm_right:.0f}\n'
                f'  Clipped: L={clipped_rpm_left:.0f}, R={clipped_rpm_right:.0f}',
                throttle_duration_sec=1.0
            )
        
        # ========== 클리핑된 RPM → 실제 Twist ==========
        # RPM → 바퀴 속도 [m/s]
        actual_v_left_ms = (clipped_rpm_left * self.wheel_circumference) / (60.0 * self.gear_ratio)
        actual_v_right_ms = (clipped_rpm_right * self.wheel_circumference) / (60.0 * self.gear_ratio)
        
        # 차동 구동 역변환: (v_left, v_right) → (v, w)
        v_actual = (actual_v_left_ms + actual_v_right_ms) / 2.0
        w_actual = (actual_v_right_ms - actual_v_left_ms) / self.wheel_base
        
        # ========== 클리핑된 Twist 발행 ==========
        output_twist = Twist()
        output_twist.linear.x = v_actual
        output_twist.angular.z = w_actual
        self.cmd_vel_pub.publish(output_twist)
        
        # 값이 변했을 때만 로그 출력
        if self.prev_v_actual != v_actual or self.prev_w_actual != w_actual:
            self.get_logger().info(
                f'[MOTOR_SIM] Input: v={v:.3f} m/s, w={w:.3f} rad/s | '
                f'Output: v={v_actual:.3f} m/s, w={w_actual:.3f} rad/s | '
                f'RPM: L={clipped_rpm_left:.0f}, R={clipped_rpm_right:.0f}'
            )
            self.prev_v_actual = v_actual
            self.prev_w_actual = w_actual
    
    def destroy_node(self):
        self.get_logger().info('[MOTOR_SIM] Shutting down Virtual Motor Driver...')
        
        # 정지 명령 발행
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
