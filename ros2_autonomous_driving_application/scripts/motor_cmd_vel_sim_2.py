#!/usr/bin/env python3
"""각속도 우선 - 폐지 """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorCmdVelSim2(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_sim_2')
        
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

        # ========== Max linear vel, angular vel ==========
        self.max_linear_velocity = (self.max_motor_rpm * self.wheel_circumference) / (60.0 * self.gear_ratio)

        max_wheel_velocity = self.max_linear_velocity
        self.max_angular_velocity = (2.0 * max_wheel_velocity) / self.wheel_base
        
        # 이전 명령 값 저장 (변화 감지용)
        self.prev_v_final = None
        self.prev_w_final = None
        
        self.get_logger().info(
            f'[MOTOR_SIM_2] Virtual Motor Driver (w priority) started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}\n'
            f'  - Max Linear Velocity: {self.max_linear_velocity:.3f} m/s\n'
            f'  - Max Angular Velocity: {self.max_angular_velocity:.3f} rad/s',
        )
    
    def cmd_vel_callback(self, msg: Twist):
        v_cmd = msg.linear.x
        w_cmd = msg.angular.z
        
        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_SIM_2] Wheel circumference is zero')
            return

        # ========== Angular Velocity Clipping ==========
        if abs(w_cmd) > self.max_angular_velocity:
            w_final = math.copysign(self.max_angular_velocity, w_cmd)
            self.get_logger().warn(
                f'[MOTOR_SIM_2] ANGULAR CLIPPED\n'
                f'  Input w: {w_cmd:.3f} rad/s\n'
                f'  Output w: {w_final:.3f} rad/s (Max: {self.max_angular_velocity:.3f})',
                throttle_duration_sec=1.0
            )
        else:
            w_final = w_cmd

        # ========== Linear Velocity Clipping ==========

        # w_final 실행에 필요한 바퀴 속도 계산
        wheel_velocity_for_w = abs(w_final * self.wheel_base * 0.5)
        
        # 남은 RPM으로 허용 가능한 최대 선속도 계산
        max_v_allowed = self.max_linear_velocity - wheel_velocity_for_w
        
        if max_v_allowed < 0:
            max_v_allowed = 0.0
        
        if abs(v_cmd) > max_v_allowed:
            v_final = math.copysign(max_v_allowed, v_cmd)
            
            # RPM 계산 (경고용)
            v_left_temp = v_final - (w_final * self.wheel_base * 0.5)
            v_right_temp = v_final + (w_final * self.wheel_base * 0.5)
            rpm_left_temp = (v_left_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_temp = (v_right_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            self.get_logger().warn(
                f'[MOTOR_SIM_2] LINEAR CLIPPED\n'
                f'  Input: v={v_cmd:.3f}, w={w_final:.3f}\n'
                f'  Max v allowed: {max_v_allowed:.3f} m/s (after w reservation)\n'
                f'  Actual RPM: L={rpm_left_temp:.0f}, R={rpm_right_temp:.0f}\n'
                f'  Output: v={v_final:.3f}, w={w_final:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            v_final = v_cmd

        # ========== Final Verification and Publish ==========
        # 최종 바퀴 속도 계산
        v_left = v_final - (w_final * self.wheel_base * 0.5)
        v_right = v_final + (w_final * self.wheel_base * 0.5)
        
        # RPM 변환
        rpm_left = (v_left * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right = (v_right * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # 최종 안전 검증 (이론적으로는 불필요 디버깅용)
        if abs(rpm_left) > self.max_motor_rpm or abs(rpm_right) > self.max_motor_rpm:
            self.get_logger().error(
                f'[MOTOR_SIM_2] RPM EXCEEDED (LOGIC ERROR)\n'
                f'  Target RPM: L={rpm_left:.0f}, R={rpm_right:.0f}\n'
                f'  Max RPM: {self.max_motor_rpm:.0f}\n'
                f'  v_final={v_final:.3f}, w_final={w_final:.3f}\n'
                f'  Emergency Stop Activated',
            )
            # 긴급 정지
            v_final = 0.0
            w_final = 0.0
            rpm_left = 0.0
            rpm_right = 0.0
        
        # Twist 발행
        output_twist = Twist()
        output_twist.linear.x = v_final
        output_twist.angular.z = w_final
        self.cmd_vel_pub.publish(output_twist)
        
        # 값이 변했을 때만 로그 출력
        if self.prev_v_final != v_final or self.prev_w_final != w_final:
            # 입력 대비 출력 로그
            v_left_cmd = v_cmd - (w_cmd * self.wheel_base * 0.5)
            v_right_cmd = v_cmd + (w_cmd * self.wheel_base * 0.5)
            rpm_left_cmd = (v_left_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_cmd = (v_right_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            self.get_logger().info(
                f'[MOTOR_SIM_2] '
                f'Input: v={v_cmd:.3f}, w={w_cmd:.3f} → '
                f'Output: v={v_final:.3f}, w={w_final:.3f} | '
                f'Target RPM: L={rpm_left_cmd:.0f}, R={rpm_right_cmd:.0f} → '
                f'Actual RPM: L={rpm_left:.0f}, R={rpm_right:.0f} (max={self.max_motor_rpm:.0f})'
            )
            self.prev_v_final = v_final
            self.prev_w_final = w_final
    
    def destroy_node(self):
        self.get_logger().info('[MOTOR_SIM_2] Shutting down...')
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelSim2()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
