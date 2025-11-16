#!/usr/bin/env python3
"""선속도 우선"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorCmdVelSim3(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_sim_3')
        
        # ========== 파라미터 선언 ==========
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
        
        self.get_logger().info(
            f'[MOTOR_SIM_3] Virtual Motor Driver (v priority) started\n'
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
            self.get_logger().error('[MOTOR_SIM_3] Wheel circumference is zero!')
            return

        # ========== Linear Velocity Clipping (최우선 확보) ==========
        if abs(v_cmd) > self.max_linear_velocity:
            v_final = math.copysign(self.max_linear_velocity, v_cmd)
            self.get_logger().warn(
                f'[MOTOR_SIM_3] LINEAR CLIPPED\n'
                f'  Input v: {v_cmd:.3f} m/s\n'
                f'  Output v: {v_final:.3f} m/s (Max: {self.max_linear_velocity:.3f})',
                throttle_duration_sec=1.0
            )
        else:
            v_final = v_cmd

        # ========== Angular Velocity Clipping (남은 자원 기반 댐핑) ==========

        # v_final 실행에 필요한 바퀴 속도 계산
        # 선속도만 실행했을 때 양쪽 바퀴는 동일한 속도로 회전
        wheel_velocity_for_v = abs(v_final)
        
        # 남은 자원으로 허용 가능한 최대 각속도 계산
        # 각속도는 좌우 바퀴의 속도 차이에 의해 발생
        # 한쪽 바퀴가 최대 속도까지 올라갈 수 있는 여유분을 이용
        max_wheel_diff = self.max_linear_velocity - wheel_velocity_for_v
        
        # 바퀴 속도 차이를 각속도로 변환
        # w = (v_right - v_left) / wheel_base
        # 최대 차이는 양쪽 바퀴가 모두 여유분을 사용할 때: 2 * max_wheel_diff
        max_w_allowed = (2.0 * max_wheel_diff) / self.wheel_base
        
        if max_w_allowed < 0:
            max_w_allowed = 0.0
        
        if abs(w_cmd) > max_w_allowed:
            w_final = math.copysign(max_w_allowed, w_cmd)
            
            # RPM 계산 (경고용)
            v_left_temp = v_final - (w_final * self.wheel_base * 0.5)
            v_right_temp = v_final + (w_final * self.wheel_base * 0.5)
            rpm_left_temp = (v_left_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_temp = (v_right_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            # 댐핑 효과 강조
            damping_ratio = (max_w_allowed / abs(w_cmd)) * 100 if w_cmd != 0 else 100
            
            self.get_logger().warn(
                f'[MOTOR_SIM_3] ANGULAR CLIPPED\n'
                f'  Input: v={v_final:.3f}, w={w_cmd:.3f}\n'
                f'  Max w allowed: {max_w_allowed:.3f} rad/s (after v reservation)\n'
                f'  Damping ratio: {damping_ratio:.1f}% (w reduced to prevent oscillation)\n'
                f'  Actual RPM: L={rpm_left_temp:.0f}, R={rpm_right_temp:.0f}\n'
                f'  Output: v={v_final:.3f}, w={w_final:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            w_final = w_cmd

        # ========== Final Verification and Publish ==========
        # 최종 바퀴 속도 계산
        v_left = v_final - (w_final * self.wheel_base * 0.5)
        v_right = v_final + (w_final * self.wheel_base * 0.5)
        
        # RPM 변환
        rpm_left = (v_left * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right = (v_right * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # 최종 안전 검증 (이론적으로는 불필요하지만 디버깅용)
        if abs(rpm_left) > self.max_motor_rpm or abs(rpm_right) > self.max_motor_rpm:
            self.get_logger().error(
                f'[MOTOR_SIM_3] RPM EXCEEDED (LOGIC ERROR)\n'
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
        
        # 입력 대비 출력 로그 (RPM 자원 활용률 표시)
        v_left_cmd = v_cmd - (w_cmd * self.wheel_base * 0.5)
        v_right_cmd = v_cmd + (w_cmd * self.wheel_base * 0.5)
        rpm_left_cmd = (v_left_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right_cmd = (v_right_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference

        
        self.get_logger().info(
            f'[MOTOR_SIM_3] '
            f'Input: v={v_cmd:.3f}, w={w_cmd:.3f} → '
            f'Output: v={v_final:.3f}, w={w_final:.3f} | '
            f'Target RPM: L={rpm_left_cmd:.0f}, R={rpm_right_cmd:.0f} → '
            f'Actual RPM: L={rpm_left:.0f}, R={rpm_right:.0f} (max={self.max_motor_rpm:.0f})',
            throttle_duration_sec=0.5
        )
    
    def destroy_node(self):
        self.get_logger().info('[MOTOR_SIM_3] Shutting down...')
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelSim3()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
