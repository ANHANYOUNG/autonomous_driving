#!/usr/bin/env python3
"""Motor clipping control, Linear priority, Sim"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorCmdVelSim3(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_sim_3')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 1.5)
        self.declare_parameter('gear_ratio', 60.0)
        self.declare_parameter('max_motor_rpm', 3000.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        
        # Subscription
        self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel_ppc',self.cmd_vel_callback,10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius

        # Max linear vel, angular vel
        self.max_linear_velocity = 0.35

        max_wheel_velocity = self.max_linear_velocity
        self.max_angular_velocity = (2.0 * max_wheel_velocity) / self.wheel_base
        
        # Previous command for change detection
        self.prev_v_final = None
        self.prev_w_final = None
        
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

        # Linear Velocity Clipping
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

        # Angular Velocity Clipping (Damping based on remaining RPM)

        # Calculate wheel velocity required for v_final
        # When only linear velocity is executed, both wheels rotate at the same speed
        wheel_velocity_for_v = abs(v_final)
        
        # Calculate max allowed angular velocity with remaining RPM
        max_wheel_diff = self.max_linear_velocity - wheel_velocity_for_v
        
        # Convert wheel speed difference to angular velocity
        # w = (v_right - v_left) / wheel_base
        # Maximum difference occurs when both wheels use all remaining RPM: 2 * max_wheel_diff
        max_w_allowed = (2.0 * max_wheel_diff) / self.wheel_base
        
        if max_w_allowed < 0:
            max_w_allowed = 0.0
        
        if abs(w_cmd) > max_w_allowed:
            w_final = math.copysign(max_w_allowed, w_cmd)
            
            # RPM calculation (for warning)
            v_left_temp = v_final - (w_final * self.wheel_base * 0.5)
            v_right_temp = v_final + (w_final * self.wheel_base * 0.5)
            rpm_left_temp = (v_left_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_temp = (v_right_temp * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            # Emphasize damping effect
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

        # Final Verification and Publish
        # Calculate final wheel velocities
        v_left = v_final - (w_final * self.wheel_base * 0.5)
        v_right = v_final + (w_final * self.wheel_base * 0.5)
        
        # Convert to RPM
        rpm_left = (v_left * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right = (v_right * 60.0 * self.gear_ratio) / self.wheel_circumference
        
        # Final safety check (theoretically unnecessary but for debugging)
        if abs(rpm_left) > self.max_motor_rpm or abs(rpm_right) > self.max_motor_rpm:
            self.get_logger().error(
                f'[MOTOR_SIM_3] RPM EXCEEDED (LOGIC ERROR)\n'
                f'  Target RPM: L={rpm_left:.0f}, R={rpm_right:.0f}\n'
                f'  Max RPM: {self.max_motor_rpm:.0f}\n'
                f'  v_final={v_final:.3f}, w_final={w_final:.3f}\n'
                f'  Emergency Stop Activated',
            )
            # Emergency Stop
            v_final = 0.0
            w_final = 0.0
            rpm_left = 0.0
            rpm_right = 0.0
        
        # Publish Twist
        output_twist = Twist()
        output_twist.linear.x = v_final
        output_twist.angular.z = w_final
        self.cmd_vel_pub.publish(output_twist)
        
        # Log only when values change
        if self.prev_v_final != v_final or self.prev_w_final != w_final:
            # Input vs Output log
            v_left_cmd = v_cmd - (w_cmd * self.wheel_base * 0.5)
            v_right_cmd = v_cmd + (w_cmd * self.wheel_base * 0.5)
            rpm_left_cmd = (v_left_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_cmd = (v_right_cmd * 60.0 * self.gear_ratio) / self.wheel_circumference
            
            self.get_logger().info(
                f'[MOTOR_SIM_3] '
                f'Input: v={v_cmd:.3f}, w={w_cmd:.3f} → '
                f'Output: v={v_final:.3f}, w={w_final:.3f} | '
                f'Target RPM: L={rpm_left_cmd:.0f}, R={rpm_right_cmd:.0f} → '
                f'Actual RPM: L={rpm_left:.0f}, R={rpm_right:.0f} (max={self.max_motor_rpm:.0f})'
            )
            self.prev_v_final = v_final
            self.prev_w_final = w_final
    
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
