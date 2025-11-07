#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from datetime import datetime
import os
import json

class StateMachineExecutor(Node):
    def __init__(self):
        super().__init__('state_machine_executor_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('State Machine Executor has been started.')

        # 1. 상태 변수
        self.current_state = 'INIT' # '관리자'로부터 상태를 받기 전

        # 3. 발행 (Publications)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 1)
        self.zero_twist = Twist() # 정지 명령용 (모든 필드 0)
        self.zero_twist.linear.x = 0.0
        self.zero_twist.linear.y = 0.0
        self.zero_twist.linear.z = 0.0
        self.zero_twist.angular.x = 0.0
        self.zero_twist.angular.y = 0.0
        self.zero_twist.angular.z = 0.0
        self.ppc_enable_pub = self.create_publisher(Bool, '/ppc/enable', 5)
        self.ppc_enable_pub.publish(Bool(data=False))
        self.ppc_enabled = False
        self.ppc_publish_timer = self.create_timer(1.0, self.publish_ppc_enable) # ppc_enable 1초마다 발행

        # 4. 구독 (Subscriptions)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)

        # Calibration I/O
        self.cal_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        self.imu_offset_pub = self.create_publisher(Float64, '/yaw_offset', 10)
        self.yaw_offset_est = 0.0
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)

        # CALIBRATION 상태변수
        self.cal_state = 'IDLE'
        self.current_ekf = None
        
        # CAL 전용 위치 변수
        self.cal_current_x = 0.0
        self.cal_current_y = 0.0
        self.cal_current_yaw = 0.0
        self.cal_points = []
        self.cal_start_time = None
        
        # CAL Parameters # TODO UWB raw data 5Hz 감안해서 시간 설정
        self.forward_time = 5.0
        self.backward_time = 5.0
        self.forward_speed = 0.30
        self.backward_speed = -0.30
        self.calculated_yaw_error = 0.0
        self.max_yaw_correction_deg = 360.0
        
        # CAL Data Logging
        self.cal_data_dir = os.path.expanduser('~/calibration_data')
        os.makedirs(self.cal_data_dir, exist_ok=True)
        self.cal_session_data = None
        
        # ALIGN I/O
        self.align_cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ALIGN 상태 변수
        self.align_state = 'IDLE'
        
        # ALIGN 전용 위치 변수
        self.align_current_x = 0.0
        self.align_current_y = 0.0
        self.align_current_yaw = 0.0
        self.start_pos_for_dot_product = None

        # ALIGN Parameters
        self.align_target_pos = (-9.0, 9.0) # (x, y)
        self.align_target_yaw = 0.0
        self.align_linear_speed = 0.3
        self.align_angular_speed = 0.2
        self.align_yaw_threshold = 0.01 # 약 0.57도
        
        # 타이머 (마지막에 생성)
        self.publish_timer = self.create_timer(0.1, self.state_machine_loop)

    # PPC enable 상태 발행
    def publish_ppc_enable(self):
        """1초마다 PPC enable 상태 발행"""
        self.ppc_enable_pub.publish(Bool(data=self.ppc_enabled))

    # 상태 변경 콜백
    def state_callback(self, msg):
        """마스터 상태 변경 감지 및 진입/이탈 동작 수행"""
        new_state = msg.data
        if new_state == self.current_state:
            return

        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'[STATE_MACHINE] Changed: {old_state} -> {new_state}')

        # === Exit 로직 ===
        # 작업 완료 후 정지 명령 발행
        if old_state == 'RUN':
            self.ppc_enabled = False
            self.get_logger().info('[STATE_MACHINE] RUN: PPC disabled')
        
        if old_state == 'CALIBRATION':
            self.cal_state = 'IDLE'
            self.cal_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] CALIBRATION: Motors stopped')

        if old_state == 'ALIGN':
            self.align_state = 'IDLE'
            self.align_cmd_vel_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] ALIGN: Motors stopped')

        # === Entry 로직 ===
        if new_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] STOP: Emergency Stop')

        elif new_state == 'RUN':
            self.ppc_enabled = True
            self.get_logger().info('[STATE_MACHINE] RUN: PPC enabled')

        elif new_state == 'KEY':
            self.get_logger().info('[STATE_MACHINE] KEY: Manual mode')

        elif new_state == 'CALIBRATION':
            self.cal_points = []
            self.calculated_yaw_error = 0.0
            self.cal_state = 'FORWARD'
            self.cal_start_time = self.get_clock().now()
            
            # 세션 데이터 초기화
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.cal_session_data = {
                'timestamp': timestamp,
                'points': [],
                'parameters': {
                    'forward_time': self.forward_time,
                    'backward_time': self.backward_time,
                    'forward_speed': self.forward_speed,
                    'backward_speed': self.backward_speed
                },
                'results': {}
            }

            self.get_logger().info('[STATE_MACHINE] CALIBRATION: Started')

        elif new_state == 'ALIGN':
            self.align_state = 'ALIGN_ROTATE_1'
            self.start_pos_for_dot_product = None
            self.get_logger().info('[STATE_MACHINE] ALIGN: Started')

    # 상태 머신 루프 0.1초마다 실행
    def state_machine_loop(self):
        """0.1초마다 실행되는 루프"""
        if self.current_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)

        elif self.current_state == 'CALIBRATION':
            self.run_calibration_step()

        elif self.current_state == 'ALIGN':
            self.align_callback()

    # EKF 값 수신 상태별 변수 구분 필요
    # TODO UWB로 cal 하기
    def odom_callback(self, msg):
        """EKF 오도메트리 수신"""
        self.current_ekf = msg

        if self.current_state == 'CALIBRATION':
            self.cal_current_x = msg.pose.pose.position.x
            self.cal_current_y = msg.pose.pose.position.y
            self.cal_current_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
            
            if self.cal_state in ['FORWARD', 'BACKWARD']:
                x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
                if math.isfinite(x) and math.isfinite(y):
                    self.cal_points.append((x, y))
        
        elif self.current_state == 'ALIGN':
            self.align_current_x = msg.pose.pose.position.x
            self.align_current_y = msg.pose.pose.position.y
            self.align_current_yaw = self._yaw_from_quat(msg.pose.pose.orientation)

    # CALIBRATION 단계별 실행
    # TODO 실시간 yaw로 보정
    def run_calibration_step(self):
        """CALIBRATION 단계별 실행"""
        if self.current_ekf is None:
            self.cal_pub.publish(self.zero_twist)
            return
            
        now = self.get_clock().now()
        dt = (now - self.cal_start_time).nanoseconds * 1e-9 if self.cal_start_time else 0.0

        if self.cal_state == 'FORWARD':
            tw = Twist()
            tw.linear.x = self.forward_speed
            self.cal_pub.publish(tw)
            if dt >= self.forward_time:
                self.cal_state = 'BACKWARD'
                self.cal_start_time = now
                self.get_logger().info('[CAL] Forward complete -> Backward')

        elif self.cal_state == 'BACKWARD':
            tw = Twist()
            tw.linear.x = self.backward_speed
            self.cal_pub.publish(tw)
            if dt >= self.backward_time:
                self.cal_state = 'CALCULATING'
                self.cal_start_time = now
                self.cal_pub.publish(self.zero_twist)
                self.get_logger().info('[CAL] Backward complete -> Calculating')
                
        elif self.cal_state == 'CALCULATING':
            if len(self.cal_points) < 50:
                self.get_logger().error('[CAL] Not enough points collected')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))

                return
            
            pts = np.array(self.cal_points, dtype=float)
            pts -= pts.mean(axis=0, keepdims=True)
            C = np.cov(pts.T)
            w, V = np.linalg.eig(C)
            i = int(np.argmax(w))
            vx, vy = V[0, i], V[1, i]
            detected_angle = math.atan2(vy, vx)

            vdir = self._estimate_direction_vector()
            if vdir is not None:
                if (vx * vdir[0] + vy * vdir[1]) < 0.0:
                    detected_angle = self._normalize_angle(detected_angle + math.pi)
                    
            target_axis_angle = 0.0
            self.calculated_yaw_error = self._normalize_angle(detected_angle - target_axis_angle)
            
            # 결과 저장
            if self.cal_session_data is not None:
                self.cal_session_data['results'] = {
                    'num_points': len(self.cal_points),
                    'detected_angle_deg': math.degrees(detected_angle),
                    'yaw_error_deg': math.degrees(self.calculated_yaw_error),
                    'eigenvalues': w.tolist(),
                    'eigenvector': [float(vx), float(vy)],
                    'direction_vector': [float(vdir[0]), float(vdir[1])] if vdir else None,
                    'covariance_matrix': C.tolist()
                }
            
            self.get_logger().info(f'[CAL] Detected angle={math.degrees(detected_angle):.1f}°, Error={math.degrees(self.calculated_yaw_error):.1f}°')
            self.cal_state = 'CORRECTING'

        elif self.cal_state == 'CORRECTING':
            yaw_err = self.calculated_yaw_error
            if abs(yaw_err) > math.radians(self.max_yaw_correction_deg):
                self.get_logger().warn(f'[CAL] Error too large: {math.degrees(yaw_err):.1f}° -> STOP')
                self.command_pub.publish(String(data='STOP'))
                self.cal_state = 'FINISHED'
                self._save_calibration_data(success=False, reason='error_too_large')
                return
            
            self.imu_offset_pub.publish(Float64(data=yaw_err))
            self.yaw_offset_est = self._normalize_angle(self.yaw_offset_est + yaw_err)
            self.get_logger().info(f'[CAL] Offset applied: {math.degrees(yaw_err):.1f}°')
            
            self.cal_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data='STOP'))
            self.cal_state = 'FINISHED'
            self._save_calibration_data(success=True)

    # 각도 정규화 및 벡터 추정 유틸리티
    def _normalize_angle(self, a):
        """각도를 -pi ~ pi 범위로 정규화"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
        
    def _yaw_from_quat(self, q):
        """쿼터니언에서 Yaw 각도 추출"""
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
    # 방향 벡터 추정
    def _estimate_direction_vector(self):
        """수집된 점들로부터 주행 방향 벡터 추정"""
        if len(self.cal_points) < 10:
            return None
        k = min(5, len(self.cal_points)//10)
        p0 = np.mean(self.cal_points[:k], axis=0)
        p1 = np.mean(self.cal_points[-k:], axis=0)
        dx, dy = (p1[0] - p0[0], p1[1] - p0[1])
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return None
        return (dx/n, dy/n)

    # CAL 데이터 저장
    def _save_calibration_data(self, success=True, reason=''):
        """캘리브레이션 데이터를 파일로 저장"""
        if self.cal_session_data is None:
            return
        
        # 포인트 데이터 저장
        self.cal_session_data['points'] = [[float(x), float(y)] for x, y in self.cal_points]
        self.cal_session_data['success'] = success
        self.cal_session_data['failure_reason'] = reason
        
        timestamp = self.cal_session_data['timestamp']
        
        # JSON 파일로 저장
        json_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}.json')
        with open(json_file, 'w') as f:
            json.dump(self.cal_session_data, f, indent=2)
        
        # NumPy 배열로도 저장 (빠른 로딩용)
        npy_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}.npy')
        np.save(npy_file, np.array(self.cal_points))
        
        self.get_logger().info(f'[CAL] Data saved: {json_file}')
        self.get_logger().info(f'[CAL] Points saved: {npy_file}')

    # ALIGN 단계별 실행
    # TODO align 제어
    def align_callback(self):
        """ALIGN 단계별 실행"""
        if self.current_ekf is None:
            self.get_logger().warn('[ALIGN] No odometry data')
            self.align_cmd_vel_pub.publish(self.zero_twist)
            return
        
        twist_msg = Twist()
        
        if self.align_state == 'ALIGN_ROTATE_1':
            target_pos = self.align_target_pos
            angle_to_target = math.atan2(
                target_pos[1] - self.align_current_y,
                target_pos[0] - self.align_current_x)
            yaw_error = self._normalize_angle(angle_to_target - self.align_current_yaw)
            
            self.get_logger().info(f'[ALIGN] ROTATE_1: Error={math.degrees(yaw_error):.1f}°')
            
            if abs(yaw_error) < self.align_yaw_threshold:
                self.align_state = 'ALIGN_FORWARD'
                self.start_pos_for_dot_product = (self.align_current_x, self.align_current_y)
                self.get_logger().info('[ALIGN] Rotation complete -> Forward')
            else:
                twist_msg.angular.z = np.clip(yaw_error, -self.align_angular_speed, self.align_angular_speed)

        elif self.align_state == 'ALIGN_FORWARD':
            target_pos = self.align_target_pos
            prev_wp = np.array(self.start_pos_for_dot_product)
            curr_wp = np.array(target_pos)
            robot_pos = np.array([self.align_current_x, self.align_current_y])

            vec_path = curr_wp - prev_wp
            vec_robot = robot_pos - curr_wp

            if np.linalg.norm(vec_path) < 0.01:
                dot = 1.0
            else:
                vec_path_norm = vec_path / np.linalg.norm(vec_path)
                dot = np.dot(vec_path_norm, vec_robot)
            
            distance = np.linalg.norm(robot_pos - curr_wp)
            self.get_logger().info(f'[ALIGN] FORWARD: Distance={distance:.2f}m, Dot={dot:.3f}')
            
            if dot > 0.0:
                self.align_state = 'ALIGN_ROTATE_2'
                self.get_logger().info('[ALIGN] Forward complete -> Final rotation')
            else:
                twist_msg.linear.x = self.align_linear_speed
                
        elif self.align_state == 'ALIGN_ROTATE_2':
            target_yaw = self.align_target_yaw
            yaw_error = self._normalize_angle(target_yaw - self.align_current_yaw)
            
            self.get_logger().info(f'[ALIGN] ROTATE_2: Error={math.degrees(yaw_error):.1f}°')
            
            if abs(yaw_error) < self.align_yaw_threshold:
                self.align_state = 'Finished'
                self.get_logger().info('[ALIGN] Final rotation complete -> Finished')
            else:
                twist_msg.angular.z = np.clip(yaw_error, -self.align_angular_speed, self.align_angular_speed)
                
        elif self.align_state == 'Finished':
            self.get_logger().info('[ALIGN] Alignment finished -> STOP')
            self.command_pub.publish(String(data='STOP'))
            self.align_cmd_vel_pub.publish(self.zero_twist)
            return
        
        elif self.align_state == 'IDLE':
            return
            
        self.align_cmd_vel_pub.publish(twist_msg)



def main(args=None):
    rclpy.init(args=args)
    state_machine_executor = StateMachineExecutor()
    rclpy.spin(state_machine_executor)
    state_machine_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()