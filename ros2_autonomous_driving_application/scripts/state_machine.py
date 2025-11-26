#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
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
        
        # 사람 감지 토픽 구독
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.person_detected = False  # 사람 감지 플래그

        # Calibration I/O
        self.cal_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        self.imu_offset_pub = self.create_publisher(Float64, '/yaw_offset', 10)
        self.yaw_offset_est = 0.0
        
        # UWB + IMU 직접 구독 (EKF 배제)
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/abs_xy', self.uwb_callback, 10)
        self.imu_cal_sub = self.create_subscription(
            Imu, '/imu_cal', self.imu_cal_callback, 10)
        
        # EKF는 ALIGN 상태에서만 사용
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)

        # CALIBRATION 상태변수
        self.cal_state = 'IDLE'
        self.current_ekf = None
        
        # CAL 전용: 따로 모으는 데이터 리스트
        self.cal_uwb_points = []  # [(x, y), ...] UWB 위치 데이터
        self.cal_imu_yaws = []    # [yaw, ...] IMU Yaw 데이터
        
        # CAL 반복별 구간 인덱스 기록
        self.cal_rep_indices = []  # [(forward_start, forward_end, backward_end), ...]
        self.cal_rep_start_idx = 0  # 현재 반복의 시작 인덱스
        
        # CAL 전용 위치 변수 (제거 - UWB/IMU 직접 사용)
        # self.cal_current_x = 0.0
        # self.cal_current_y = 0.0
        # self.cal_current_yaw = 0.0
        # self.cal_points = []
        self.cal_start_time = None
        
        # CAL 시간 보상 (사람 감지 시 스톱워치 멈춤)
        self.cal_pause_start_time = None  # 일시정지 시작 시각
        self.cal_total_paused_duration = 0.0  # 누적 일시정지 시간 (초)
        
        # CAL Parameters # TODO UWB raw data 5Hz 감안해서 시간 설정
        self.forward_time = 5.0
        self.backward_time = 5.0
        self.forward_speed = 0.30
        self.backward_speed = -0.30
        self.num_repetitions = 3  # ★ 전후진 반복 횟수 (변경 가능)
        self.calculated_yaw_error = 0.0
        self.max_yaw_correction_deg = 360.0
        
        # CAL 반복 카운터
        self.cal_current_rep = 0  # 현재 반복 횟수 (0부터 시작)
        
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

    # ========== State Machine Loop ==========
    def state_machine_loop(self):
        """0.1초마다 실행되는 루프"""
        # ========== 검문소: 사람 감지 시 일시정지 ==========
        if self.person_detected:
            # 모든 상태에서 즉시 정지 (상태는 유지)
            self.stop_pub.publish(self.zero_twist)
            
            # PPC 일시정지 (진행 상황은 유지하되 움직임만 멈춤)
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=False))
            
            return  # 이후 로직 건너뛰기
        
        # ========== 사람 없을 때만 정상 동작 수행 ==========
        # RUN 상태일 때 PPC 재활성화
        if self.current_state == 'RUN':
            self.ppc_enable_pub.publish(Bool(data=True))
        
        if self.current_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)

        elif self.current_state == 'CALIBRATION':
            self.run_calibration_step()

        elif self.current_state == 'ALIGN':
            self.align_callback()

    # ========== ppc_enable ==========
    def publish_ppc_enable(self):
        """1초마다 PPC enable 상태 발행"""
        if self.ppc_enabled and not self.person_detected:
            self.ppc_enable_pub.publish(Bool(data=self.ppc_enabled))
        else:
            self.ppc_enable_pub.publish(Bool(data=False))

    # ========== State Transition ==========
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
            self.cal_uwb_points = []
            self.cal_imu_yaws = []
            self.cal_rep_indices = []  # 반복별 구간 인덱스 초기화
            self.cal_rep_start_idx = 0  # 시작 인덱스 초기화
            self.calculated_yaw_error = 0.0
            self.cal_state = 'FORWARD'
            self.cal_start_time = self.get_clock().now()
            
            # 시간 보상 변수 초기화
            self.cal_pause_start_time = None
            self.cal_total_paused_duration = 0.0
            
            # 반복 카운터 초기화
            self.cal_current_rep = 0
            
            # 세션 데이터 초기화
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.cal_session_data = {
                'timestamp': timestamp,
                'uwb_points': [],  # [(x, y), ...]
                'imu_yaws': [],    # [yaw, ...]
                'parameters': {
                    'forward_time': self.forward_time,
                    'backward_time': self.backward_time,
                    'forward_speed': self.forward_speed,
                    'backward_speed': self.backward_speed,
                    'num_repetitions': self.num_repetitions  # ★ 파라미터 기록
                },
                'results': {}
            }

            self.get_logger().info(
                f'[STATE_MACHINE] CALIBRATION: Started ({self.num_repetitions} repetitions)'
            )

        elif new_state == 'ALIGN':
            self.align_state = 'ALIGN_ROTATE_1'
            self.start_pos_for_dot_product = None
            self.get_logger().info('[STATE_MACHINE] ALIGN: Started')

    # ========== Person Detected ==========
    def person_detected_callback(self, msg: Bool):
        """사람 감지 상태 업데이트 - 상태 변화만 로그"""
        was_detected = self.person_detected
        self.person_detected = msg.data
        
        # 상태 변화 시에만 로그 출력 및 시간 보상
        if not was_detected and self.person_detected:
            # ========== 사람 감지: 일시정지 시작 ==========
            self.get_logger().warn('[SAFETY] Person DETECTED - PAUSING all operations')
            
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=False))
            
            # CALIBRATION 중이면 스톱워치 멈춤
            if self.current_state == 'CALIBRATION' and self.cal_state in ['FORWARD', 'BACKWARD']:
                self.cal_pause_start_time = self.get_clock().now()
                self.get_logger().info('[CAL] Stopwatch PAUSED')
                
        elif was_detected and not self.person_detected:
            # ========== 사람 사라짐: 재개 ==========
            self.get_logger().info('[SAFETY] Person CLEARED - RESUMING operations')
            
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=True))
            
            # CALIBRATION 중이면 시간 보상 (스톱워치 시간 이동)
            if self.current_state == 'CALIBRATION' and self.cal_pause_start_time is not None:
                now = self.get_clock().now()
                paused_duration = (now - self.cal_pause_start_time).nanoseconds * 1e-9
                self.cal_total_paused_duration += paused_duration
                
                self.get_logger().info(
                    f'[CAL] Stopwatch RESUMED - Paused for {paused_duration:.1f}s '
                    f'(Total paused: {self.cal_total_paused_duration:.1f}s)'
                )
                
                self.cal_pause_start_time = None

    # EKF 값 수신 상태별 변수 구분 필요
    # ========== Sensor & Input ==========
    def odom_callback(self, msg):
        """EKF 오도메트리 수신"""
        self.current_ekf = msg

        if self.current_state == 'ALIGN':
            self.align_current_x = msg.pose.pose.position.x
            self.align_current_y = msg.pose.pose.position.y
            self.align_current_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
    
    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        """UWB 절대 위치 수신 (/abs_xy)"""
        if self.current_state != 'CALIBRATION':
            return
        
        if self.cal_state not in ['FORWARD', 'BACKWARD']:
            return
        
        # 사람 감지 시 기록 중단
        if self.person_detected:
            return
        
        # 위치만 저장 (타임스탬프 신경 안 씀)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 유효성 검사
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        
        self.cal_uwb_points.append((x, y))
        
        # 로그 (1초마다)
        if len(self.cal_uwb_points) % 10 == 0:
            self.get_logger().info(
                f'[CAL] UWB data collected: {len(self.cal_uwb_points)} points'
            )
    
    def imu_cal_callback(self, msg: Imu):
        """IMU 보정 결과 수신 (/imu_cal)"""
        if self.current_state != 'CALIBRATION':
            return
        
        if self.cal_state not in ['FORWARD', 'BACKWARD']:
            return
        
        # 사람 감지 시 기록 중단
        if self.person_detected:
            return
        
        # Yaw만 저장 (타임스탬프 신경 안 씀)
        yaw = self._yaw_from_quat(msg.orientation)
        self.cal_imu_yaws.append(yaw)
        
        # 로그 (1초마다)
        if len(self.cal_imu_yaws) % 100 == 0:
            self.get_logger().info(
                f'[CAL] IMU data collected: {len(self.cal_imu_yaws)} yaw values'
            )

    # ========== Calibration ==========
    def run_calibration_step(self):
        """CALIBRATION 단계별 실행"""
        if self.current_state != 'CALIBRATION':
            return
            
        now = self.get_clock().now()
        
        # 실제 경과 시간 = (현재 시각 - 시작 시각) - 누적 일시정지 시간
        raw_elapsed = (now - self.cal_start_time).nanoseconds * 1e-9 if self.cal_start_time else 0.0
        dt = raw_elapsed - self.cal_total_paused_duration

        if self.cal_state == 'FORWARD':
            tw = Twist()
            tw.linear.x = self.forward_speed
            self.cal_pub.publish(tw)
            
            if dt >= self.forward_time:
                # Forward 구간 종료 인덱스 기록
                forward_end_idx = len(self.cal_uwb_points) - 1
                
                self.cal_state = 'BACKWARD'
                self.cal_start_time = now
                self.get_logger().info(
                    f'[CAL] Rep {self.cal_current_rep + 1}/{self.num_repetitions}: Forward complete -> Backward\n'
                    f'  Forward range: [{self.cal_rep_start_idx}, {forward_end_idx}] ({forward_end_idx - self.cal_rep_start_idx + 1} points)\n'
                    f'  Total - UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                )
                
                # 임시로 forward_end_idx 저장 (backward 끝나면 완성)
                self.cal_forward_end_idx = forward_end_idx

        elif self.cal_state == 'BACKWARD':
            tw = Twist()
            tw.linear.x = self.backward_speed
            self.cal_pub.publish(tw)
            
            if dt >= self.backward_time:
                # Backward 구간 종료 인덱스 기록
                backward_end_idx = len(self.cal_uwb_points) - 1
                
                # 현재 반복의 구간 인덱스 저장
                self.cal_rep_indices.append({
                    'rep_num': self.cal_current_rep + 1,
                    'forward_start': self.cal_rep_start_idx,
                    'forward_end': self.cal_forward_end_idx,
                    'backward_end': backward_end_idx,
                    'num_forward_points': self.cal_forward_end_idx - self.cal_rep_start_idx + 1,
                    'num_backward_points': backward_end_idx - self.cal_forward_end_idx
                })
                
                # 반복 카운터 증가
                self.cal_current_rep += 1
                
                # 반복 완료 여부 확인
                if self.cal_current_rep >= self.num_repetitions:
                    # ★ 모든 반복 완료 → 계산 단계로
                    self.cal_state = 'CALCULATING'
                    self.cal_start_time = now
                    self.cal_pub.publish(self.zero_twist)
                    self.get_logger().info(
                        f'[CAL] All {self.num_repetitions} repetitions complete -> Calculating\n'
                        f'  Backward range: [{self.cal_forward_end_idx + 1}, {backward_end_idx}] ({backward_end_idx - self.cal_forward_end_idx} points)\n'
                        f'  Total UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                    )
                else:
                    # ★ 다음 반복 시작 → FORWARD로 복귀
                    # 다음 반복의 시작 인덱스 설정
                    self.cal_rep_start_idx = backward_end_idx + 1
                    
                    self.cal_state = 'FORWARD'
                    self.cal_start_time = now
                    self.get_logger().info(
                        f'[CAL] Rep {self.cal_current_rep}/{self.num_repetitions}: Backward complete -> Next Forward\n'
                        f'  Backward range: [{self.cal_forward_end_idx + 1}, {backward_end_idx}] ({backward_end_idx - self.cal_forward_end_idx} points)\n'
                        f'  Next rep starts at index {self.cal_rep_start_idx}\n'
                        f'  Total - UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                    )
                
        elif self.cal_state == 'CALCULATING':
            # 최소 데이터 확인
            if len(self.cal_uwb_points) < 20:
                self.get_logger().error(f'[CAL] Not enough UWB data: {len(self.cal_uwb_points)} < 20')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient_uwb_data')
                return
            
            if len(self.cal_imu_yaws) < 50:
                self.get_logger().error(f'[CAL] Not enough IMU data: {len(self.cal_imu_yaws)} < 50')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient_imu_data')
                return
            
            # UWB 각도
            # 방법: PCA로 좌표들의 주성분(방향 벡터) 구한 후 각도 계산
            uwb_array = np.array(self.cal_uwb_points)
            
            if len(uwb_array) < 20:
                self.get_logger().error(f'[CAL] Not enough UWB data: {len(uwb_array)} < 20')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient UWB data')
                return
            
            # 최대 거리 지점 기준 Forward 구간 Backward 구간 구분
            start_pt = uwb_array[0]
            distances = np.linalg.norm(uwb_array - start_pt, axis=1)
            max_dist_idx = np.argmax(distances)  # 최대 거리 지점 (반환점)
            uwb_forward = uwb_array[:max_dist_idx+1]
            
            # 전체 구간으로 PCA 수행
            mean_pt = np.mean(uwb_array, axis=0)
            centered = uwb_array - mean_pt
            
            # SVD로 주성분 추출
            U, S, Vt = np.linalg.svd(centered, full_matrices=False)
            pc1 = Vt[0]  # 첫 번째 주성분 (방향 벡터)
            
            # 방향 일관성: Forward 시작->끝 방향과 일치하도록 조정 (전진 방향)
            end_pt = uwb_forward[-1]  # start_pt는 위에서 이미 계산됨
            forward_vec = end_pt - start_pt
            if np.dot(pc1, forward_vec) < 0:
                pc1 = -pc1  # PCA 방향을 전진 방향으로 뒤집기
            
            # Forward 주행 거리 확인 (시작 -> forward 끝)
            forward_distance = math.hypot(forward_vec[0], forward_vec[1])
            
            # Backward 주행 거리 확인 (반환점 -> 끝)
            backward_vec = uwb_array[-1] - end_pt
            backward_distance = math.hypot(backward_vec[0], backward_vec[1])
            
            spread_along_pc = np.std(np.dot(centered, pc1))
            
            # UWB 정밀도: PCA 선에서 수직 거리의 절댓값 평균 (mm)
            residuals = np.cross(centered, pc1)  # 부호 있는 잔차
            uwb_precision_mm = np.mean(np.abs(residuals)) * 1000  # mm 단위
            
            if forward_distance < 0.5:  # Forward 주행 거리가 너무 짧음
                self.get_logger().error(f'[CAL] Forward distance too short: {forward_distance:.2f}m < 0.5m')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='short_forward_distance')
                return
            
            # PCA 방향 벡터로부터 각도 계산 (전진 방향)
            uwb_angle = math.atan2(pc1[1], pc1[0])  # UWB 경로 각도 (rad)
            
            # IMU 각도
            # 방법: 각 yaw를 단위벡터로 변환 후 평균 벡터의 각도 계산
            imu_array = np.array(self.cal_imu_yaws)
            
            # 각도를 2D 단위벡터로 변환: (cos(yaw), sin(yaw))
            imu_vectors = np.column_stack([np.cos(imu_array), np.sin(imu_array)])  # (N, 2)
            
            # 벡터 평균 (방향의 평균)
            imu_mean = np.mean(imu_vectors, axis=0)
            
            # 평균 벡터로부터 각도 계산
            imu_angle = math.atan2(imu_mean[1], imu_mean[0])  # IMU 방향 각도 (rad)
            
            # IMU 편차 계산 (각도 공간에서의 표준편차)
            angle_diffs = np.array([self._normalize_angle(yaw - imu_angle) for yaw in imu_array])
            imu_std_dev = np.std(angle_diffs)
            
            # Offset 각도
            # Yaw 오프셋 = (UWB) - (IMU)
            yaw_offset_angle = self._normalize_angle(uwb_angle - imu_angle)
            
            self.calculated_yaw_error = yaw_offset_angle
            
            # 결과 저장
            if self.cal_session_data is not None:
                self.cal_session_data['results'] = {
                    'num_repetitions_completed': self.cal_current_rep,  # ★ 완료된 반복 횟수
                    'repetition_indices': self.cal_rep_indices,  # ★ 각 반복의 구간 정보
                    'num_uwb_points': len(self.cal_uwb_points),
                    'num_uwb_forward': len(uwb_forward),
                    'max_dist_idx': int(max_dist_idx),
                    'num_imu_yaws': len(self.cal_imu_yaws),
                    'forward_distance_m': float(forward_distance),
                    'backward_distance_m': float(backward_distance),
                    'uwb_angle_deg': float(math.degrees(uwb_angle)),
                    'imu_angle_deg': float(math.degrees(imu_angle)),
                    'yaw_offset_angle_deg': float(math.degrees(yaw_offset_angle)),
                    'uwb_spread_along_pc': float(spread_along_pc),
                    'uwb_precision_mm': float(uwb_precision_mm),
                    'imu_std_dev_deg': float(math.degrees(imu_std_dev)),
                    'start_point': [float(start_pt[0]), float(start_pt[1])],
                    'forward_end_point': [float(end_pt[0]), float(end_pt[1])],
                    'pca_direction_uwb': [float(pc1[0]), float(pc1[1])],
                    'mean_direction_imu': [float(imu_mean[0]), float(imu_mean[1])]
                }
            
            self.get_logger().info(
                f'[CAL] Analysis complete:\n'
                f'  UWB Angle: {math.degrees(uwb_angle):.2f}° (from {len(uwb_array)} total UWB points)\n'
                f'  IMU Angle: {math.degrees(imu_angle):.2f}° (from {len(self.cal_imu_yaws)} IMU values)\n'
                f'  Yaw Offset Angle: {math.degrees(yaw_offset_angle):.2f}°\n'
                f'  Forward distance: {forward_distance:.2f}m, Backward distance: {backward_distance:.2f}m\n'
            )
            self.cal_state = 'CORRECTING'

        elif self.cal_state == 'CORRECTING':
            yaw_err = self.calculated_yaw_error
            if abs(yaw_err) > math.radians(self.max_yaw_correction_deg):
                self.get_logger().warn(f'[CAL] Error too large: {math.degrees(yaw_err):.1f}° > {self.max_yaw_correction_deg}° -> STOP')
                self.command_pub.publish(String(data='STOP'))
                self.cal_state = 'FINISHED'
                self._save_calibration_data(success=False, reason='error_too_large')
                return
            
            # 증분 오차를 /yaw_offset 토픽으로 발행
            # imu_offset 노드가 기존 오프셋에 이 값을 더함 (누적)
            self.imu_offset_pub.publish(Float64(data=yaw_err))
            self.yaw_offset_est = self._normalize_angle(self.yaw_offset_est + yaw_err)
            
            self.get_logger().info(
                f'[CAL] Offset published: {math.degrees(yaw_err):.2f}°\n'
                f'  Total offset: {math.degrees(self.yaw_offset_est):.2f}°'
            )
            
            self.cal_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data='STOP'))
            self.cal_state = 'FINISHED'
            self._save_calibration_data(success=True)

    def _save_calibration_data(self, success=True, reason=''):
        """캘리브레이션 데이터를 파일로 저장"""
        if self.cal_session_data is None:
            return
        
        # UWB와 IMU 데이터를 별도로 저장
        self.cal_session_data['uwb_points'] = [
            [float(x), float(y)] for x, y in self.cal_uwb_points
        ]
        self.cal_session_data['imu_yaws'] = [
            float(yaw) for yaw in self.cal_imu_yaws
        ]
        self.cal_session_data['success'] = success
        self.cal_session_data['failure_reason'] = reason
        
        timestamp = self.cal_session_data['timestamp']
        
        # JSON 파일로 저장
        json_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}.json')
        with open(json_file, 'w') as f:
            json.dump(self.cal_session_data, f, indent=2)
        
        # NumPy 배열로도 저장 (빠른 로딩용)
        npy_uwb_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}_uwb.npy')
        npy_imu_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}_imu.npy')
        np.save(npy_uwb_file, np.array(self.cal_uwb_points))
        np.save(npy_imu_file, np.array(self.cal_imu_yaws))
        
        self.get_logger().info(f'[CAL] Data saved: {json_file}')
        self.get_logger().info(f'[CAL] UWB data: {npy_uwb_file} ({len(self.cal_uwb_points)} points)')
        self.get_logger().info(f'[CAL] IMU data: {npy_imu_file} ({len(self.cal_imu_yaws)} yaw values)')

    # ========== Align ==========
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

    # ========== Utility ==========
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
    
def main(args=None):
    rclpy.init(args=args)
    state_machine_executor = StateMachineExecutor()
    rclpy.spin(state_machine_executor)
    state_machine_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()