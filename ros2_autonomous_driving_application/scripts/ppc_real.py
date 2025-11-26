#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from nav_msgs.msg import Odometry, Path
import math
from sensor_msgs.msg import Imu, NavSatFix
import pyproj
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_matrix
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ========== 파라미터 선언 ==========
        self.declare_parameter('lookahead_distance', 1.5)  # 런치 인자로 주입됨
        
        # Publishers
        
        # 속도
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ppc', 10)
        # plot 용
        self.lookahead_pub = self.create_publisher(PoseStamped, '/ppc/lookahead_point', 10)
        self.state_pub = self.create_publisher(String, '/ppc/state', 10)
        self.idx_pub = self.create_publisher(Int32, '/ppc/lookahead_idx', 10)
        self.heading_error_pub = self.create_publisher(Float64, '/ppc/heading_error', 10)
        self.cte_pub = self.create_publisher(Float64, '/ppc/cte', 10)
        self.waypoint_idx_pub = self.create_publisher(Int32, '/ppc/waypoint_idx', 10)
        self.waypoints_path_pub = self.create_publisher(Path, '/waypoints_path', 10)
        # 미션 완료 시 state machine에 stop 전송
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        # 센서
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)

        # Subscribers
        self.dt_gps = 0.1
        self.dt_imu = 0.01
        self.imu_sub = self.create_subscription(Imu, '/imu_cal', self.imu_callback, 1)

        self.dt_timer = 0.25
        self.timer = self.create_timer(self.dt_timer, self.timer_callback)

        # EKF
        self.global_odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.global_odom_callback, 1)
        
        # ppc enable
        self.ppc_enable_sub = self.create_subscription(Bool, '/ppc/enable', self.ppc_enable_callback, 10)
        # 활성화 상태 변수
        self.is_enabled = False 
        # 정지 명령용 Twist
        self.zero_twist = Twist()

        # 예시 경로점
        self.waypoints = [
            # (-9, 3.75), (9, 3.75),
            # (9, 2.25), (-9, 2.25),
            # (-9, 0.75), (9, 0.75),
            # (9, -0.75), (-9, -0.75),
            # (-9, -2.25), (9, -2.25),
            # (9, -3.75), (-9, -3.75)
            (-9,9), (-3,9),
            (-3,7)
            # (-1,5), (-9,5),
            # (-9,1), (-1,1)
        ]
        
        # 런치 인자에서 받은 lookahead_distance 적용
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.get_logger().info(f'[RUN] Ld={self.lookahead_distance}m')

        self.lookahead_idx = 1
        self.interpolated_waypoints = self.interpolate_waypoints(self.waypoints, self.lookahead_distance)

        # 현재 목표 waypoint index (원본 waypoints 기준)
        self.current_waypoint_idx = 0  # 처음 시작은 waypoint[0] → waypoint[1]로 향함

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.global_x = None
        self.global_y = None
        self.global_yaw = None

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.mag_yaw = None  # 자북 방향(heading) 추정값 (rad)

        self.clock_cnt = 0
        self.clock_cnt_pre = 0
        self.current_time = 0.0
        self.current_time_pre = 0.0

        self.rotate_flag = 0
        self.state = "move"  # "move" 또는 "rotate"
        
        # 완료 판정
        self.mission_completed = False

        # clock이 시작되면 1번만 발행하도록 플래그 추가
        # Waypoints를 Path로 발행
        self.publish_waypoints_path()
        self.waypoints_published = False

    # ========== Timer Loop ==========
    def timer_callback(self):
        # 1. 주행 실행 조건 검사
        if not self.check_run():
            return
        
        # 2. 도착 판정 및 상태 갱신
        self.check_arrival()
        
        # 3. 코너 진입 감지
        self.check_rotate()
        
        # 4. 속도 명령 계산
        twist = Twist()
        if self.state == "move":
            twist = self.move_state()
        elif self.state == "rotate":
            twist = self.rotate_state()
        # 5. 속도 명령 발행
        self.publish_cmd_vel(twist)
        
        # 6. 시각화 데이터 발행
        self.publish_for_plot()
    # ========== Main Control Logic ==========
    def check_run(self):
        # 비활성화 시 정지
        if not self.is_enabled:
            self.cmd_pub.publish(self.zero_twist)
            return False
        
        # 완료 시 정지
        if self.mission_completed:
            self.cmd_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data="STOP"))
            return False
        
        # EKF 위치 정보가 없으면 종료
        if self.global_x is None or self.global_y is None or self.global_yaw is None:
            return False
        
        return True
    
    def check_arrival(self):
        x = self.global_x
        y = self.global_y
        
        min_idx = self.lookahead_idx
        last_idx = len(self.interpolated_waypoints) - 1
        
        if last_idx < 1:
            self.get_logger().warn("[RUN] Not enough waypoints")
            return self.interpolate_waypoints[last_idx]
        # 완료 판정
        if min_idx >= last_idx:
            self.lookahead_idx = last_idx
            prev_wp = self.interpolated_waypoints[last_idx - 1]
            curr_wp = self.interpolated_waypoints[last_idx]
            vec_path = np.array([curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]])
            robot_pos = np.array([x, y])
            vec_robot = np.array([robot_pos[0] - curr_wp[0], robot_pos[1] - curr_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            if dot > 0 and not self.mission_completed:
                self.get_logger().info("[RUN] Reached final waypoint - Mission completed")
                self.mission_completed = True
                # waypoint index를 마지막으로 설정 (plot에서 완료 감지용)
                self.current_waypoint_idx = len(self.waypoints) - 1
            return curr_wp
        
        # 내적기반 도착 판정 + waypoint index 증가
        elif min_idx > 0 and min_idx + 1 < len(self.interpolated_waypoints):
            prev_wp = self.interpolated_waypoints[min_idx - 1]
            curr_wp = self.interpolated_waypoints[min_idx]
            robot_pos = np.array([x, y])
            vec_path = np.array([curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]])
            vec_robot = np.array([robot_pos[0] - curr_wp[0], robot_pos[1] - curr_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            
            if dot > 0:
                if self.rotate_flag == 1:
                    self.rotate_flag = 0
                    self.state = "rotate"
                    self.get_logger().info("[RUN] Entering rotate state")
                    # waypoint 도착 판정 시 index 증가
                    self.current_waypoint_idx += 1
                    self.get_logger().info(
                        f'[RUN] Reached waypoint {self.current_waypoint_idx-1}, '
                        f'[RUN] moving to waypoint {self.current_waypoint_idx}'
                    )
                
                min_idx += 1
        
        # lookahead_point 업데이트
        self.lookahead_idx = min_idx
    
    def check_rotate(self):
        lookahead_point = self.interpolated_waypoints[self.lookahead_idx]
        
        # waypoints 중 하나와 lookahead_point의 거리가 0.01 이하이면 회전 플래그 on
        for wp in self.waypoints:
            if np.hypot(lookahead_point[0] - wp[0], lookahead_point[1] - wp[1]) < 0.01:
                self.rotate_flag = 1
                break
    
    def move_state(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        lookahead_point = self.interpolated_waypoints[self.lookahead_idx]
        
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        ld = np.hypot(dx, dy)
        
        if ld > self.lookahead_distance * 0.25:
            # ========== Pure Pursuit 모드 (부드러움 우선) ==========
            # 차량의 heading 기준 lookahead point의 좌표
            x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
            curvature = 2.0 * y_r / (ld ** 2)
        else:
            # ========== 정렬 모드 (정확성 우선) ==========
            # 다음 실제 웨이포인트를 타겟으로 설정
            if self.current_waypoint_idx < len(self.waypoints):
                target_wp = self.waypoints[self.current_waypoint_idx]
            else:
                # 마지막 웨이포인트 사용
                target_wp = self.waypoints[-1]
            
            # 타겟 웨이포인트 방향 계산
            dx_target = target_wp[0] - x
            dy_target = target_wp[1] - y
            ld_target = np.hypot(dx_target, dy_target)
            
            if ld_target > 0.01:  # 거의 도착한 경우 방지
                # 차량 좌표계로 변환
                x_r_target = math.cos(yaw) * dx_target + math.sin(yaw) * dy_target
                y_r_target = -math.sin(yaw) * dx_target + math.cos(yaw) * dy_target
                # 정렬 곡률 계산
                curvature = 2.0 * y_r_target / (ld_target ** 2)
            else:
                curvature = 0.0
        
        # 속도 명령 생성
        twist = Twist()
        if self.rotate_flag == 1:
            twist.linear.x = 0.1
            twist.angular.z = twist.linear.x * curvature
        else:
            twist.linear.x = 0.3
            twist.angular.z = twist.linear.x * curvature
        
        return twist
    
    def rotate_state(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # 가장 가까운 waypoint(시작점 제외)를 찾음
        min_wp_dist = float('inf')
        target_yaw = yaw
        for i in range(1, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = np.hypot(wp[0] - x, wp[1] - y)
            if dist < min_wp_dist:
                min_wp_dist = dist
                prev_wp = wp
                target_wp = self.waypoints[i + 1]
        
        # prev_wp → target_wp 방향을 목표 yaw로 사용
        target_yaw = math.atan2(target_wp[1] - prev_wp[1], target_wp[0] - prev_wp[0])
        yaw_error = self.normalize_angle(target_yaw - yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        twist = Twist()
        if yaw_error_deg < 1.0:
            self.state = "move"
            self.get_logger().info(
                f" [RUN] Yaw aligned (error {yaw_error_deg:.2f} deg). Switching to move state."
            )
        else:
            twist.linear.x = 0.0
            twist.angular.z = np.sign(yaw_error) * (np.pi / 36)  # 10 deg/s, 방향 고려
            self.get_logger().info(
                f"[RUN] Rotating in place. Yaw error: {yaw_error_deg:.2f} current: {math.degrees(yaw):.2f} deg) | "
            )
        
        return twist

    def publish_cmd_vel(self, twist):
        self.cmd_pub.publish(twist)
    
    def interpolate_waypoints(self, waypoints, lookahead_distance):
        # 최소 경로점 개수
        if len(waypoints) < 2:
            return waypoints

        interpolated = []
        for i in range(len(waypoints) - 1):
            x0, y0 = waypoints[i]
            x1, y1 = waypoints[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            segment_length = np.hypot(dx, dy) # 최소 거리 계산
            num_points = max(int(segment_length // lookahead_distance), 1) # ld 간격 계산, max()로 1 이상 보장
            # 두 점 사이를 일정 간격으로 쪼개서 중간 점 만들기
            for j in range(num_points):
                t = j * lookahead_distance / segment_length
                xi = x0 + t * dx
                yi = y0 + t * dy
                interpolated.append((xi, yi))
        # 마지막 점 추가
        interpolated.append(waypoints[-1])

        return interpolated

    # ========== Input ==========
    def ppc_enable_callback(self, msg: Bool):
        # 상태가 변경될 때만 로그 출력
        if self.is_enabled != msg.data:
            if msg.data:
                # ========== ENABLE (활성화) ==========
                # mission_completed가 True면 → 새 미션 시작 (완전 초기화)
                # mission_completed가 False면 → 일시정지 해제 (진행 상황 유지)
                if self.mission_completed:
                    # 완전히 새로운 미션 시작
                    self.mission_completed = False
                    self.current_waypoint_idx = 0
                    self.lookahead_idx = 1
                    self.state = "move"
                    self.rotate_flag = 0
                    self.get_logger().info('[PPC] ENABLED - Starting NEW mission')
                else:
                    # 일시정지 해제 - 진행 중이던 위치에서 재개
                    self.get_logger().info(
                        f'[PPC] RESUMED - Continuing from waypoint {self.current_waypoint_idx}'
                    )
            else:
                # ========== DISABLE (비활성화) ==========
                # 정지 명령만 발행, 진행 상황은 모두 유지
                self.cmd_pub.publish(self.zero_twist)
                self.get_logger().info(
                    f'[PPC] PAUSED - State preserved (mission_completed={self.mission_completed})'
                )
                
        self.is_enabled = msg.data

    def imu_callback(self, msg):

        # 쿼터니언 -> yaw 변환
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        # self.current_yaw = yaw

    def global_odom_callback(self, msg: Odometry):
        self.global_x = msg.pose.pose.position.x
        self.global_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ========== Utility ==========
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ========== For Plot ==========
    def publish_waypoints_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.waypoints_path_pub.publish(path)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints to /waypoints_path')
    
    def publish_for_plot(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        lookahead_point = self.interpolated_waypoints[self.lookahead_idx]
        
        # CTE 계산 (보간된 경로 기준)
        min_dist = float('inf')
        for i in range(len(self.interpolated_waypoints) - 1):
            x1, y1 = self.interpolated_waypoints[i]
            x2, y2 = self.interpolated_waypoints[i + 1]
            
            dx = x2 - x1
            dy = y2 - y1
            
            if dx == 0 and dy == 0:
                dist = math.hypot(x - x1, y - y1)
            else:
                t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx**2 + dy**2)))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                dist = math.hypot(x - closest_x, y - closest_y)
            
            min_dist = min(min_dist, dist)
        cte = min_dist
        
        # Heading Error 계산
        target_yaw = math.atan2(lookahead_point[1] - y, lookahead_point[0] - x)
        heading_error = self.normalize_angle(target_yaw - yaw)
        
        # Lookahead Point 발행
        lookahead_msg = PoseStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.pose.position.x = float(lookahead_point[0])
        lookahead_msg.pose.position.y = float(lookahead_point[1])
        lookahead_msg.pose.position.z = 0.0
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # State 발행
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # Waypoint Index 발행 (도착 판정 기반)
        waypoint_idx_msg = Int32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_idx_pub.publish(waypoint_idx_msg)
        
        # Lookahead Index 발행
        idx_msg = Int32()
        idx_msg.data = self.lookahead_idx
        self.idx_pub.publish(idx_msg)
        
        # Heading Error 발행
        heading_msg = Float64()
        heading_msg.data = math.degrees(heading_error)
        self.heading_error_pub.publish(heading_msg)
        
        # CTE 발행
        cte_msg = Float64()
        cte_msg.data = cte
        self.cte_pub.publish(cte_msg)



def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
