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
from sensor_msgs.msg import MagneticField
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node_v2')

        # ========== 파라미터 선언 ==========
        self.declare_parameter('lookahead_distance', 1.5)  # 런치 인자로 주입됨
        
        # Publishers
        
        # 속도
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ppc', 10)
        # plot 용
        self.lookahead_pub = self.create_publisher(PoseStamped, '/ppc/lookahead_point', 10)
        self.state_pub = self.create_publisher(String, '/ppc/state', 10)
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
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 1)

        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

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
            (-9,9), (-5,9),
            (-5,7), (-8,7)
            # (-1,5), (-9,5),
            # (-9,1), (-1,1)
        ]
        
        # 런치 인자에서 받은 lookahead_distance 적용
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.get_logger().info(f'[RUN] Line Following PPC, Ld={self.lookahead_distance}m')

        # 현재 목표 waypoint index (원본 waypoints 기준)
        self.current_waypoint_idx = 1  # 처음 시작은 waypoint[0] → waypoint[1]로 향함

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.global_x = None
        self.global_y = None
        self.global_yaw = None

        # 기준점 위도, 경도 (예시: world의 (0,0)에 해당하는 위경도)
        origin_lat = 37.0
        origin_lon = 127.0

        # WGS84 -> ENU 변환기 생성
        self.enu_converter = pyproj.Transformer.from_crs(
            crs_from="epsg:4326",
            crs_to=pyproj.CRS.from_proj4(
                f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84"
            ),
            always_xy=True
        )

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
        """내적 기반 도착 판정 - 목표 웨이포인트의 수직 평면을 통과했는지 검사"""
        x = self.global_x
        y = self.global_y
        
        # 마지막 waypoint 도달 판정
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            target_wp = self.waypoints[-1]
            prev_wp = self.waypoints[-2]
            
            # 경로 벡터 (이전 → 목표)
            vec_path = np.array([target_wp[0] - prev_wp[0], target_wp[1] - prev_wp[1]])
            # 로봇 벡터 (목표 → 로봇)
            vec_robot = np.array([x - target_wp[0], y - target_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            
            if dot > 0 and not self.mission_completed:
                self.get_logger().info("[RUN] Reached final waypoint - Mission completed")
                self.mission_completed = True
                self.current_waypoint_idx = len(self.waypoints) - 1
            return
        
        # 중간 waypoint 도착 판정
        target_wp = self.waypoints[self.current_waypoint_idx]
        prev_wp = self.waypoints[self.current_waypoint_idx - 1]
        
        vec_path = np.array([target_wp[0] - prev_wp[0], target_wp[1] - prev_wp[1]])
        vec_robot = np.array([x - target_wp[0], y - target_wp[1]])
        dot = np.dot(vec_path, vec_robot)
        
        if dot > 0:
            # 수직 평면 통과 → 다음 waypoint로 이동
            if self.rotate_flag == 1:
                self.rotate_flag = 0
                self.state = "rotate"
                self.get_logger().info("[RUN] Entering rotate state")
            
            self.get_logger().info(
                f'[RUN] Reached waypoint {self.current_waypoint_idx}, '
                f'moving to waypoint {self.current_waypoint_idx + 1}'
            )
            self.current_waypoint_idx += 1
    
    def check_rotate(self):
        """다음 waypoint가 코너인지 판정"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        target_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_target = np.hypot(target_wp[0] - self.global_x, target_wp[1] - self.global_y)
        
        # 목표 waypoint에 충분히 가까우면 회전 준비
        if dist_to_target < self.lookahead_distance * 0.5:
            self.rotate_flag = 1
    
    def intersection_point(self, p1, p2, robot_x, robot_y, ld):
        """
        로봇 반경(Ld)과 무한한 직선의 교점을 구하고,
        내적(Dot Product)을 통해 진행 방향의 점을 선택하는 핵심 함수
        
        Args:
            p1: 선분 시작점 (x, y)
            p2: 선분 끝점 (x, y)
            robot_x, robot_y: 로봇 현재 위치
            ld: lookahead distance (원의 반지름)
        
        Returns:
            (x, y): 진행 방향의 교점 좌표, 또는 None (교점 없음)
        """
        # 1. 직선 벡터(d)와 로봇 상대 벡터(f)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        fx = p1[0] - robot_x
        fy = p1[1] - robot_y
        
        # 2. 2차 방정식 계수 및 판별식 (원과 직선의 교점 공식)
        a = dx**2 + dy**2
        b = 2 * (fx * dx + fy * dy)
        c = (fx**2 + fy**2) - ld**2
        discriminant = b**2 - 4*a*c
        
        # 교점이 없는 경우 (선에서 너무 멀어짐)
        if discriminant < 0:
            return None
            
        # 3. 두 교점(t1, t2) 좌표 계산
        sqrt_dis = math.sqrt(discriminant)
        t1 = (-b - sqrt_dis) / (2*a)
        t2 = (-b + sqrt_dis) / (2*a)
        
        i1_x, i1_y = p1[0] + t1 * dx, p1[1] + t1 * dy
        i2_x, i2_y = p1[0] + t2 * dx, p1[1] + t2 * dy
        
        # 4. [핵심] 내적으로 진행 방향(순방향) 점 선택
        # (경로 벡터) • (로봇->교점 벡터) > 0 인 쪽이 앞쪽
        dot1 = dx * (i1_x - robot_x) + dy * (i1_y - robot_y)
        dot2 = dx * (i2_x - robot_x) + dy * (i2_y - robot_y)
        
        return (i1_x, i1_y) if dot1 > dot2 else (i2_x, i2_y)
    
    def move_state(self):
        """Line Following 방식의 Pure Pursuit"""
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # 현재 추종할 선분 정의 (이전 waypoint → 목표 waypoint)
        if self.current_waypoint_idx < 1:
            self.current_waypoint_idx = 1
        
        if self.current_waypoint_idx >= len(self.waypoints):
            # 마지막 도달 시 정지
            twist = Twist()
            return twist
        
        p1 = self.waypoints[self.current_waypoint_idx - 1]  # 시작점
        p2 = self.waypoints[self.current_waypoint_idx]      # 끝점
        
        # 로봇 반경(Ld)와 활성 선분의 교점 계산
        lookahead_point = self.intersection_point(p1, p2, x, y, self.lookahead_distance)
        
        # 교점이 없으면 수선의 발(경로 위 가장 가까운 점)을 목표로 복귀 우선
        if lookahead_point is None:
            # 선분 위의 가장 가까운 점(수선의 발) 계산
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            
            if dx_seg == 0 and dy_seg == 0:
                # 선분이 점인 경우
                closest_point = p1
            else:
                # 매개변수 t 계산 (0~1 사이로 클램핑)
                t = max(0, min(1, ((x - p1[0]) * dx_seg + (y - p1[1]) * dy_seg) / (dx_seg**2 + dy_seg**2)))
                closest_point = (p1[0] + t * dx_seg, p1[1] + t * dy_seg)
            
            self.get_logger().warn(
                f"[RUN] No intersection found (too far from path). "
                f"Using closest point on segment for recovery."
            )
            lookahead_point = closest_point
        
        # 목표점까지 거리
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        ld = np.hypot(dx, dy)
        
        # Pure Pursuit 곡률 계산
        if ld > 0.01:  # 너무 가까우면 제어 불안정
            # 차량 좌표계로 변환
            x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
            curvature = 2.0 * y_r / (ld ** 2)
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
        
        # plot용으로 lookahead_point 저장
        self.current_lookahead_point = lookahead_point
        
        return twist
    
    def rotate_state(self):
        """제자리 회전으로 다음 경로 방향 정렬"""
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # 현재 waypoint에서 다음 waypoint 방향 계산
        if self.current_waypoint_idx >= len(self.waypoints):
            # 마지막 도달 시
            self.state = "move"
            return Twist()
        
        curr_wp = self.waypoints[self.current_waypoint_idx - 1]
        next_wp = self.waypoints[self.current_waypoint_idx]
        
        # 목표 방향
        target_yaw = math.atan2(next_wp[1] - curr_wp[1], next_wp[0] - curr_wp[0])
        yaw_error = self.normalize_angle(target_yaw - yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        twist = Twist()
        if yaw_error_deg < 1.0:
            self.state = "move"
            self.get_logger().info(
                f"[RUN] Yaw aligned (error {yaw_error_deg:.2f} deg). Switching to move state."
            )
        else:
            twist.linear.x = 0.0
            twist.angular.z = np.sign(yaw_error) * (np.pi / 36)  # 10 deg/s
            self.get_logger().info(
                f"[RUN] Rotating in place. Yaw error: {yaw_error_deg:.2f}°"
            )
        
        return twist

    def publish_cmd_vel(self, twist):
        self.cmd_pub.publish(twist)

    # ========== Input ==========
    def ppc_enable_callback(self, msg: Bool):
        # 상태가 변경될 때만 로그 출력
        if self.is_enabled != msg.data:
            if msg.data:
                # ========== ENABLE (활성화) ==========
                if self.mission_completed:
                    # 완전히 새로운 미션 시작
                    self.mission_completed = False
                    self.current_waypoint_idx = 1
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
                self.cmd_pub.publish(self.zero_twist)
                self.get_logger().info(
                    f'[PPC] PAUSED - State preserved (mission_completed={self.mission_completed})'
                )
                
        self.is_enabled = msg.data

    def gps_callback(self, msg):
        lon = msg.longitude
        lat = msg.latitude
        x, y = self.enu_converter.transform(lon, lat)

        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = 0.0
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0

        p.pose.covariance = [
            0.04, 0, 0, 0, 0, 0,
            0, 0.04, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e6,
        ]
        self.absxy_pub.publish(p)
        
    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.current_time == 0.0:
            self.clock_cnt = 0.0
        elif self.current_time > self.current_time_pre:
            self.clock_cnt += 1.0
            self.current_time_pre = self.current_time
            
            if not self.waypoints_published:
                self.publish_waypoints_path()
                self.waypoints_published = True

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
        
        # 활성 선분 정의
        if self.current_waypoint_idx < 1 or self.current_waypoint_idx >= len(self.waypoints):
            return
        
        p1 = self.waypoints[self.current_waypoint_idx - 1]
        p2 = self.waypoints[self.current_waypoint_idx]
        
        # Lookahead Point 계산 (이미 move_state에서 계산됨)
        lookahead_point = getattr(self, 'current_lookahead_point', p2)
        
        # CTE 계산 (현재 활성 선분까지의 최단 거리)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        if dx == 0 and dy == 0:
            cte = math.hypot(x - p1[0], y - p1[1])
        else:
            t = max(0, min(1, ((x - p1[0]) * dx + (y - p1[1]) * dy) / (dx**2 + dy**2)))
            closest_x = p1[0] + t * dx
            closest_y = p1[1] + t * dy
            cte = math.hypot(x - closest_x, y - closest_y)
        
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
        
        # Waypoint Index 발행
        waypoint_idx_msg = Int32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_idx_pub.publish(waypoint_idx_msg)
        
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
