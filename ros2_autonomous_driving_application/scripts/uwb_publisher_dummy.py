#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import random
import math
import time


class UWBPublisherDummy(Node):
    def __init__(self):
        super().__init__('uwb_publisher_dummy')
        
        # 퍼블리셔 생성
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)
        self.raw_data_pub = self.create_publisher(String, '/uwb_raw_data', 1)
        
        # 더미 데이터 설정
        self.declare_parameter('tag_x', 5.532)  # 태그 x 좌표 (기준점)
        self.declare_parameter('tag_y', 10.345)  # 태그 y 좌표 (기준점)
        self.declare_parameter('publish_rate', 0.25)  # 발행 주기 (Hz)
        
        # 기준 위치 (중심점)
        self.base_x = self.get_parameter('tag_x').get_parameter_value().double_value
        self.base_y = self.get_parameter('tag_y').get_parameter_value().double_value
        
        # 현재 위치 (움직이는 실제 위치)
        self.tag_x = self.base_x
        self.tag_y = self.base_y
        
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 랜덤 움직임 설정
        self.max_movement = 0.3  # 최대 이동 거리 (m)
        self.movement_probability = 0.7  # 움직일 확률 (70%)
        self.boundary_radius = 20.0  # 기준점에서 최대 거리 (m)
        
        # 더미 앵커 위치 설정 (4개 앵커)
        self.anchor_positions = [
            (0.0, 0.0),    # 앵커 1
            (10.546, 0.0),   # 앵커 2
            (10.124, 40.124),  # 앵커 3
            (0.234, 48.356)    # 앵커 4
        ]
        
        # 타이머 생성 (주기적으로 데이터 발행)
        timer_period = 1.0 / publish_rate  # 초 단위
        self.timer = self.create_timer(timer_period, self.publish_data)
        
        self.get_logger().info(f'UWB Dummy Publisher started - Base position: ({self.base_x}, {self.base_y})')
        self.get_logger().info(f'Publishing rate: {publish_rate} Hz, Random movement enabled')

    def update_random_position(self):
        """랜덤한 위치 업데이트"""
        # 일정 확률로만 움직임
        if random.random() < self.movement_probability:
            # 랜덤 각도와 거리
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, self.max_movement)
            
            # 새로운 위치 계산
            new_x = self.tag_x + distance * math.cos(angle)
            new_y = self.tag_y + distance * math.sin(angle)
            
            # 기준점에서 너무 멀어지지 않도록 제한
            distance_from_base = math.sqrt((new_x - self.base_x)**2 + (new_y - self.base_y)**2)
            
            if distance_from_base <= self.boundary_radius:
                self.tag_x = new_x
                self.tag_y = new_y
            else:
                # 경계를 벗어나면 기준점 방향으로 조금 이동
                direction_to_base = math.atan2(self.base_y - self.tag_y, self.base_x - self.tag_x)
                self.tag_x += 0.1 * math.cos(direction_to_base)
                self.tag_y += 0.1 * math.sin(direction_to_base)

    def publish_data(self):
        """더미 UWB 데이터 발행"""
        
        # 랜덤 위치 업데이트
        self.update_random_position()
        
        # 1. 원시 데이터 생성 (앵커1_x, 앵커1_y, 앵커2_x, 앵커2_y, ..., 태그_x, 태그_y)
        raw_data_parts = []
        
        # 앵커 위치들 추가
        for anchor_x, anchor_y in self.anchor_positions:
            raw_data_parts.append(f"{anchor_x:.2f}")
            raw_data_parts.append(f"{anchor_y:.2f}")
        
        # 태그 위치 추가 (마지막 두 값)
        raw_data_parts.append(f"{self.tag_x:.2f}")
        raw_data_parts.append(f"{self.tag_y:.2f}")
        
        # 쉼표로 연결된 문자열 생성
        raw_data_string = ", ".join(raw_data_parts)
        
        # 2. 원시 데이터 발행 (String 메시지)
        raw_msg = String()
        raw_msg.data = raw_data_string
        self.raw_data_pub.publish(raw_msg)
        
        # 3. PoseWithCovarianceStamped 메시지 생성 및 발행
        pose_msg = PoseWithCovarianceStamped()
        
        # 헤더 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # 위치 설정
        pose_msg.pose.pose.position.x = self.tag_x
        pose_msg.pose.pose.position.y = self.tag_y
        pose_msg.pose.pose.position.z = 0.0
        
        # 오리엔테이션 설정 (단위 쿼터니언)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # 공분산 설정 (x,y는 신뢰도 높게, 나머지는 낮게)
        pose_msg.pose.covariance = [
            0.01, 0,    0,    0,    0,    0,    # x: 10cm 표준편차
            0,    0.01, 0,    0,    0,    0,    # y: 10cm 표준편차
            0,    0,    1e6,  0,    0,    0,    # z: 무시
            0,    0,    0,    1e6,  0,    0,    # roll: 무시
            0,    0,    0,    0,    1e6,  0,    # pitch: 무시
            0,    0,    0,    0,    0,    1e6,  # yaw: 무시
        ]
        
        # 메시지 발행
        self.absxy_pub.publish(pose_msg)
        
        # 로그 출력 (가끔씩만)
        current_time = int(time.time())
        if current_time % 8 == 0:  # 8초마다
            distance_from_base = math.sqrt((self.tag_x - self.base_x)**2 + (self.tag_y - self.base_y)**2)
            self.get_logger().info(f'Tag moving: ({self.tag_x:.2f}, {self.tag_y:.2f}), dist from base: {distance_from_base:.2f}m')

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('UWB Dummy Publisher shutting down...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UWBPublisherDummy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()