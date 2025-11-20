#!/usr/bin/env python3

import rclpy
import threading
import queue
from rclpy.node import Node
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class UWB_Publisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher')
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)
        self.raw_data_pub = self.create_publisher(String, '/uwb_raw_data', 1)

        # 파라미터: 포트, 보레이트
        self.declare_parameter('port', '/dev/usb-left-bottom')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 데이터 큐와 스레드 제어용 플래그
        self.data_queue = queue.Queue()
        self.running = True

        try:
            self.ser = serial.Serial(port, baud, timeout=None)  # blocking 모드로 변경
            self.get_logger().info(f'Opened serial port: {port} at {baud} baud')
            
            # 별도 스레드에서 시리얼 데이터 읽기 시작
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None

        # 큐 처리용 타이머 (빠른 주기로 - 10ms마다 큐 확인)
        self.timer = self.create_timer(0.01, self.process_queue)

    def serial_reader(self):
        """별도 스레드에서 시리얼 데이터를 실시간으로 읽기"""
        while self.running and self.ser and self.ser.is_open:
            try:
                # 개행문자까지 한 줄 읽기 (blocking)
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # 큐에 데이터 추가 (메인 스레드에서 처리하도록)
                    self.data_queue.put(line)
                    self.get_logger().debug(f'[UWB] Queued data: "{line}"')
            except Exception as e:
                if self.running:  # 정상 종료가 아닌 경우만 에러 로그
                    self.get_logger().error(f'Serial read error: {e}')
                break

    def process_queue(self):
        """큐에서 데이터를 꺼내서 처리 (메인 스레드에서 실행)"""
        processed_count = 0
        max_process_per_cycle = 10  # 한 번에 최대 10개까지 처리
        
        while not self.data_queue.empty() and processed_count < max_process_per_cycle:
            try:
                line = self.data_queue.get_nowait()
                self.process_serial_data(line)
                processed_count += 1
            except queue.Empty:
                break

    def process_serial_data(self, line):
        """실제 데이터 처리 및 ROS 메시지 발행"""
        # 디버깅용 로그
        # self.get_logger().info(f'Processing data: "{line}"')

        # 1. 원시 데이터를 그대로 String 메시지로 발행
        # raw_data : "앵커1_x, 앵커1_y, 앵커2_x, 앵커2_y, ..., 태그_x, 태그_y"
        raw_msg = String()
        raw_msg.data = line
        self.raw_data_pub.publish(raw_msg)
        self.get_logger().debug(f'[UWB] Published raw data: "{line}"')
        
        # 2. 태그 위치만 추출해서 PoseWithCovarianceStamped로 발행
        try:
            parts = line.split(',')
            # 마지막 두 값이 pos_x, pos_y
            if len(parts) >= 2:
                # 마지막에서 두 번째와 마지막 값 추출
                x = float(parts[-2].strip())  # 마지막에서 두 번째
                y = float(parts[-1].strip())  # 마지막
                
                # PoseWithCovarianceStamped 메시지 생성
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

                # x,y만 신뢰, 나머지는 크게(무시되도록)
                p.pose.covariance = [
                    0.5, 0,    0,    0,    0,    0,
                    0,    0.5, 0,    0,    0,    0,
                    0,    0,    1e6,  0,    0,    0,
                    0,    0,    0,    1e6,  0,    0,
                    0,    0,    0,    0,    1e6,  0,
                    0,    0,    0,    0,    0,    1e6,
                ]
                self.absxy_pub.publish(p)
                # self.get_logger().info(f'Published position: x={x:.3f}, y={y:.3f}')   
            else:
                self.get_logger().warn(f'[UWB] Not enough data values: {len(parts)} values in "{line}"')
        except ValueError as e:
            self.get_logger().warn(f'[UWB] Failed to parse position data: {line}, error: {e}')

    def destroy_node(self):
        """노드 종료 시 스레드 정리"""
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UWB_Publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()