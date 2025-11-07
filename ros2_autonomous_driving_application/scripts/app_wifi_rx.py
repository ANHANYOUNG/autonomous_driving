#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP 요청을 처리하는 핸들러"""
    
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)
    
    def do_POST(self):
        """POST 요청 처리"""
        if self.path == '/to_rasp':
            try:
                # Content-Length 헤더로 데이터 크기 확인
                content_length = int(self.headers['Content-Length'])
                
                # POST 데이터 읽기
                post_data = self.rfile.read(content_length)
                
                # JSON 파싱
                json_data = json.loads(post_data.decode('utf-8'))
                
                # ROS 노드로 데이터 전달
                if self.ros_node:
                    self.ros_node.process_app_data(json_data)
                
                # 성공 응답
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')  # CORS 허용
                self.end_headers()
                
                response = {"status": "success", "message": "Data received"}
                self.wfile.write(json.dumps(response).encode('utf-8'))
                
            except json.JSONDecodeError as e:
                # JSON 파싱 에러
                self.send_error(400, f"Invalid JSON: {e}")
                
            except Exception as e:
                # 기타 에러
                self.send_error(500, f"Server error: {e}")
        else:
            # 잘못된 경로
            self.send_error(404, "Not Found")
    
    def do_OPTIONS(self):
        """CORS preflight 요청 처리"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """로그 메시지 출력 (선택적으로 비활성화 가능)"""
        if self.ros_node:
            self.ros_node.get_logger().debug(f"HTTP: {format % args}")


class AppWiFiReceiver(Node):
    def __init__(self):
        super().__init__('app_wifi_receiver')
        
        # ROS2 퍼블리셔 생성
        self.sw_bits_pub = self.create_publisher(String, '/app/sw_bits', 10)
        self.key_bits_pub = self.create_publisher(String, '/app/key_bits', 10)
        self.raw_app_data_pub = self.create_publisher(String, '/app/raw_data', 10)
        
        # HTTP 서버 설정
        self.declare_parameter('port', 8889)
        self.declare_parameter('host', '0.0.0.0')  # 모든 인터페이스에서 접속 허용
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # 데이터 카운터 (통계용)
        self.data_count = 0
        self.last_data_time = None
        
        # HTTP 서버 시작
        self.start_http_server()
        
        self.get_logger().info(f'[WIFI_RX] App WiFi Receiver started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_RX] ip: http://192.168.4.1:8889/to_rasp')
    
    def start_http_server(self):
        """HTTP 서버를 별도 스레드에서 시작"""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        
        try:
            self.server = HTTPServer((self.host, self.port), handler_factory)
            
            # 서버를 별도 스레드에서 실행
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()

            self.get_logger().info(f'[WIFI_RX] HTTP Server listening on {self.host}:{self.port}')

        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Failed to start HTTP server: {e}')

    def process_app_data(self, json_data):
        """앱에서 받은 JSON 데이터를 ROS2 토픽으로 발행"""
        try:
            # 데이터 카운터 증가
            self.data_count += 1
            current_time = self.get_clock().now()
            
            # 전체 JSON 데이터를 원시 데이터로 발행
            raw_msg = String()
            raw_msg.data = json.dumps(json_data)
            self.raw_app_data_pub.publish(raw_msg)
            
            # sw_bits 데이터 발행
            if 'sw_bits' in json_data:
                sw_msg = String()
                sw_msg.data = str(json_data['sw_bits'])
                self.sw_bits_pub.publish(sw_msg)
                self.get_logger().debug(f'[WIFI_RX] Published sw_bits: {json_data["sw_bits"]}')
            
            # key_bits 데이터 발행
            if 'key_bits' in json_data:
                key_msg = String()
                key_msg.data = str(json_data['key_bits'])
                self.key_bits_pub.publish(key_msg)
                self.get_logger().debug(f'[WIFI_RX] Published key_bits: {json_data["key_bits"]}')

            # 통계 로그 (10개마다 출력)
            if self.data_count % 10 == 0:
                self.get_logger().info(f'[WIFI_RX] Received {self.data_count} app data packets')

            # 데이터 수신 간격 계산 (처음이 아닌 경우)
            if self.last_data_time:
                interval = (current_time.nanoseconds - self.last_data_time.nanoseconds) / 1e9
                if interval > 1.0:  # 1초 이상 간격이면 로그
                    self.get_logger().warn(f'[WIFI_RX] Large data interval: {interval:.2f}s')

            self.last_data_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Error processing app data: {e}')

    def destroy_node(self):
        """노드 종료 시 HTTP 서버 정리"""
        self.get_logger().info('[WIFI_RX] Shutting down App WiFi Receiver...')
    
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
