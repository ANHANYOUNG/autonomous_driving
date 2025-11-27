#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
import threading
import math
from http.server import HTTPServer, BaseHTTPRequestHandler


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP 요청을 처리하는 핸들러"""
    
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """GET 요청 처리"""
        if self.path == '/to_app':
            try:
                # ROS 노드에서 최신 UWB 데이터 가져오기, CPU 과부하 방식
                # if self.ros_node:
                #     json_data = self.ros_node.get_uwb_json_data()
                # else:
                #     json_data = {"error": "ROS node not available"}
                # CPU 적게 쓰는 방식
                if self.ros_node:
                    json_str = self.ros_node.get_uwb_json_data()
                else:
                    json_str = json.dumps({"error": "ROS node not available"})
                
                # 성공 응답
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')  # CORS 허용
                self.end_headers()
                
                # JSON 데이터 전송
                # 기존 방식
                # self.wfile.write(json.dumps(json_data).encode('utf-8'))
                
                # CPU 적게 쓰는 방식
                self.wfile.write(json_str.encode('utf-8'))
                
            except Exception as e:
                # 서버 에러
                self.send_error(500, f"Server error: {e}")
        else:
            # 잘못된 경로
            self.send_error(404, "Not Found")
    
    def do_OPTIONS(self):
        """CORS preflight 요청 처리"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """로그 메시지 출력 비활성화 (성능 향상)"""
        pass  # 로그 출력하지 않음


class AppWiFiTransmitter(Node):
    def __init__(self):
        super().__init__('app_wifi_transmitter')
        
        # ROS2 구독자 생성 - uwb_raw_data 토픽 구독
        self.uwb_subscription = self.create_subscription(
            String,
            '/uwb_raw_data',
            self.uwb_data_callback,
            1
        )

        # EKF 오도메트리 구독자 추가
        self.ekf_subscription = self.create_subscription(
            Odometry,
            '/odometry/ekf_single',
            self.ekf_data_callback,
            1
        )
        
        # HTTP 서버 설정
        self.declare_parameter('port', 8888)
        self.declare_parameter('host', '0.0.0.0')  # 모든 인터페이스에서 접속 허용
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # 데이터 저장 (msg 객체는 저장하지 않음)
        self.latest_uwb_data = None
        self.has_ekf_data = False  # msg 대신 bool 플래그만 사용
        self.ekf_callback_count = 0  # 콜백 카운터
        
        self.uwb_json_data = {
            "anchors": [
                {"id": "A1", "x": 0.00, "y": 0.00},
                {"id": "A2", "x": 0.00, "y": 0.00},
                {"id": "A3", "x": 0.00, "y": 0.00},
                {"id": "A4", "x": 0.00, "y": 0.00}
            ],
            "tag": {"x": 0.00, "y": 0.00},
            "stop_flag": 0,
            "err_msg": [],
            "tag_vel": 0.00,
            "tag_ori": 0.00
        }
        
        # 데이터 접근 락
        self.data_lock = threading.Lock()
        
        # 통계
        self.request_count = 0
        self.data_count = 0
        
        # HTTP 서버 시작
        self.start_http_server()

        self.get_logger().info(f'[WIFI_TX] App WiFi Transmitter started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_TX] Endpoint: http://192.168.4.1:8888/to_app')
        self.get_logger().info('[WIFI_TX] Subscribed to /uwb_raw_data and /odometry/ekf_single')

    def start_http_server(self):
        """HTTP 서버를 별도 스레드에서 시작"""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        
        max_retries = 5
        for retry in range(max_retries):
            try:
                current_port = self.port + retry
                # 서버 설정 최적화
                self.server = HTTPServer((self.host, current_port), handler_factory)
                self.server.timeout = 1.0  # 1초 타임아웃 설정
                
                # 서버를 데몬 스레드에서 실행 (메인 프로세스 종료시 자동 종료)
                self.server_thread = threading.Thread(
                    target=self.serve_forever_with_shutdown, 
                    daemon=True
                )
                self.server_thread.start()

                self.get_logger().info(f'[WIFI_TX] HTTP Server listening on {self.host}:{current_port}')
                self.port = current_port  # 실제 사용하는 포트 업데이트
                break
                
            except OSError as e:
                if e.errno == 98:  # Address already in use
                    self.get_logger().warn(f'[WIFI_TX] Port {current_port} in use, trying {current_port + 1}')
                    continue
                else:
                    self.get_logger().error(f'[WIFI_TX] Failed to start HTTP server: {e}')
                    break
        else:
            self.get_logger().error(f'[WIFI_TX] Could not find available port after {max_retries} attempts')

    def serve_forever_with_shutdown(self):
        """서버 실행 (주기적으로 종료 확인)"""
        try:
            self.server.serve_forever()
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] HTTP server error: {e}')

    def quaternion_to_yaw(self, x, y, z, w):
        """쿼터니언을 yaw 각도(degree)로 변환"""
        # yaw (z-axis rotation)
        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg
    
    def ekf_data_callback(self, msg):
        """EKF 오도메트리 데이터 수신 (50번에 1번만 처리)"""
        try:
            with self.data_lock:  # 스레드 안전성
                self.ekf_callback_count += 1
                self.has_ekf_data = True  # msg 저장 대신 플래그만 설정
                
                # 50번에 한 번만 데이터 업데이트
                if self.ekf_callback_count % 50 == 0:
                    # 위치 정보 (x, y)
                    position_x = msg.pose.pose.position.x
                    position_y = msg.pose.pose.position.y
                    
                    # 속도 정보 (x, y 평면 속도)
                    vel_x = msg.twist.twist.linear.x
                    vel_y = msg.twist.twist.linear.y
                    velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
                    
                    # 방향 정보 (yaw 각도)
                    orientation = msg.pose.pose.orientation
                    yaw_deg = self.quaternion_to_yaw(
                        orientation.x, orientation.y, orientation.z, orientation.w
                    )
                    
                    # JSON 데이터 업데이트
                    self.uwb_json_data["tag"]["x"] = round(position_x, 2)
                    self.uwb_json_data["tag"]["y"] = round(position_y, 2)
                    self.uwb_json_data["tag_vel"] = round(velocity_magnitude, 2)
                    self.uwb_json_data["tag_ori"] = round(yaw_deg, 2)
                    
                    # 디버깅 로그 (가끔씩만)
                    # if self.ekf_callback_count <= 100 or self.ekf_callback_count % 500 == 0:
                    #     self.get_logger().info(
                    #         f'[WIFI_TX] EKF data updated #{self.ekf_callback_count//50}: '
                    #         f'Pos=({position_x:.2f}, {position_y:.2f}), '
                    #         f'Vel={velocity_magnitude:.2f}, Yaw={yaw_deg:.2f}°'
                    #     )
            
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing EKF data: {e}')

    def uwb_data_callback(self, msg):
        """UWB 원시 데이터 수신 및 JSON 변환 (앵커 정보만)"""
        try:
            with self.data_lock:  # 스레드 안전성
                # 원시 데이터 저장
                self.latest_uwb_data = msg.data
                self.data_count += 1
                
                # uwb_raw_data 파싱: "0.00, 0.00, 10.00, 0.00, 10.00, 15.00, 0.00, 15.00, 5.00, 10.00"
                data_parts = [float(x.strip()) for x in msg.data.split(',')]
                
                if len(data_parts) >= 8:  # 앵커 4개 = 8개 값 (태그 정보는 EKF에서 가져옴)
                    # 앵커 위치들 (x, y 쌍으로 4개)
                    self.uwb_json_data["anchors"] = [
                        {"id": "A1", "x": round(data_parts[0], 2), "y": round(data_parts[1], 2)},
                        {"id": "A2", "x": round(data_parts[2], 2), "y": round(data_parts[3], 2)},
                        {"id": "A3", "x": round(data_parts[4], 2), "y": round(data_parts[5], 2)},
                        {"id": "A4", "x": round(data_parts[6], 2), "y": round(data_parts[7], 2)}
                    ]
                    
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing UWB data: {e}')
    
    def get_uwb_json_data(self):
        """앱 요청 시 최신 UWB JSON 데이터 반환"""
        with self.data_lock:  # 스레드 안전성
            self.request_count += 1
            
            # EKF 데이터가 없으면 경고 및 기본값 사용
            if not self.has_ekf_data:  # msg 객체 대신 플래그 확인
                self.uwb_json_data["stop_flag"] = 1
                self.uwb_json_data["err_msg"] = ["No EKF data available"]
            else:
                self.uwb_json_data["stop_flag"] = 0
                self.uwb_json_data["err_msg"] = []
            
            # UWB 데이터가 없으면 경고 (앵커 정보)
            if self.latest_uwb_data is None:
                if "No anchor data available" not in self.uwb_json_data["err_msg"]:
                    self.uwb_json_data["err_msg"].append("No anchor data available")
            
            # # 최신 데이터 반환 (깊은 복사), 에러 방지용 but cpu 너무 많이 쓰는 중
            # 1차 변환 딕셔너리 - 문자열 (json.dumps)
            # 2차 변환 문자열 - 다시 딕셔너리 (json.loads)
            # 3차 변환 딕셔너리 - 문자열로 압축 (do_GET 에서 다시 json.dumps)
            # return json.loads(json.dumps(self.uwb_json_data))

            # CPU 적게 쓰는 방식 - 문자열 바로 반환
            return json.dumps(self.uwb_json_data)
    
    def destroy_node(self):
        """노드 종료 시 HTTP 서버 정리"""
        self.get_logger().info('[WIFI_TX] Shutting down App WiFi Transmitter...')
        
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        
        self.get_logger().info(f'[WIFI_TX] Total requests served: {self.request_count}')
        self.get_logger().info(f'[WIFI_TX] Total UWB data received: {self.data_count}')
        self.get_logger().info(f'[WIFI_TX] Total EKF callbacks: {self.ekf_callback_count}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiTransmitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
