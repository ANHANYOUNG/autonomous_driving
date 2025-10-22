#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import json
import time


class UWBWiFiTransmitter(Node):
    def __init__(self):
        super().__init__('uwb_wifi_transmitter')
        
        # ROS2 구독자 - raw_data_pub 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/uwb_raw_data',
            self.uwb_data_callback,
            1
        )
        
        # WiFi 전송 설정
        self.declare_parameter('port', 8888)
        self.declare_parameter('max_clients', 5)
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.max_clients = self.get_parameter('max_clients').get_parameter_value().integer_value
        
        # 클라이언트 연결 관리
        self.client_list = []
        self.server_running = True
        
        # TCP 서버 시작
        self.start_tcp_server()
        
        self.get_logger().info(f'UWB WiFi Transmitter started on port {self.port}')

    def start_tcp_server(self):
        """TCP 서버를 별도 스레드에서 시작"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))  # 모든 인터페이스에서 접속 허용
            self.server_socket.listen(self.max_clients)
            
            # 서버 스레드 시작
            self.server_thread = threading.Thread(target=self.accept_clients, daemon=True)
            self.server_thread.start()
            
            self.get_logger().info(f'TCP Server listening on 0.0.0.0:{self.port}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start TCP server: {e}')

    def accept_clients(self):
        """클라이언트 연결을 받아들이는 스레드"""
        while self.server_running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().info(f'Client connected: {client_address}')
                
                # 클라이언트 리스트에 추가
                self.client_list.append({
                    'socket': client_socket,
                    'address': client_address,
                    'connected': True
                })
                
                # 클라이언트별 처리 스레드 시작
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address),
                    daemon=True
                )
                client_thread.start()
                
            except Exception as e:
                if self.server_running:
                    self.get_logger().error(f'Error accepting client: {e}')

    def handle_client(self, client_socket, client_address):
        """개별 클라이언트 연결 관리"""
        try:
            # 연결 확인 메시지 전송
            welcome_msg = {
                'type': 'connection',
                'message': 'Connected to UWB data stream',
                'timestamp': time.time()
            }
            self.send_to_client(client_socket, json.dumps(welcome_msg))
            
            # 클라이언트가 연결을 유지하는 동안 대기
            while self.server_running:
                # 클라이언트로부터 데이터 수신 (keep-alive 확인용)
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                except socket.timeout:
                    continue
                except:
                    break
                    
        except Exception as e:
            self.get_logger().warn(f'Client {client_address} disconnected: {e}')
        finally:
            self.remove_client(client_socket, client_address)

    def remove_client(self, client_socket, client_address):
        """클라이언트 연결 해제 및 리스트에서 제거"""
        try:
            client_socket.close()
            self.client_list = [c for c in self.client_list if c['address'] != client_address]
            self.get_logger().info(f'Client disconnected: {client_address}')
        except:
            pass

    def send_to_client(self, client_socket, message):
        """개별 클라이언트에게 메시지 전송"""
        try:
            # 메시지 길이를 먼저 전송 (프로토콜)
            message_bytes = message.encode('utf-8')
            length = len(message_bytes)
            client_socket.send(length.to_bytes(4, byteorder='big'))
            client_socket.send(message_bytes)
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to send to client: {e}')
            return False

    def uwb_data_callback(self, msg):
        """UWB 데이터 수신 콜백 - 모든 연결된 클라이언트에게 전송"""
        if not self.client_list:
            return
            
        # UWB 데이터 패키지 생성
        data_package = {
            'type': 'uwb_data',
            'data': msg.data,  # 원시 UWB 데이터 그대로
            'timestamp': time.time()
        }
        
        message = json.dumps(data_package)
        self.get_logger().debug(f'Broadcasting UWB data: {msg.data}')
        
        # 모든 연결된 클라이언트에게 전송
        disconnected_clients = []
        for client in self.client_list:
            if not self.send_to_client(client['socket'], message):
                disconnected_clients.append(client)
        
        # 연결이 끊어진 클라이언트 제거
        for client in disconnected_clients:
            self.remove_client(client['socket'], client['address'])

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('Shutting down UWB WiFi Transmitter...')
        
        self.server_running = False
        
        # 모든 클라이언트 연결 종료
        for client in self.client_list:
            try:
                client['socket'].close()
            except:
                pass
        
        # 서버 소켓 종료
        if hasattr(self, 'server_socket'):
            try:
                self.server_socket.close()
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UWBWiFiTransmitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()