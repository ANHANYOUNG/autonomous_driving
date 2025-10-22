#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StateMachineExecutor(Node):
    """
    - '/robot_state'를 구독하여 현재 마스터 상태를 인지합니다.
    - 해당 상태에 맞는 *동작*을 수행합니다. (예: IDLE일 때 정지, AUTO일 때 경로 발행)
    - AUTO 상태일 때 'path_A', 'AUTO_START' 같은 세부 명령을 처리합니다.
    """
    def __init__(self):
        super().__init__('state_machine_executor_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('State Machine Executor has been started.')

        # 1. 상태 변수
        self.current_state = 'INIT' # '관리자'로부터 상태를 받기 전

        # 2. 경로 파라미터 로드 (원본 코드와 동일)
        self.available_paths = {}
        path_params = self.get_parameters_by_prefix('paths')
        for subkey, param in path_params.items():
            path_key = subkey
            flat_list = list(param.value) if param.value is not None else []
            if not flat_list or len(flat_list) % 2 != 0:
                self.get_logger().error(f"Path '{path_key}' invalid. Skipping.")
                continue
            grouped = [[float(v) for v in flat_list[i:i+2]] for i in range(0, len(flat_list), 2)]
            self.available_paths[path_key] = grouped
        
        if self.available_paths:
            self.get_logger().info(f"Available paths loaded: {list(self.available_paths.keys())}")
        else:
            self.get_logger().warn("'paths.*' parameters not found. Check path.yaml.")

        # 3. 발행 (Publications)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.zero_twist = Twist() # 정지 명령용 (모든 필드 0)

        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.ppc_path_pub = self.create_publisher(Path, '/ppc/path', path_qos)
        self.ppc_enable_pub = self.create_publisher(Bool, '/ppc/enable', 10)
        

        # 4. 구독 (Subscriptions)
        # 4-1. (핵심) '관리자'로부터 마스터 상태 수신
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(String, '/robot_state', self.state_callback, state_qos)

        # 4-2. AUTO 상태일 때 세부 명령 (경로 선택 등) 수신
        self.create_subscription(String, '/state_command', self.command_callback, 10)
        
        # TODO: '/ppc/status' 토픽을 구독하여 경로 완료 시 '/auto/status' 발행 로직 추가
        # self.create_subscription(String, '/ppc/status', self.ppc_status_callback, 10)

        # 5. AUTO 모드용 내부 변수
        self.auto_phase = 'PENDING'
        self.selected_path_name = None
        self.selected_path_points = None

        # 6. 주기적인 동작 수행용 타이머 (예: IDLE 상태에서 계속 정지 명령)
        self.timer = self.create_timer(0.1, self.loop)

    def state_callback(self, msg):
        """
        마스터 상태(/robot_state) 변경을 감지하고,
        상태 '진입(Entry)'/'이탈(Exit)' 시의 동작을 수행합니다.
        """
        new_state = msg.data
        if new_state == self.current_state:
            return # 상태 변경 없음

        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'State changed: {old_state} -> {new_state}')

        # --- 이탈 (Exit) 로직 ---
        # 이전 상태가 AUTO였다면, PPC를 비활성화해야 함
        if old_state == 'AUTO':
            self.ppc_enable_pub.publish(Bool(data=False))
            self.get_logger().info('Exiting AUTO: PPC disabled.')
        
        # TODO: CALIBRATION 이탈 시 필요한 동작 추가
        if old_state == 'CALIBRATION':
            self.get_logger().info('Exiting CALIBRATION.')

        # --- 진입 (Entry) 로직 ---
        # 새 상태에 따라 초기 동작 수행
        if new_state == 'IDLE':
            self.stop_pub.publish(self.zero_twist) # 즉시 정지
            # AUTO 상태 변수 초기화
            self.auto_phase = 'PENDING'
            self.selected_path_name = None
            self.selected_path_points = None
        
        elif new_state == 'AUTO':
            # AUTO 상태 진입 시 변수 초기화
            self.auto_phase = 'PENDING'
            self.selected_path_name = None
            self.selected_path_points = None
            self.get_logger().info('Entered AUTO mode. Waiting for path selection.')

        elif new_state == 'MANUAL':
            self.get_logger().info('Entered MANUAL mode.')

        elif new_state == 'CALIBRATION':
            self.get_logger().info('Entered CALIBRATION mode.')

    def loop(self):
        """
        0.1초마다 실행되는 루프.
        현재 상태에서 "지속적으로" 수행해야 하는 동작을 처리합니다.
        """
        if self.current_state == 'IDLE':
            # IDLE 상태에서는 계속 정지 명령을 보냄
            self.stop_pub.publish(self.zero_twist)


    def command_callback(self, msg):
        """
        /state_command를 처리합니다.
        '실행기'는 AUTO 상태일 때의 세부 명령('path_A' 등)만 관심 있습니다.
        """
        if self.current_state != 'AUTO':
            return # AUTO 상태가 아니면 세부 명령 무시

        command = msg.data

        # AUTO 상태에서 경로 선택
        if command in self.available_paths:
            path_key = command
            self.selected_path_name = path_key
            self.selected_path_points = self.available_paths[path_key]
            self.auto_phase = 'PENDING' # 경로가 바뀌었으므로 PENDING
            self.ppc_enable_pub.publish(Bool(data=False)) # 일단 비활성화
            self.get_logger().info(f'Selected {path_key} ({len(self.selected_path_points)} pts). Ready to AUTO_START.')

        # AUTO 상태에서 자율주행 시작
        elif command == 'AUTO_START':
            if not self.selected_path_points:
                self.get_logger().warn('No path selected. Cannot AUTO_START.')
            else:
                # 경로 퍼블리시 및 PPC 활성화
                self._publish_path(self.selected_path_name, self.selected_path_points)
                self.auto_phase = 'RUNNING'
                self.ppc_enable_pub.publish(Bool(data=True))
                self.get_logger().info(f'AUTO started. Path {self.selected_path_name} published. PPC enabled.')

    def _publish_path(self, path_name, points_xy):
        """ (원본 코드와 동일) 경로를 /ppc/path 토픽으로 발행 """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # 전역 프레임

        poses = []
        for xy in points_xy:
            x, y = float(xy[0]), float(xy[1])
            ps = PoseStamped()
            ps.header.stamp = path_msg.header.stamp
            ps.header.frame_id = path_msg.header.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            poses.append(ps)

        path_msg.poses = poses
        self.ppc_path_pub.publish(path_msg)
        self.get_logger().info(f"Published '{path_name}' to /ppc/path ({len(poses)} poses).")



def main(args=None):
    rclpy.init(args=args)
    state_machine_executor = StateMachineExecutor()
    rclpy.spin(state_machine_executor)
    state_machine_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()