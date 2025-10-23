import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # 설정 파일들의 경로를 가져옵니다.
    bringup_pkg_path = get_package_share_directory('ros2_autonomous_driving_bringup')
    twist_mux_config = os.path.join(bringup_pkg_path, 'config', 'twist_mux.yaml')
    path_config = os.path.join(bringup_pkg_path, 'config', 'path.yaml')
    ekf_config = os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')

    # 1. Twist Mux 실행 (속도 명령 다중화)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],

    )

    # # 2. Robot Localization (EKF) 실행 (센서 융합)
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     parameters=[ekf_config]
    # )

    # 3. 상태 관리 및 실행 노드들
    state_manager_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_manager.py',
        name='state_manager'
    )
    
    state_machine_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_machine.py',
        name='state_machine_executor',
        parameters=[path_config] # path.yaml 파일을 파라미터로 로드
    )

    # 4. 앱 브릿지 노드 (앱 인벤터 연동)
    app_bridge_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_bridge.py',
        name='app_bridge'
    )

    # # 5. 자율주행 핵심 로직 (Pure Pursuit)
    # pure_pursuit_node = Node(
    #     package='ros2_autonomous_driving_application',
    #     executable='pure_pursuit_controller.py',
    #     name='pure_pursuit_controller'
    # )

    # # 6. 센서 및 드라이버 노드들
    # imu_node = Node(
    #     package='ros2_autonomous_driving_application',
    #     executable='imu.py',
    #     name='imu_publisher'
    # )

    # uwb_node = Node(
    #     package='ros2_autonomous_driving_application',
    #     executable='uwb_publisher.py',
    #     name='uwb_publisher'
    # )
    
    # motor_driver_node = Node(
    #     package='ros2_autonomous_driving_application',
    #     executable='motor_cmd_vel_trx.py',
    #     name='motor_driver'
    # )
    app_wifi_rx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_wifi_rx.py',
        name='app_wifi_receiver_node',
        output='screen',
        parameters=[{'port': 8889},   # 수신 포트
                    {'host': '0.0.0.0'},]  # 수신 호스트
    )

    app_wifi_tx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_wifi_tx.py',
        name='app_wifi_transmitter_node',
        output='screen',
        parameters=[{'port': 8888},   # 송신 포트
                    {'host': '0.0.0.0'},]  # 송신 호스트
    )


    return LaunchDescription([
        twist_mux_node,
        # robot_localization_node,
        state_manager_node,
        state_machine_node,
        app_bridge_node,
        # pure_pursuit_node,
        # imu_node,
        # uwb_node,
        # motor_driver_node,
        app_wifi_rx_node,
        app_wifi_tx_node,
    ])
