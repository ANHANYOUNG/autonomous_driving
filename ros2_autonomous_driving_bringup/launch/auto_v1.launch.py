
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros2_autonomous_driving_bringup')

    pure_pursuit_node = Node(
        package='ros2_autonomous_driving_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        output='screen'
    )

    # 6-1. 로컬 EKF (IMU -> odom): IMU 데이터만 사용하여 부드럽지만 드리프트가 있는 지역(local) 주행 거리계를 생성합니다.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml")],
        remappings=[('odometry/filtered', 'odometry/local')] # 출력 토픽 이름을 변경
    )

    # 6-2. 글로벌 EKF (GPS+IMU -> map): GPS와 IMU 데이터를 융합하여 드리프트가 없는 전역(global) 위치를 추정합니다.
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml")],
        remappings=[('odometry/filtered', 'odometry/global')] # 출력 토픽 이름을 변경
    )

    # 6-3. NavSat Transform: GPS의 위도/경도 데이터를 EKF가 사용할 수 있는 UTM 또는 지역 좌표계(x,y)로 변환합니다.
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"),],
        remappings=[
            ("imu", "/imu/data"),
            ("gps/fix", "/navsat"),
            ("odometry/filtered", "/odometry/global"),
            ("gps/filtered", "/gps/filtered"),
            ("odometry/gps", "/odometry/gps"),
            
        ]
    )

    ekf_single_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_single",
        output="screen",
        parameters=[os.path.join(pkg_project_bringup, "config", "ekf.yaml"),],
        remappings=[('odometry/filtered', 'odometry/ekf_single')] # 출력 토픽 이름을 변경
    )

    uwb_publisher_node = Node(
        package='ros2_autonomous_driving_application',
        executable='uwb_publisher.py',
        name='uwb_publisher_node',
        output='screen'
    )

    uwb_data_wifi_out = Node(
        package='ros2_autonomous_driving_application',
        executable='uwb_data_wifi_out.py',
        name='uwb_data_wifi_out',
        output='screen',
        parameters=[{'port': 8888},
                    {'max_clients': 5},]  # 8888번 포트 사용
    )

    imu_node = Node(
        package='ros2_autonomous_driving_application',
        executable='imu.py',
        name='wt901c_imu_node',
        output='screen'
    )

    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "chassis_link", "imu_link"],
        output="screen",
    )

    # odom -> chassis_link (고정 좌표 변환), 에러로 인해 추가함.
    static_odom_to_chassis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_chassis_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'chassis_link']
    )

    uwb_publisher_dummy_node = Node(
        package='ros2_autonomous_driving_application',
        executable='uwb_publisher_dummy.py',
        name='uwb_publisher_dummy_node',
        output='screen',
        parameters=[{'tag_x': 5.0},   # 태그 x 좌표
                    {'tag_y': 10.0},  # 태그 y 좌표
                    {'publish_rate': 1.0},]  # 발행 주기 (Hz)
    )

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

    motor_cmd_vel_trx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='motor_cmd_vel_trx.py',
        name='motor_cmd_vel_trx_node',
        output='screen'
    )

    return LaunchDescription([
        ### pure_pursuit_node,
        ### ekf_local_node,
        ### ekf_global_node,
        ### navsat_transform_node,
        # ekf_single_node,
        # uwb_publisher_node,
        ### uwb_data_wifi_out,
        # imu_node,
        # static_imu_tf,
        # static_odom_to_chassis,
        ### uwb_publisher_dummy_node,
        app_wifi_rx_node,
        app_wifi_tx_node,
        # motor_cmd_vel_trx_node,
    ])
