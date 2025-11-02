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

    ### --- 1. use_sim_time 런치 인자 선언 --- ###
    # 시뮬레이션 <-> 실제 로봇 전환 시 여기만 'true' 또는 'false'로 변경
    # 시뮬레이션: ros2 launch ros2_autonomous_driving_bringup state_test.launch.py use_sim_time:=true
    # 실제 로봇: ros2 launch ros2_autonomous_driving_bringup state_test.launch.py use_sim_time:=false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # ★★★ 여기만 수정하세요 ★★★
        description='Use simulation (Gazebo) clock if true'
    )

    ### --- 2. 런치 인자 값을 변수로 가져오기 --- ###
    use_sim_time = LaunchConfiguration('use_sim_time')


    # 1. Twist Mux 실행
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        # 'use_sim_time' 변수를 사용하도록 수정
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}], 
    )

    # 2. Robot Localization (EKF) 실행
    ekf_single_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_single",
        output="screen",
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}, # 'use_sim_time' 변수 사용
            {'print_diagnostics': True},     # 추가!

        ],
        remappings=[
            ('odometry/filtered', 'odometry/ekf_single')
        ]
    )

    # 3. 상태 관리 및 실행 노드들
    state_manager_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{'use_sim_time': use_sim_time}] # 'use_sim_time' 변수 사용
    )
    
    state_machine_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_machine.py',
        name='state_machine_executor',
        parameters=[
            path_config,
            {'use_sim_time': use_sim_time} # 'use_sim_time' 변수 사용
        ]
    )

    # 4. 앱 브릿지 노드
    app_bridge_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_bridge.py',
        name='app_bridge',
        parameters=[{'use_sim_time': use_sim_time}] # 'use_sim_time' 변수 사용
    )

    # 5. 자율주행 핵심 로직 (Pure Pursuit)
    pure_pursuit_node = Node(
        package='ros2_autonomous_driving_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_controller',
        parameters=[{'use_sim_time': use_sim_time}] # 'use_sim_time' 변수 사용
    )
    
    # 6. Wifi 통신 노드들
    app_wifi_rx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_wifi_rx.py',
        name='app_wifi_receiver_node',
        output='screen',
        parameters=[
            {'port': 8889},
            {'host': '0.0.0.0'},
            {'use_sim_time': use_sim_time} # 'use_sim_time' 변수 사용
        ]
    )

    app_wifi_tx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_wifi_tx.py',
        name='app_wifi_transmitter_node',
        output='screen',
        parameters=[
            {'port': 8888},
            {'host': '0.0.0.0'},
            {'use_sim_time': use_sim_time} # 'use_sim_time' 변수 사용
        ]
    )

    # # 7. 실제 센서 및 드라이버 노드들
    motor_cmd_vel_trx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='motor_cmd_vel_trx_v2.py',
        name='motor_cmd_vel_trx_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}
        ] # 'use_sim_time' 변수 사용
    )
    # CAL 결과를 사용한 imu offset node
    imu_offset_node = Node(
        package='ros2_autonomous_driving_application',
        executable='imu_offset.py',
        name='imu_offset_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}
        ] # 'use_sim_time' 변수 사용
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

    # plot
    plot_ppc_node = Node(
        package = 'ros2_autonomous_driving_application',
        executable = 'plot_ppc.py',
        name = 'plot_ppc_node',
        output = 'screen',
        parameters = [{'use_sim_time': use_sim_time}] 
    )


    return LaunchDescription([
        ### --- 4. 런치 인자를 실행 목록에 추가 --- ###
        use_sim_time_arg,
        
        # --- 노드 실행 목록 ---
        # 속도 선택
        twist_mux_node,
        # 칼만 필터
        ekf_single_node,
        # 상태 머신
        state_manager_node,
        state_machine_node,
        # 어플 연동
        app_bridge_node,
        app_wifi_rx_node,
        app_wifi_tx_node,
        # 주행 제어
        pure_pursuit_node,
        motor_cmd_vel_trx_node,
        # # 센서
        imu_offset_node,
        uwb_publisher_node,
        # uwb_data_wifi_out,
        imu_node,
        # uwb_publisher_dummy_node,
        # TF 고정 변환
        static_imu_tf,
        static_odom_to_chassis,
        # plot
        plot_ppc_node,
    ])