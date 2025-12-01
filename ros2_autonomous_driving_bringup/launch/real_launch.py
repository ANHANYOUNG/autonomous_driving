import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # Bring file paths
    bringup_pkg_path = get_package_share_directory('ros2_autonomous_driving_bringup')
    twist_mux_config = os.path.join(bringup_pkg_path, 'config', 'twist_mux.yaml')
    path_config = os.path.join(bringup_pkg_path, 'config', 'path.yaml')
    ekf_config = os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')

    # use_sim_time launch argument declaration

    # execution example
    # sim: ros2 launch ros2_autonomous_driving_bringup state_test.launch.py use_sim_time:=true
    # real: ros2 launch ros2_autonomous_driving_bringup state_test.launch.py use_sim_time:=false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # execution example:
    # ros2 launch ros2_autonomous_driving_bringup state_test.launch.py motor_script:=motor_cmd_vel_sim.py ld:=1.0

    # Motor control script selection
    motor_script_arg = DeclareLaunchArgument(
        'motor_script',
        default_value='motor_cmd_vel_real.py',
        description='Motor control script: motor_cmd_vel_real.py,motor_cmd_vel_sim.py, motor_cmd_vel_sim_1.py, motor_cmd_vel_sim_2.py, motor_cmd_vel_sim_3.py, motor_cmd_vel_real_proportional.py, motor_cmd_vel_real_linear.py'
    )
    
    # Lookahead Distance
    ld_arg = DeclareLaunchArgument(
        'ld',
        default_value='1.5',
        description='Pure Pursuit lookahead distance (m)'
    )

    # Get launch argument values as variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    motor_script = LaunchConfiguration('motor_script')
    ld = LaunchConfiguration('ld')


    # Twist Mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}], 
    )

    # EKF
    ekf_single_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_single",
        output="screen",
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/ekf_single')
        ]
    )

    # State Manager
    state_manager_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # State Machine
    state_machine_node = Node(
        package='ros2_autonomous_driving_application',
        executable='state_machine.py',
        name='state_machine_executor',
        parameters=[
            path_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # App Bridge
    app_bridge_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_bridge.py',
        name='app_bridge',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ppc
    pure_pursuit_node = Node(
        package='ros2_autonomous_driving_application',
        executable='ppc_real.py',
        name='pure_pursuit_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'lookahead_distance': ld}
        ]
    )

    # Wifi RX/TX
    app_wifi_rx_node = Node(
        package='ros2_autonomous_driving_application',
        executable='app_wifi_rx.py',
        name='app_wifi_receiver_node',
        output='screen',
        parameters=[
            {'port': 8889},
            {'host': '0.0.0.0'},
            {'use_sim_time': use_sim_time}
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
            {'use_sim_time': use_sim_time}
        ]
    )

    # Motor Drivers
    motor_cmd_vel_trx_node = Node(
        package='ros2_autonomous_driving_application',
        executable=motor_script,  #  런치 인자로 스크립트 선택
        name='motor_cmd_vel_trx_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Sensors
    imu_node = Node(
        package='ros2_autonomous_driving_application',
        executable='imu.py',
        name='wt901c_imu_node',
        output='screen'
    )

    # imu_offset after CAL
    imu_offset_node = Node(
        package='ros2_autonomous_driving_application',
        executable='imu_offset.py',
        name='imu_offset_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}
        ]
    )

    uwb_publisher_node = Node(
        package='ros2_autonomous_driving_application',
        executable='uwb_publisher.py',
        name='uwb_publisher_node',
        output='screen'
    )

    # tf static transform: imu_link to chassis_link
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "chassis_link", "imu_link"],
        output="screen",
    )

    # tf static transform: odom to chassis_link
    static_odom_to_chassis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_chassis_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'chassis_link']
    )


    return LaunchDescription([
        # Add launch arguments to the execution list
        use_sim_time_arg,
        motor_script_arg,
        ld_arg,
        
        # cmd_vel selector
        twist_mux_node,

        # Kalman filter
        ekf_single_node,

        # State machine
        state_manager_node,
        state_machine_node,

        # App
        app_bridge_node,
        app_wifi_rx_node,
        app_wifi_tx_node,

        # Driving control
        pure_pursuit_node,
        motor_cmd_vel_trx_node,

        # Sensors
        imu_offset_node,
        uwb_publisher_node,
        imu_node,
   
        # TF static transform
        static_imu_tf,
        static_odom_to_chassis
    ])