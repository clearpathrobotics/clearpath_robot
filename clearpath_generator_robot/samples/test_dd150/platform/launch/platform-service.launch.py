from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_diagnostic_updater_params = DeclareLaunchArgument(
        'diagnostic_updater_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150/platform/config/diagnostic_updater.yaml',
        description='')

    diagnostic_updater_params = LaunchConfiguration('diagnostic_updater_params')

    launch_arg_diagnostic_aggregator_params = DeclareLaunchArgument(
        'diagnostic_aggregator_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150/platform/config/diagnostic_aggregator.yaml',
        description='')

    diagnostic_aggregator_params = LaunchConfiguration('diagnostic_aggregator_params')

    launch_arg_foxglove_bridge_parameters = DeclareLaunchArgument(
        'foxglove_bridge_parameters',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150/platform/config/foxglove_bridge.yaml',
        description='')

    foxglove_bridge_parameters = LaunchConfiguration('foxglove_bridge_parameters')

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150/platform/config/imu_filter.yaml',
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])
    launch_file_foxglove_bridge = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'foxglove_bridge.launch.py'])
    launch_file_vcan0_receiver = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'receiver.launch.py'])
    launch_file_vcan0_sender = PathJoinSubstitution([
        pkg_clearpath_ros2_socketcan_interface, 'launch', 'sender.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150'
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'false'
                )
                ,
                (
                    'namespace'
                    ,
                    'do150_0000'
                )
                ,
                (
                    'enable_ekf'
                    ,
                    'true'
                )
                ,
            ]
    )

    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_diagnostics]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'do150_0000'
                )
                ,
                (
                    'updater_parameters'
                    ,
                    diagnostic_updater_params
                )
                ,
                (
                    'aggregator_parameters'
                    ,
                    diagnostic_aggregator_params
                )
                ,
            ]
    )

    launch_foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_foxglove_bridge]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'do150_0000'
                )
                ,
                (
                    'parameters'
                    ,
                    foxglove_bridge_parameters
                )
                ,
            ]
    )

    launch_vcan0_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_vcan0_receiver]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'do150_0000'
                )
                ,
                (
                    'interface'
                    ,
                    'vcan0'
                )
                ,
                (
                    'from_can_bus_topic'
                    ,
                    'vcan0/rx'
                )
                ,
                (
                    'enable_can_fd'
                    ,
                    'false'
                )
                ,
                (
                    'interval_sec'
                    ,
                    '0.01'
                )
                ,
                (
                    'use_bus_time'
                    ,
                    'false'
                )
                ,
                (
                    'filters'
                    ,
                    '0:0'
                )
                ,
                (
                    'auto_configure'
                    ,
                    'true'
                )
                ,
                (
                    'auto_activate'
                    ,
                    'true'
                )
                ,
                (
                    'timeout'
                    ,
                    '5.0'
                )
                ,
                (
                    'transition_attempts'
                    ,
                    '3'
                )
                ,
            ]
    )

    launch_vcan0_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_vcan0_sender]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'do150_0000'
                )
                ,
                (
                    'interface'
                    ,
                    'vcan0'
                )
                ,
                (
                    'to_can_bus_topic'
                    ,
                    'vcan0/tx'
                )
                ,
                (
                    'enable_can_fd'
                    ,
                    'false'
                )
                ,
                (
                    'interval_sec'
                    ,
                    '0.01'
                )
                ,
                (
                    'auto_configure'
                    ,
                    'true'
                )
                ,
                (
                    'auto_activate'
                    ,
                    'true'
                )
                ,
                (
                    'timeout'
                    ,
                    '5.0'
                )
                ,
                (
                    'transition_attempts'
                    ,
                    '3'
                )
                ,
            ]
    )

    # Nodes
    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace='do150_0000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150'
                ,
            ]
        ,
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace='do150_0000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150'
                ,
            ]
        ,
    )

    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='do150_0000',
        output='screen',
        remappings=
            [
                (
                    '/diagnostics'
                    ,
                    'diagnostics'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'hz': 1.0
                    ,
                    'dev': ''
                    ,
                    'connected_topic': 'platform/wifi_connected'
                    ,
                    'connection_topic': 'platform/wifi_status'
                    ,
                }
                ,
            ]
        ,
    )

    node_imu_filter_madgwick = Node(
        name='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        namespace='do150_0000',
        output='screen',
        remappings=
            [
                (
                    'imu/data_raw'
                    ,
                    'sensors/imu_0/data_raw'
                )
                ,
                (
                    'imu/mag'
                    ,
                    'sensors/imu_0/magnetic_field'
                )
                ,
                (
                    'imu/data'
                    ,
                    'sensors/imu_0/data'
                )
                ,
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                imu_filter
                ,
            ]
        ,
    )

    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        namespace='do150_0000',
        output='screen',
        arguments=
            [
                'udp4'
                ,
                '--port'
                ,
                '11411'
                ,
            ]
        ,
    )

    node_lighting_node = Node(
        name='lighting_node',
        executable='lighting_node',
        package='clearpath_hardware_interfaces',
        namespace='do150_0000',
        output='screen',
        remappings=
            [
                (
                    '/diagnostics'
                    ,
                    'diagnostics'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'platform': 'dd150'
                    ,
                }
                ,
            ]
        ,
    )

    node_puma_control = Node(
        name='puma_control',
        executable='multi_puma_node',
        package='puma_motor_driver',
        namespace='do150_0000',
        output='screen',
        remappings=
            [
                (
                    '/diagnostics'
                    ,
                    'diagnostics'
                )
                ,
            ]
        ,
        parameters=
            [
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_dd150/platform/config/control.yaml'
                ,
            ]
        ,
    )

    node_pinout_control_node = Node(
        name='pinout_control_node',
        executable='pinout_control_node',
        package='clearpath_hardware_interfaces',
        namespace='do150_0000',
        output='screen',
        parameters=
            [
                {
                    'platform': 'dd150'
                    ,
                }
                ,
            ]
        ,
    )

    # Processes
    process_configure_mcu = ExecuteProcess(
        shell=True,
        cmd=
            [
                [
                    'export ROS_DOMAIN_ID=0;'
                    ,
                ]
                ,
                [
                    FindExecutable(name='ros2')
                    ,
                    ' service call platform/mcu/configure'
                    ,
                    ' clearpath_platform_msgs/srv/ConfigureMcu'
                    ,
                    ' "{domain_id: 0,'
                    ,
                    ' robot_namespace: \'do150_0000\'}"'
                    ,
                ]
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_diagnostic_updater_params)
    ld.add_action(launch_arg_diagnostic_aggregator_params)
    ld.add_action(launch_arg_foxglove_bridge_parameters)
    ld.add_action(launch_arg_imu_filter)
    ld.add_action(launch_platform)
    ld.add_action(launch_diagnostics)
    ld.add_action(launch_foxglove_bridge)
    ld.add_action(launch_vcan0_receiver)
    ld.add_action(launch_vcan0_sender)
    ld.add_action(node_battery_state_control)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_imu_filter_madgwick)
    ld.add_action(node_micro_ros_agent)
    ld.add_action(node_lighting_node)
    ld.add_action(node_puma_control)
    ld.add_action(node_pinout_control_node)
    ld.add_action(process_configure_mcu)
    return ld
