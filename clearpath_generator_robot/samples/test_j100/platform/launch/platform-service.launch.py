from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_diagnostic_updater_params = DeclareLaunchArgument(
        'diagnostic_updater_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100/platform/config/diagnostic_updater.yaml',
        description='')

    diagnostic_updater_params = LaunchConfiguration('diagnostic_updater_params')

    launch_arg_diagnostic_aggregator_params = DeclareLaunchArgument(
        'diagnostic_aggregator_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100/platform/config/diagnostic_aggregator.yaml',
        description='')

    diagnostic_aggregator_params = LaunchConfiguration('diagnostic_aggregator_params')

    launch_arg_foxglove_bridge_parameters = DeclareLaunchArgument(
        'foxglove_bridge_parameters',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100/platform/config/foxglove_bridge.yaml',
        description='')

    foxglove_bridge_parameters = LaunchConfiguration('foxglove_bridge_parameters')

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100/platform/config/imu_filter.yaml',
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])
    launch_file_foxglove_bridge = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'foxglove_bridge.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100'
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
                    'j100_0000'
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
                    'j100_0000'
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
                    'j100_0000'
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

    # Nodes
    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace='j100_0000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100'
                ,
            ]
        ,
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace='j100_0000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_j100'
                ,
            ]
        ,
    )

    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='j100_0000',
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
        namespace='j100_0000',
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
        namespace='j100_0000',
        output='screen',
        arguments=
            [
                'serial'
                ,
                '--dev'
                ,
                '/dev/clearpath/j100'
                ,
            ]
        ,
    )

    node_nmea_topic_driver = Node(
        name='nmea_topic_driver',
        executable='nmea_topic_driver',
        package='nmea_navsat_driver',
        namespace='j100_0000',
        output='screen',
        remappings=
            [
                (
                    'nmea_sentence'
                    ,
                    'sensors/gps_0/nmea_sentence'
                )
                ,
                (
                    'fix'
                    ,
                    'sensors/gps_0/fix'
                )
                ,
                (
                    'heading'
                    ,
                    'sensors/gps_0/heading'
                )
                ,
                (
                    'time_reference'
                    ,
                    'sensors/gps_0/time_reference'
                )
                ,
                (
                    'vel'
                    ,
                    'sensors/gps_0/vel'
                )
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
                    ' robot_namespace: \'j100_0000\'}"'
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
    ld.add_action(node_battery_state_control)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_imu_filter_madgwick)
    ld.add_action(node_micro_ros_agent)
    ld.add_action(node_nmea_topic_driver)
    ld.add_action(process_configure_mcu)
    return ld
