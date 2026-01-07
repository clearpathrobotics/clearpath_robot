from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_diagnostic_updater_params = DeclareLaunchArgument(
        'diagnostic_updater_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300/platform/config/diagnostic_updater.yaml',
        description='')

    diagnostic_updater_params = LaunchConfiguration('diagnostic_updater_params')

    launch_arg_diagnostic_aggregator_params = DeclareLaunchArgument(
        'diagnostic_aggregator_params',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300/platform/config/diagnostic_aggregator.yaml',
        description='')

    diagnostic_aggregator_params = LaunchConfiguration('diagnostic_aggregator_params')

    launch_arg_foxglove_bridge_parameters = DeclareLaunchArgument(
        'foxglove_bridge_parameters',
        default_value='/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300/platform/config/foxglove_bridge.yaml',
        description='')

    foxglove_bridge_parameters = LaunchConfiguration('foxglove_bridge_parameters')

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare('clearpath_ros2_socketcan_interface')
    pkg_canopen_inventus_bringup = FindPackageShare('canopen_inventus_bringup')

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
    launch_file_canopen_inventus = PathJoinSubstitution([
        pkg_canopen_inventus_bringup, 'launch', 'inventus.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300'
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
                    'a300_00000'
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
                    'a300_00000'
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
                    'a300_00000'
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
                    'a300_00000'
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
                    'a300_00000'
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

    launch_canopen_inventus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_canopen_inventus]),
        launch_arguments=
            [
                (
                    'namespace'
                    ,
                    'a300_00000/platform/bms'
                )
                ,
                (
                    'interface'
                    ,
                    'vcan1'
                )
                ,
                (
                    'battery_count'
                    ,
                    '2'
                )
                ,
                (
                    'master_id'
                    ,
                    '49'
                )
                ,
                (
                    'battery_0_id'
                    ,
                    '49'
                )
                ,
                (
                    'battery_1_id'
                    ,
                    '50'
                )
                ,
                (
                    'battery_2_id'
                    ,
                    '51'
                )
                ,
                (
                    'battery_3_id'
                    ,
                    '52'
                )
                ,
                (
                    'battery_4_id'
                    ,
                    '53'
                )
                ,
                (
                    'battery_5_id'
                    ,
                    '54'
                )
                ,
            ]
    )

    # Nodes
    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_hardware_interfaces',
        namespace='a300_00000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300'
                ,
            ]
        ,
    )

    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='a300_00000',
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

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_hardware_interfaces',
        namespace='a300_00000',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300'
                ,
            ]
        ,
    )

    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        namespace='a300_00000',
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
        namespace='a300_00000',
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
                    'platform': 'a300'
                    ,
                }
                ,
            ]
        ,
    )

    node_lynx_control = Node(
        name='lynx_control',
        executable='lynx_motor_driver',
        package='lynx_motor_driver',
        namespace='a300_00000',
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
                '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_a300/platform/config/control.yaml'
                ,
            ]
        ,
    )

    node_a300_fan_control = Node(
        name='a300_fan_control',
        executable='fan_control_node',
        package='clearpath_hardware_interfaces',
        namespace='a300_00000',
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
    )

    node_a300_sw_low_soc_cutoff = Node(
        name='a300_sw_low_soc_cutoff',
        executable='sw_low_soc_cutoff_node',
        package='clearpath_hardware_interfaces',
        namespace='a300_00000',
        output='screen',
    )

    node_pinout_control_node = Node(
        name='pinout_control_node',
        executable='pinout_control_node',
        package='clearpath_hardware_interfaces',
        namespace='a300_00000',
        output='screen',
        parameters=
            [
                {
                    'platform': 'a300'
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
                    ' robot_namespace: \'a300_00000\'}"'
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
    ld.add_action(launch_platform)
    ld.add_action(launch_diagnostics)
    ld.add_action(launch_foxglove_bridge)
    ld.add_action(launch_vcan0_receiver)
    ld.add_action(launch_vcan0_sender)
    ld.add_action(launch_canopen_inventus)
    ld.add_action(node_battery_state_control)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_micro_ros_agent)
    ld.add_action(node_lighting_node)
    ld.add_action(node_lynx_control)
    ld.add_action(node_a300_fan_control)
    ld.add_action(node_a300_sw_low_soc_cutoff)
    ld.add_action(node_pinout_control_node)
    ld.add_action(process_configure_mcu)
    return ld
