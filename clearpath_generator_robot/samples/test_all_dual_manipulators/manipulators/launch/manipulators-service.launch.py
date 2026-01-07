from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_manipulators = FindPackageShare('clearpath_manipulators')

    # Declare launch files
    launch_file_manipulators = PathJoinSubstitution([
        pkg_clearpath_manipulators, 'launch', 'manipulators.launch.py'])

    # Include launch files
    launch_manipulators = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_manipulators]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    '/home/lcamero/Workspaces/sample_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_manipulators'
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
                    'cpr_generic_e'
                )
                ,
                (
                    'launch_moveit'
                    ,
                    'false'
                )
                ,
                (
                    'delay_moveit'
                    ,
                    '5.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
                (
                    'control_delay'
                    ,
                    '1.0'
                )
                ,
            ]
    )

    # Nodes
    node_arm_5_ur_tool_comm = Node(
        name='arm_5_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_5_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_6_ur_tool_comm = Node(
        name='arm_6_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_6_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_7_ur_tool_comm = Node(
        name='arm_7_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_7_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_8_ur_tool_comm = Node(
        name='arm_8_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_8_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_9_ur_tool_comm = Node(
        name='arm_9_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_9_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_10_ur_tool_comm = Node(
        name='arm_10_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_10_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_11_ur_tool_comm = Node(
        name='arm_11_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_11_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_12_ur_tool_comm = Node(
        name='arm_12_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_12_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_13_ur_tool_comm = Node(
        name='arm_13_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_13_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_14_ur_tool_comm = Node(
        name='arm_14_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_14_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_15_ur_tool_comm = Node(
        name='arm_15_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_15_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_16_ur_tool_comm = Node(
        name='arm_16_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_16_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_17_ur_tool_comm = Node(
        name='arm_17_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_17_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_18_ur_tool_comm = Node(
        name='arm_18_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_18_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_19_ur_tool_comm = Node(
        name='arm_19_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_19_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_20_ur_tool_comm = Node(
        name='arm_20_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_20_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_21_ur_tool_comm = Node(
        name='arm_21_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_21_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_22_ur_tool_comm = Node(
        name='arm_22_ur_tool_comm',
        executable='tool_communication.py',
        package='ur_robot_driver',
        namespace='cpr_generic_e',
        output='screen',
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'tcp_port': 54321
                    ,
                    'device_name': '/tmp/arm_22_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_manipulators)
    ld.add_action(node_arm_5_ur_tool_comm)
    ld.add_action(node_arm_6_ur_tool_comm)
    ld.add_action(node_arm_7_ur_tool_comm)
    ld.add_action(node_arm_8_ur_tool_comm)
    ld.add_action(node_arm_9_ur_tool_comm)
    ld.add_action(node_arm_10_ur_tool_comm)
    ld.add_action(node_arm_11_ur_tool_comm)
    ld.add_action(node_arm_12_ur_tool_comm)
    ld.add_action(node_arm_13_ur_tool_comm)
    ld.add_action(node_arm_14_ur_tool_comm)
    ld.add_action(node_arm_15_ur_tool_comm)
    ld.add_action(node_arm_16_ur_tool_comm)
    ld.add_action(node_arm_17_ur_tool_comm)
    ld.add_action(node_arm_18_ur_tool_comm)
    ld.add_action(node_arm_19_ur_tool_comm)
    ld.add_action(node_arm_20_ur_tool_comm)
    ld.add_action(node_arm_21_ur_tool_comm)
    ld.add_action(node_arm_22_ur_tool_comm)
    return ld
