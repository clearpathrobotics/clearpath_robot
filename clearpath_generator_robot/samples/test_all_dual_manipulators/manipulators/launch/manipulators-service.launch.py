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
                    '/home/lcamero/Workspaces/jazzy_ws/src/clearpath_robot/clearpath_generator_robot/samples/test_all_dual_manipulators'
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
    node_arm_1_gripper_controller = Node(
        name='arm_1_gripper_controller',
        executable='franka_gripper_node',
        package='franka_gripper',
        namespace='cpr_generic_e/manipulators',
        output='screen',
        remappings=
            [
                (
                    '~/joint_states'
                    ,
                    '/cpr_generic_e/platform/joint_states'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'joint_names': ['arm_1_gripper_fr3_finger_joint1', 'arm_1_gripper_fr3_finger_joint2']
                    ,
                    'state_publish_rate': 15
                    ,
                    'feedback_publish_rate': 30
                    ,
                    'default_speed': 0.1
                    ,
                    'default_grasp_epsilon': {'inner': 0.005, 'outer': 0.005}
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_5_gripper_controller = Node(
        name='arm_5_gripper_controller',
        executable='franka_gripper_node',
        package='franka_gripper',
        namespace='cpr_generic_e/manipulators',
        output='screen',
        remappings=
            [
                (
                    '~/joint_states'
                    ,
                    '/cpr_generic_e/platform/joint_states'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'joint_names': ['arm_5_gripper_fp3_finger_joint1', 'arm_5_gripper_fp3_finger_joint2']
                    ,
                    'state_publish_rate': 15
                    ,
                    'feedback_publish_rate': 30
                    ,
                    'default_speed': 0.1
                    ,
                    'default_grasp_epsilon': {'inner': 0.005, 'outer': 0.005}
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_9_gripper_controller = Node(
        name='arm_9_gripper_controller',
        executable='franka_gripper_node',
        package='franka_gripper',
        namespace='cpr_generic_e/manipulators',
        output='screen',
        remappings=
            [
                (
                    '~/joint_states'
                    ,
                    '/cpr_generic_e/platform/joint_states'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'robot_ip': '192.168.131.40'
                    ,
                    'joint_names': ['arm_9_gripper_fer_finger_joint1', 'arm_9_gripper_fer_finger_joint2']
                    ,
                    'state_publish_rate': 15
                    ,
                    'feedback_publish_rate': 30
                    ,
                    'default_speed': 0.1
                    ,
                    'default_grasp_epsilon': {'inner': 0.005, 'outer': 0.005}
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

    node_arm_23_ur_tool_comm = Node(
        name='arm_23_ur_tool_comm',
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
                    'device_name': '/tmp/arm_23_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_24_ur_tool_comm = Node(
        name='arm_24_ur_tool_comm',
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
                    'device_name': '/tmp/arm_24_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_25_ur_tool_comm = Node(
        name='arm_25_ur_tool_comm',
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
                    'device_name': '/tmp/arm_25_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_26_ur_tool_comm = Node(
        name='arm_26_ur_tool_comm',
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
                    'device_name': '/tmp/arm_26_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_27_ur_tool_comm = Node(
        name='arm_27_ur_tool_comm',
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
                    'device_name': '/tmp/arm_27_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_28_ur_tool_comm = Node(
        name='arm_28_ur_tool_comm',
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
                    'device_name': '/tmp/arm_28_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_29_ur_tool_comm = Node(
        name='arm_29_ur_tool_comm',
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
                    'device_name': '/tmp/arm_29_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_30_ur_tool_comm = Node(
        name='arm_30_ur_tool_comm',
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
                    'device_name': '/tmp/arm_30_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_31_ur_tool_comm = Node(
        name='arm_31_ur_tool_comm',
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
                    'device_name': '/tmp/arm_31_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_32_ur_tool_comm = Node(
        name='arm_32_ur_tool_comm',
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
                    'device_name': '/tmp/arm_32_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_33_ur_tool_comm = Node(
        name='arm_33_ur_tool_comm',
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
                    'device_name': '/tmp/arm_33_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    node_arm_34_ur_tool_comm = Node(
        name='arm_34_ur_tool_comm',
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
                    'device_name': '/tmp/arm_34_gripper'
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_manipulators)
    ld.add_action(node_arm_1_gripper_controller)
    ld.add_action(node_arm_5_gripper_controller)
    ld.add_action(node_arm_9_gripper_controller)
    ld.add_action(node_arm_17_ur_tool_comm)
    ld.add_action(node_arm_18_ur_tool_comm)
    ld.add_action(node_arm_19_ur_tool_comm)
    ld.add_action(node_arm_20_ur_tool_comm)
    ld.add_action(node_arm_21_ur_tool_comm)
    ld.add_action(node_arm_22_ur_tool_comm)
    ld.add_action(node_arm_23_ur_tool_comm)
    ld.add_action(node_arm_24_ur_tool_comm)
    ld.add_action(node_arm_25_ur_tool_comm)
    ld.add_action(node_arm_26_ur_tool_comm)
    ld.add_action(node_arm_27_ur_tool_comm)
    ld.add_action(node_arm_28_ur_tool_comm)
    ld.add_action(node_arm_29_ur_tool_comm)
    ld.add_action(node_arm_30_ur_tool_comm)
    ld.add_action(node_arm_31_ur_tool_comm)
    ld.add_action(node_arm_32_ur_tool_comm)
    ld.add_action(node_arm_33_ur_tool_comm)
    ld.add_action(node_arm_34_ur_tool_comm)
    return ld
