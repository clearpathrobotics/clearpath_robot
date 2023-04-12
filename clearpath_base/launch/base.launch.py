from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')

    # Launch Arguments
    arg_robot_model = DeclareLaunchArgument(
        'robot_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )
    robot_model = LaunchConfiguration('robot_model')
 
    log_platform_model = LogInfo(msg=["Launching Clearpath base for platform model: ", LaunchConfiguration('robot_model')])

    # Launch clearpath_control/control.launch.py which is just robot_localization.
    launch_clearpath_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("clearpath_control"), 'launch', 'control.launch.py'])))

    # Launch clearpath_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_clearpath_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("clearpath_control"), 'launch', 'teleop_base.launch.py'])))

    # Launch clearpath_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_clearpath_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("clearpath_control"), 'launch', 'teleop_joy.launch.py'])))

    # Group for actions needed for the Clearpath Jackal J100 platform.
    group_j100_action = GroupAction(
        condition=LaunchConfigurationEquals('robot_model', 'j100'),
        actions=[
            # Wireless Watcher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
                )),
                launch_arguments=[('connected_topic', 'wifi_connected')]
            ),

            # MicroROS Agent
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                arguments=['serial', '--dev', '/dev/clearpath/j100'],
                output='screen'),

            # Set ROS_DOMAIN_ID
            ExecuteProcess(
                cmd=[
                    ['export ROS_DOMAIN_ID=0;'],
                    [FindExecutable(name='ros2'),
                    ' service call /set_domain_id ',
                    ' clearpath_platform_msgs/srv/SetDomainId ',
                    '"domain_id: ',
                    EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                    '"']
                ],
                shell=True,
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arg_robot_model)
    ld.add_action(log_platform_model)
    ld.add_action(launch_clearpath_control)
    ld.add_action(launch_clearpath_teleop_base)
    ld.add_action(launch_clearpath_teleop_joy)
    ld.add_action(group_j100_action)
    return ld
