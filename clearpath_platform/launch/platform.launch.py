from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages
    pkg_clearpath_control = FindPackageShare('clearpath_control')
    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')

    # Launch Arguments
    arg_platform_model = DeclareLaunchArgument(
        'platform_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )
    platform_model = LaunchConfiguration('platform_model')

    log_platform_model = LogInfo(msg=["Launching Clearpath platform model: ", platform_model])

    group_platform_action = GroupAction(
        actions=[
            # Launch clearpath_control/control.launch.py which is just robot_localization.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                [pkg_clearpath_control, 'launch', 'control.launch.py']))
            ),

            # Launch clearpath_control/teleop_base.launch.py which is various ways to tele-op
            # the robot but does not include the joystick. Also, has a twist mux.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                [pkg_clearpath_control, 'launch', 'teleop_base.launch.py']))
            ),

            # Launch clearpath_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                [pkg_clearpath_control, 'launch', 'teleop_joy.launch.py']))
            )
        ]
    )


    # Group for actions needed for the Clearpath Jackal J100 platform.
    group_j100_action = GroupAction(
        condition=LaunchConfigurationEquals('platform_model', 'j100'),
        actions=[
            # Wireless Watcher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
                )),
                launch_arguments=[('connected_topic', 'platform/wifi_connected')]
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
                    ' service call platform/mcu/set_domain_id ',
                    ' clearpath_platform_msgs/srv/SetDomainId ',
                    '"domain_id: ',
                    EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                    '"']
                ],
                shell=True,
            )
        ]
    )

    launch_description = IncludeLaunchDescription(
            PathJoinSubstitution([
                pkg_clearpath_platform_description,
                'launch',
                'description.launch.py']))

    ld = LaunchDescription()
    ld.add_action(arg_platform_model)
    ld.add_action(log_platform_model)
    ld.add_action(launch_description)
    ld.add_action(group_platform_action)
    ld.add_action(group_j100_action)
    return ld
