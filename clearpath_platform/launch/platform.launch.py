# Software License Agreement (BSD)
#
# @author    Tony Baltovski <tbaltovski@clearpathrobotics.com>
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
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
    pkg_clearpath_platform = FindPackageShare('clearpath_platform')
    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')

    # Launch Arguments
    arg_platform_model = DeclareLaunchArgument(
        'platform_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )

    arg_imu_filter_config = DeclareLaunchArgument(
        'imu_filter_config',
        default_value=PathJoinSubstitution([
          pkg_clearpath_platform, 'config', 'imu_filter.yaml'])
    )

    platform_model = LaunchConfiguration('platform_model')
    config_imu_filter = LaunchConfiguration('imu_filter_config')

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
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(PathJoinSubstitution(
            #         [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
            #     )),
            #     launch_arguments=[('connected_topic', 'platform/wifi_connected')]
            # ),

            # Madgwick Filter
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_node',
                output='screen',
                parameters=[config_imu_filter],
                remappings=[
                  ('imu/data_raw', 'platform/sensors/imu_0/data_raw'),
                  ('imu/mag', 'platform/sensors/imu_0/magnetic_field'),
                  ('imu/data', 'platform/sensors/imu_0/data')
                ],
                condition=LaunchConfigurationEquals('platform_model', 'j100')
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
    ld.add_action(arg_imu_filter_config)
    ld.add_action(log_platform_model)
    ld.add_action(launch_description)
    ld.add_action(group_platform_action)
    ld.add_action(group_j100_action)
    return ld
