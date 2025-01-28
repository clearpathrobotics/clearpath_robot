# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    container = LaunchConfiguration('container')
    input_mag = LaunchConfiguration('input_mag')
    input_raw = LaunchConfiguration('input_raw')
    output = LaunchConfiguration('output')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
            FindPackageShare('clearpath_sensors'),
            'config',
            'imu_filter.yaml'
        ])
    )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='sensors/imu_0'
    )

    arg_container = DeclareLaunchArgument(
        'container',
        default_value=''
    )

    arg_filter = DeclareLaunchArgument(
        'filter',
        choices=['none', 'madgwick'],
        default_value='none'
    )

    arg_input_mag = DeclareLaunchArgument(
        'input_mag',
        default_value='mag',
    )

    arg_input_raw = DeclareLaunchArgument(
        'input_raw',
        default_value='data_raw'
    )

    arg_output = DeclareLaunchArgument(
        'output',
        default_value='data'
    )

    # Filters
    composable_nodes = [
        ComposableNode(
            package='imu_filter_madgwick',
            plugin='ImuFilterMadgwickRos',
            name='imu_filter_madgwick',
            namespace=namespace,
            parameters=[parameters],
            condition=LaunchConfigurationEquals('filter', 'madgwick'),
            remappings=[
                ('imu/data', output),
                ('imu/data_raw', input_raw),
                ('imu/mag', input_mag),
                ('/tf', 'tf')
            ]
        )
    ]

    # Create container
    imu_filter_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='imu_filter_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # Use container launched by IMU
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=PythonExpression(["'", namespace, '/', container, "'"])
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(arg_container)
    ld.add_action(arg_filter)
    ld.add_action(arg_input_mag)
    ld.add_action(arg_input_raw)
    ld.add_action(arg_output)
    ld.add_action(imu_filter_container)
    ld.add_action(load_composable_nodes)
    return ld
