# Software License Agreement (BSD)
#
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    parameters = LaunchConfiguration('parameters')
    robot_namespace = LaunchConfiguration('robot_namespace')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'velodyne_lidar.yaml'
        ]))

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        namespace=namespace,
        parameters=[parameters],
        remappings=[
          ('/diagnostics', PathJoinSubstitution(['/', robot_namespace, 'diagnostics'])),
        ],
        output='screen'
    )

    velodyne_pointcloud_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[parameters],
        remappings=[
          ('/diagnostics', PathJoinSubstitution(['/', robot_namespace, 'diagnostics'])),
          ('/tf', PathJoinSubstitution(['/', robot_namespace, 'tf'])),
          ('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])),
          ('velodyne_points', 'points'),
        ],
    )

    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        namespace=namespace,
        output='screen',
        parameters=[parameters],
        remappings=[
          ('velodyne_points', 'points'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(velodyne_driver_node)
    ld.add_action(velodyne_pointcloud_node)
    ld.add_action(velodyne_laserscan_node)
    return ld
