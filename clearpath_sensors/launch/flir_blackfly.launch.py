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
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    param_mapping_file = LaunchConfiguration('param_mapping_file')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'flir_blackfly.yaml'
        ]))
    
    arg_param_mapping_file = DeclareLaunchArgument(
        'param_mapping_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('spinnaker_camera_driver'), 'config',
             'blackfly_s.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='platform/sensors/camera_0')

    blackfly_camera_node = Node(
        package='spinnaker_camera_driver',
        namespace=namespace,
        name='flir_blackfly',
        executable='camera_driver_node',
        parameters=[parameters, {'parameter_file': param_mapping_file,}],
        output='screen',
    )
    

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_param_mapping_file)
    ld.add_action(arg_namespace)
    ld.add_action(blackfly_camera_node)
    return ld
