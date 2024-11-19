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
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    input_ns = LaunchConfiguration('input_ns')
    output_ns = LaunchConfiguration('output_ns')
    container = LaunchConfiguration('container')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'image_rectify.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='sensors/camera_0')

    arg_input_ns = DeclareLaunchArgument(
        'input_ns',
        default_value='color'
    )

    arg_output_ns = DeclareLaunchArgument(
        'output_ns',
        default_value='rectify'
    )

    arg_container = DeclareLaunchArgument(
        'container',
        default_value='image_processing_container'
    )

    # Rectify composable node
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name=PythonExpression(["'image_rectify_", input_ns, "'"]),
            namespace=namespace,
            # Remap subscribers and publishers
            remappings=[
                ('image',  PathJoinSubstitution([input_ns, 'image'])),
                ('rectify', PathJoinSubstitution([output_ns, 'image'])),
                ('rectify/compressed', PathJoinSubstitution([output_ns, 'compressed'])),
                ('rectify/compressedDepth', PathJoinSubstitution([output_ns, 'compressedDepth'])),
                ('rectify/theora', PathJoinSubstitution([output_ns, 'theora'])),
            ],
            parameters=[parameters],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    # Create container if none provided, by default look for `image_processing_node`
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_processing_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # Use default container that should have been launched by the camera launch file
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=PythonExpression(["'", namespace, '/', container, "'"])
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(arg_input_ns)
    ld.add_action(arg_output_ns)
    ld.add_action(arg_container)
    ld.add_action(image_processing_container)
    ld.add_action(load_composable_nodes)
    return ld
