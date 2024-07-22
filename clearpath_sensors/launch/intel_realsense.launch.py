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

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.substitutions import FindPackageShare

CAMERAS = [
    'color',
    'depth',
    'infra1',
    'infra2',
    'aligned_depth_to_color',
    'aligned_depth_to_infra1',
    'aligned_dpeth_to_infra2',
]

IMAGES = [
    'image_raw',
    'image_rect_raw',
]

TOPICS = [
    'camera_info',
    'metadata',
]

OTHERS = [
    'rgbd',
    'extrinsics/depth_to_color',
    'extrinsics/depth_to_infra1',
    'extrinsics/depth_to_infra2',
]


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    robot_namespace = LaunchConfiguration('robot_namespace')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'intel_realsense.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='')

    remappings = [
        ('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])),
        ('~/depth/color/points', 'points'),
    ]

    for camera in CAMERAS:
        for image in IMAGES:
            remappings.extend([
                ('~/%s/%s' % (camera, image), '%s/image' % camera),
                ('~/%s/%s/compressed' % (camera, image), '%s/image/compressed' % camera),
                ('~/%s/%s/compressedDepth' % (camera, image), '%s/image/compressedDepth' % camera),
                ('~/%s/%s/theora' % (camera, image), '%s/image/theora' % camera),
            ])
        for topic in TOPICS:
            remappings.append(
                ('~/%s/%s' % (camera, topic), '%s/%s' % (camera, topic))
            )

    for topic in OTHERS:
        remappings.append(('~/%s' % topic, '%s' % topic))

    name = 'intel_realsense'
    realsense2_camera_node = Node(
        package='realsense2_camera',
        namespace=namespace,
        name=name,
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen',
        remappings=remappings,
    )

    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_namespace)
    ld.add_action(realsense2_camera_node)
    ld.add_action(image_processing_container)
    return ld
