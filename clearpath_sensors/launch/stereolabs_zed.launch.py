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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.substitutions import FindPackageShare

HIDDEN = [
    'odom',
    'path_map',
    'path_odom',
    'pose',
    'pose/status',
    'pose_with_covariance',
    'left_cam_imu_transform',
]

CAMERAS = [
    # Depth
    ('depth', 'depth'),
    # Left
    ('left', 'left'),
    ('left_gray', 'left/gray'),
    ('left_raw', 'left/raw'),
    ('left_raw_gray', 'left/raw_gray'),
    # Color
    ('rgb', 'color'),
    ('rgb_gray', 'color/gray'),
    ('rgb_raw', 'color/raw'),
    ('rgb_raw_gray', 'color/raw_gray'),
    # Right
    ('right', 'right'),
    ('right_gray', 'right/gray'),
    ('right_raw', 'right/raw'),
    ('right_raw_gray', 'right/raw_gray'),
    # Stereo
    ('stereo', 'stereo'),
    ('stereo_raw', 'stereo/raw')
]

OTHERS = [
    'temperature/imu',
    'temperature/left',
    'temperature/right',
    'imu/data',
    'imu/data_raw',
    'imu/mag',
    'atm_press',
    'confidence/confidence_map',
    'disparity/disparity_image',
    'point_cloud/cloud_registered',
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
          'stereolabs_zed.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='')

    remappings = []

    # Hidden Topics
    for hidden in HIDDEN:
        remappings.append(
            ('~/%s' % hidden, PathJoinSubstitution(['/', namespace, '_' + hidden]))
        )

    # Cameras
    for old, new in CAMERAS:
        if 'depth' in old:
            image = 'depth_registered'
            remappings.append(
                ('~/%s/depth_info' % (old),
                    PathJoinSubstitution(['/', namespace, new, 'depth_info'])),
            )
        elif 'raw_gray' in old:
            image = 'image_raw_gray'
        elif 'raw' in old:
            image = 'image_raw_color'
        elif 'gray' in old:
            image = 'image_rect_gray'
        else:
            image = 'image_rect_color'
        remappings.extend([
            ('~/%s/camera_info' % (old),
                PathJoinSubstitution(['/', namespace, new, 'camera_info'])),
            ('~/%s/%s' % (old, image),
                PathJoinSubstitution(['/', namespace, new, 'image'])),
            ('~/%s/%s/compressed' % (old, image),
                PathJoinSubstitution(['/', namespace, new, 'compressed'])),
            ('~/%s/%s/compressedDepth' % (old, image),
                PathJoinSubstitution(['/', namespace, new, 'compressedDepth'])),
            ('~/%s/%s/theora' % (old, image),
                PathJoinSubstitution(['/', namespace, new, 'theora'])),
        ])

    # Others
    for topic in OTHERS:
        remappings.append(
            ('~/%s' % topic,
                PathJoinSubstitution(['/', namespace, topic])),
        )

    # Transforms
    remappings.append(('/tf', PathJoinSubstitution(['/', robot_namespace, 'tf'])))
    remappings.append(('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])))

    stereolabs_zed_node = Node(
        package='zed_wrapper',
        namespace=namespace,
        executable='zed_wrapper',
        name='stereolabs_zed',
        output='screen',
        parameters=[parameters],
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
    ld.add_action(stereolabs_zed_node)
    ld.add_action(image_processing_container)
    return ld
