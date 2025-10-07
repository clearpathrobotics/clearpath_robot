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
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

TRANSPORTS = [
    'compressed',
    'compressedDepth',
    'theora',
    'ffmpeg',
    'zstd',
    'foxglove',
]


def launch_setup(context):
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    name = os.path.basename(namespace.perform(context))

    remappings = [
            ('~/imu/data', 'imu/data'),
            ('~/nn/spatial_detections', 'nn/spatial_detections'),
            ('~/rgb/camera_info', 'color/camera_info'),
            ('~/rgb/image_raw', 'color/image'),
            ('~/rgb/preview/image_raw', 'color/preview/image'),
            ('~/stereo/camera_info', 'stereo/camera_info'),
            ('~/stereo/image_raw', 'stereo/image'),
            ('/diagnostics', 'diagnostics'),
    ]

    for transport in TRANSPORTS:
        remappings.extend([
            (f'~/rgb/image_raw/{transport}', f'color/{transport}'),
            (f'~/rgb/preview/image_raw/{transport}', f'color/preview/{transport}'),
            (f'~/stereo/image_raw/{transport}', f'stereo/{transport}')
        ])

    depthai_oakd_node = ComposableNode(
        package='depthai_ros_driver',
        name=name,
        namespace=namespace,
        plugin='depthai_ros_driver::Camera',
        parameters=[parameters],
        remappings=remappings,
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    depthai_pcl_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='point_cloud_xyz_node',
        namespace=namespace,
        remappings=[
            ('image_rect', 'stereo/image'),
            ('camera_info', 'stereo/camera_info'),
        ],
    )

    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
          depthai_oakd_node,
          depthai_pcl_node,
        ],
        output='screen'
    )

    return [image_processing_container]


def generate_launch_description():
    # Launch configurations
    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'luxonis_oakd.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
