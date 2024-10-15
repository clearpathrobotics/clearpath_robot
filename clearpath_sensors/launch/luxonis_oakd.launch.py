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


def launch_setup(context):
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    name = os.path.basename(namespace.perform(context))

    depthai_oakd_node = ComposableNode(
        package='depthai_ros_driver',
        name=name,
        namespace=namespace,
        plugin='depthai_ros_driver::Camera',
        parameters=[parameters],
        remappings=[
            ('~/imu/data', 'imu/data'),
            ('~/nn/spatial_detections', 'nn/spatial_detections'),
            ('~/rgb/camera_info', 'color/camera_info'),
            ('~/rgb/image_raw', 'color/image'),
            ('~/rgb/image_raw/compressed', 'color/compressed'),
            ('~/rgb/image_raw/compressedDepth', 'color/compressedDepth'),
            ('~/rgb/image_raw/ffmpeg', 'color/ffmpeg'),
            ('~/rgb/image_raw/theora', 'color/theora'),
            ('~/rgb/preview/image_raw', 'color/image'),
            ('~/rgb/preview/image_raw/compressed', 'color/compressed'),
            ('~/rgb/preview/image_raw/compressedDepth', 'color/compressedDepth'),
            ('~/rgb/preview/image_raw/ffmpeg', 'color/ffmpeg'),
            ('~/rgb/preview/image_raw/theora', 'color/theora'),
            ('~/stereo/camera_info', 'stereo/camera_info'),
            ('~/stereo/image_raw', 'stereo/image'),
            ('~/stereo/image_raw/compressed', 'stereo/compressed'),
            ('~/stereo/image_raw/compressedDepth', 'stereo/compressedDepth'),
            ('~/stereo/image_raw/ffmpeg', 'stereo/ffmpeg'),
            ('~/stereo/image_raw/theora', 'stereo/theora'),
            ('/diagnostics', 'diagnostics'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    depthai_pcl_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzrgbNode',
        name='point_cloud_xyzrgb_node',
        namespace=namespace,
        remappings=[
            ('depth_registered/image_rect', 'stereo/image'),
            ('rgb/image_rect_color', 'color/image'),
            ('rgb/camera_info', 'color/camera_info'),
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
          'intel_realsense.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
