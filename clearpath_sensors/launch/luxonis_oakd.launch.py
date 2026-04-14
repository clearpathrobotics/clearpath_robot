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
import yaml

# Pipeline types that produce stereo/depth output
DEPTH_PIPELINES = ['RGBD', 'RGBStereo', 'Stereo', 'Depth', 'DepthToF', 'StereoToF']

CAMERAS = [
    'rgb',
    'stereo',
]

CAMERA_RENAMES = {
    'rgb': 'color',
    'stereo': 'stereo',
}

IMAGES = [
    'image_raw',
]

TOPICS = [
    'camera_info',
]

PREVIEWS = [
    'rgb',
]

TRANSPORTS = [
    'compressed',
    'compressedDepth',
    'theora',
    'ffmpeg',
    'zstd',
    'foxglove',
]

OTHERS = [
    'imu/data',
    'nn/spatial_detections',
]


def launch_setup(context):
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')

    name = os.path.basename(namespace.perform(context))

    # Determine pipeline type from parameters file
    params_file = parameters.perform(context)
    pipeline_type = 'RGBD'
    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
        for key, val in params.items():
            if isinstance(val, dict):
                ros_params = val.get('ros__parameters', val)
                # Check nested camera.i_pipeline_type
                camera = ros_params.get('camera', {})
                if isinstance(camera, dict) and 'i_pipeline_type' in camera:
                    pipeline_type = camera['i_pipeline_type']
                # Check flattened camera.i_pipeline_type
                if 'camera.i_pipeline_type' in ros_params:
                    pipeline_type = ros_params['camera.i_pipeline_type']
    except Exception:
        pass

    remappings = [
        ('/diagnostics', 'diagnostics'),
    ]

    for camera in CAMERAS:
        renamed = CAMERA_RENAMES[camera]
        for image in IMAGES:
            remappings.append(
                (f'~/{camera}/{image}', f'{renamed}/image'),
            )
            for transport in TRANSPORTS:
                remappings.append(
                    (f'~/{camera}/{image}/{transport}', f'{renamed}/{transport}'),
                )
        for topic in TOPICS:
            remappings.append(
                (f'~/{camera}/{topic}', f'{renamed}/{topic}'),
            )

    for camera in PREVIEWS:
        renamed = CAMERA_RENAMES[camera]
        for image in IMAGES:
            remappings.append(
                (f'~/{camera}/preview/{image}', f'{renamed}/preview/image'),
            )
            for transport in TRANSPORTS:
                remappings.append(
                    (f'~/{camera}/preview/{image}/{transport}',
                     f'{renamed}/preview/{transport}'),
                )
        for topic in TOPICS:
            remappings.append(
                (f'~/{camera}/preview/{topic}', f'{renamed}/preview/{topic}'),
            )

    for topic in OTHERS:
        remappings.append(('~/%s' % topic, '%s' % topic))

    depthai_oakd_node = ComposableNode(
        package='depthai_ros_driver',
        name=name,
        namespace=namespace,
        plugin='depthai_ros_driver::Camera',
        parameters=[parameters],
        remappings=remappings,
    )

    composable_nodes = [depthai_oakd_node]

    if pipeline_type in DEPTH_PIPELINES:
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
        composable_nodes.append(depthai_pcl_node)

    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
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
