#!/usr/bin/env python3

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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from clearpath_config.manipulators.types.arms import (
  BaseKinova,
  Franka,
  KinovaGen3Dof6,
  KinovaGen3Dof7,
  KinovaGen3Lite,
  UniversalRobots
)
from clearpath_config.manipulators.types.grippers import FrankaGripper
from clearpath_generator_common.common import LaunchFile, Package
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_robot.launch import platforms  # noqa: F401
from clearpath_generator_robot.launch.platform import PlatformLaunch
from clearpath_generator_robot.launch.sensors import SensorLaunch


class RobotLaunchGenerator(LaunchGenerator):

    def generate_sensors(self) -> None:
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        for sensor in sensors:
            if sensor.launch_enabled:
                sensor_launch = SensorLaunch(
                    sensor,
                    self.namespace,
                    self.sensors_launch_path,
                    self.sensors_params_path,
                )
                # Generate sensor launch file
                sensor_launch.generate()
                # Include sensor launch in top level sensors launch file
                sensors_service_launch_writer.add(sensor_launch.launch_file)

        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add(self.platform_launch_file)

        # Per-platform launch composition (MCU, common, and platform-specific components).
        # The concrete subclass is selected from the PlatformLaunch registry by platform_model.
        platform_launch = PlatformLaunch.get(self.platform_model)(
            self.clearpath_config,
            self.platform_params_path,
            self.setup_path,
        )
        for component in platform_launch.get_components():
            platform_service_launch_writer.add(component)

        platform_service_launch_writer.generate_file()

        os.makedirs(os.path.dirname(self.platform_extras_launch_path), exist_ok=True)
        platform_extras_service_launch_writer = LaunchWriter(
            self.platform_extras_service_launch_file)
        platform_extras_service_launch_writer.add(self.platform_extras_launch_file)

        for launch in self.clearpath_config.platform.extras.launch:
            extra_launch = LaunchFile(
                name=(os.path.basename(launch.path)).split('.')[0],
                path=os.path.dirname(launch.path),
                package=Package(launch.package),
                args=[
                    (key, launch.args[key]) for key in launch.args
                ]
            )
            platform_extras_service_launch_writer.add(extra_launch)

        platform_extras_service_launch_writer.generate_file()

    def generate_manipulators(self) -> None:
        manipulator_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        for arm in self.clearpath_config.manipulators.get_all_arms():
            # Universal Robots Tool Communication
            if arm.MANIPULATOR_MODEL == UniversalRobots.MANIPULATOR_MODEL:
                node = LaunchFile.Node(
                    name=f'{arm.name}_ur_tool_comm',
                    package='ur_robot_driver',
                    executable='tool_communication.py',
                    namespace=self.namespace,
                    parameters=[{
                        'robot_ip': arm.ip,
                        'tcp_port': 54321,
                        'device_name': f'/tmp/{arm.name}_gripper'
                    }],
                )
                manipulator_service_launch_writer.add_node(node)
                # Delay controllers
                self.manipulators_launch_file.args.append(
                    ('control_delay', '1.0')
                )
            # Franka Hand Communication
            if arm.MANIPULATOR_MODEL == Franka.MANIPULATOR_MODEL:
                if arm.gripper:
                    if arm.gripper.MANIPULATOR_MODEL == FrankaGripper.MANIPULATOR_MODEL:
                        node = LaunchFile.Node(
                            name=f'{arm.gripper.name}_controller',
                            package='franka_gripper',
                            executable='franka_gripper_node',
                            namespace=f'{self.namespace}/manipulators',
                            parameters=[{
                                'robot_ip': arm.ip,
                                'joint_names': [
                                    f'{arm.gripper.name}_{arm.gripper.arm_id}_finger_joint1',
                                    f'{arm.gripper.name}_{arm.gripper.arm_id}_finger_joint2'
                                ],
                                'state_publish_rate': 15,  # [Hz]
                                'feedback_publish_rate': 30,  # [Hz]
                                'default_speed': 0.1,  # [m/s]
                                'default_grasp_epsilon': {
                                    'inner': 0.005,  # [m]
                                    'outer': 0.005  # [m]
                                }
                            }],
                            remappings=[
                                ('~/joint_states', f'/{self.namespace}/platform/joint_states')
                            ]
                        )
                        manipulator_service_launch_writer.add_node(node)
            # Kinova Vision
            if (arm.MANIPULATOR_MODEL == KinovaGen3Dof6.MANIPULATOR_MODEL or
                    arm.MANIPULATOR_MODEL == KinovaGen3Dof7.MANIPULATOR_MODEL or
                    arm.MANIPULATOR_MODEL == KinovaGen3Lite.MANIPULATOR_MODEL):
                if (arm.get_urdf_parameters().get(BaseKinova.VISION, False)):
                    depth_node_parameters = {
                        'camera_type': 'depth',
                        'camera_name': 'depth',
                        'camera_info_url_default':
                        'package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini',
                        'camera_info_url_user': '',
                        'stream_config': 'rtspsrc location=rtsp://'
                            + arm.ip
                            + '/depth latency=30'
                            + ' ! '
                            + 'rtpgstdepay',
                        'frame_id': f'{arm.name}_camera_depth_frame',
                        'max_pub_rate': 30.0,
                    }
                    depth_node = LaunchFile.Node(
                        name=f'{arm.name}_depth_camera',
                        package='kinova_vision',
                        executable='kinova_vision_node',
                        namespace=f'{self.namespace}/manipulators',
                        parameters=[depth_node_parameters],
                        remappings=[
                            ('camera_info', '~/camera_info'),
                            ('image_raw', '~/image_raw'),
                            ('image_raw/compressed', '~/image_raw/compressed'),
                            ('image_raw/compressedDepth', '~/image_raw/compressedDepth'),
                            ('image_raw/theora', '~/image_raw/theora'),
                            ('image_raw/ffmpeg', '~/image_raw/ffmpeg'),
                            ('image_raw/zstd', '~/image_raw/zstd'),
                        ]
                    )
                    color_node_parameters = {
                        'camera_type': 'color',
                        'camera_name': 'color',
                        'camera_info_url_default':
                        'package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini',
                        'camera_info_url_user': '',
                        'stream_config': 'rtspsrc location=rtsp://'
                            + arm.ip
                            + '/color latency=30'
                            + ' ! rtph264depay ! avdec_h264 ! videoconvert',
                        'frame_id': f'{arm.name}_camera_color_frame',
                        'max_pub_rate': 30.0,
                    }
                    color_node = LaunchFile.Node(
                        name=f'{arm.name}_color_camera',
                        package='kinova_vision',
                        executable='kinova_vision_node',
                        namespace=f'{self.namespace}/manipulators',
                        parameters=[color_node_parameters],
                        remappings=[
                            ('camera_info', '~/camera_info'),
                            ('image_raw', '~/image_raw'),
                            ('image_raw/compressed', '~/image_raw/compressed'),
                            ('image_raw/compressedDepth', '~/image_raw/compressedDepth'),
                            ('image_raw/theora', '~/image_raw/theora'),
                            ('image_raw/ffmpeg', '~/image_raw/ffmpeg'),
                            ('image_raw/zstd', '~/image_raw/zstd'),
                        ]
                    )

                    pointcloud_node = LaunchFile.ComposableNodeContainer(
                        name=f'{arm.name}_depth_proc_container',
                        namespace=f'{self.namespace}/manipulators',
                        remappings=[
                            ('/tf', f'/{self.namespace}/tf'),
                            ('/tf_static', f'/{self.namespace}/tf_static')
                        ],
                        composable_node_descriptions=[
                            LaunchFile.ComposableNode(
                                name=f'{arm.name}_depth_upsampled',
                                package='depth_image_proc',
                                plugin='depth_image_proc::RegisterNode',
                                namespace=f'{self.namespace}/manipulators',
                                parameters=[{'fill_upsampling_holes': True}],
                                remappings=[
                                    ('rgb/camera_info',
                                        f'{arm.name}_color_camera/camera_info'),
                                    ('depth/camera_info',
                                        f'{arm.name}_depth_camera/camera_info'),
                                    ('depth/image_rect',
                                        f'{arm.name}_depth_camera/image_raw'),
                                    ('depth_registered/camera_info',
                                        '~/camera_info'),
                                    ('depth_registered/image_rect',
                                        '~/image_rect'),
                                    ('depth_registered/image_rect/compressed',
                                        '~/image_rect/compressed'),
                                    ('depth_registered/image_rect/compressedDepth',
                                        '~/image_rect/compressedDepth'),
                                    ('depth_registered/image_rect/ffmpeg',
                                        '~/image_rect/ffmpeg'),
                                    ('depth_registered/image_rect/theora',
                                        '~/image_rect/theora'),
                                    ('depth_registered/image_rect/zstd',
                                        '~/image_rect/zstd'),
                                ]
                            ),
                            LaunchFile.ComposableNode(
                                name=f'{arm.name}_pointcloud_node',
                                package='depth_image_proc',
                                plugin='depth_image_proc::PointCloudXyzrgbNode',
                                namespace=f'{self.namespace}/manipulators',
                                remappings=[
                                    ('depth_registered/camera_info',
                                        f'{arm.name}_depth_upsampled/camera_info'),
                                    ('depth_registered/image_rect',
                                        f'{arm.name}_depth_upsampled/image_rect'),
                                    ('depth_registered/image_rect/compressed',
                                        f'{arm.name}_depth_upsampled/image_rect/compressed'),
                                    ('depth_registered/image_rect/compressedDepth',
                                        f'{arm.name}_depth_upsampled/image_rect/compressedDepth'),
                                    ('depth_registered/image_rect/ffmpeg',
                                        f'{arm.name}_depth_upsampled/image_rect/ffmpeg'),
                                    ('depth_registered/image_rect/theora',
                                        f'{arm.name}_depth_upsampled/image_rect/theora'),
                                    ('depth_registered/image_rect/zstd',
                                        f'{arm.name}_depth_upsampled/image_rect/zstd'),
                                    ('rgb/camera_info',
                                        f'{arm.name}_color_camera/camera_info'),
                                    ('depth/camera_info',
                                        f'{arm.name}_depth_camera/camera_info'),
                                    ('rgb/image_rect_color',
                                        f'{arm.name}_color_camera/image_raw'),
                                    ('depth/image_rect',
                                        f'{arm.name}_depth_camera/image_raw'),
                                    ('points',
                                        f'{arm.name}_depth_camera/color/points'),
                                ]
                            )
                        ]
                    )

                    manipulator_service_launch_writer.add_node(depth_node)
                    manipulator_service_launch_writer.add_node(color_node)
                    manipulator_service_launch_writer.add(pointcloud_node)

        if self.clearpath_config.manipulators.get_all_manipulators():
            manipulator_service_launch_writer.add(self.manipulators_launch_file)
        manipulator_service_launch_writer.generate_file()
