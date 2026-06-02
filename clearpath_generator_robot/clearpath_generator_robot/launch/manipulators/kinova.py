# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
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
"""Kinova Gen3 (6-DoF, 7-DoF, Lite) arm launch."""
from clearpath_config.manipulators.types.arms import (
    BaseKinova,
    KinovaGen3Dof6,
    KinovaGen3Dof7,
    KinovaGen3Lite,
)
from clearpath_generator_common.common import LaunchFile
from clearpath_generator_robot.launch.manipulator import (
    BaseManipulatorLaunch,
    ManipulatorLaunch,
)


class KinovaManipulatorLaunch(BaseManipulatorLaunch):
    """
    Kinova Gen3 arms (all variants).

    When the arm's URDF parameters enable vision, emits a depth camera node,
    a color camera node, and a composable-node container running depth
    registration and pointcloud generation.
    """

    MANIPULATOR_MODELS = (
        KinovaGen3Dof6.MANIPULATOR_MODEL,
        KinovaGen3Dof7.MANIPULATOR_MODEL,
        KinovaGen3Lite.MANIPULATOR_MODEL,
    )

    def is_vision_enabled(self) -> bool:
        """Return whether the arm's URDF parameters enable the vision pipeline."""
        return bool(self.arm.get_urdf_parameters().get(BaseKinova.VISION, False))

    def get_components(self) -> list:
        """Return Kinova vision pipeline (depth + color + pointcloud container) if enabled."""
        if not self.is_vision_enabled():
            return []
        return [
            self._make_depth_camera(),
            self._make_color_camera(),
            self._make_pointcloud_container(),
        ]

    def _make_depth_camera(self) -> LaunchFile.Node:
        """Kinova depth camera streaming node."""
        parameters = {
            'camera_type': 'depth',
            'camera_name': 'depth',
            'camera_info_url_default':
                'package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini',
            'camera_info_url_user': '',
            'stream_config': 'rtspsrc location=rtsp://'
                + self.arm.ip
                + '/depth latency=30'
                + ' ! '
                + 'rtpgstdepay',
            'frame_id': f'{self.arm.name}_camera_depth_frame',
            'max_pub_rate': 30.0,
        }
        return LaunchFile.Node(
            name=f'{self.arm.name}_depth_camera',
            package='kinova_vision',
            executable='kinova_vision_node',
            namespace=f'{self.namespace}/manipulators',
            parameters=[parameters],
            remappings=[
                ('camera_info', '~/camera_info'),
                ('image_raw', '~/image_raw'),
                ('image_raw/compressed', '~/image_raw/compressed'),
                ('image_raw/compressedDepth', '~/image_raw/compressedDepth'),
                ('image_raw/theora', '~/image_raw/theora'),
                ('image_raw/ffmpeg', '~/image_raw/ffmpeg'),
                ('image_raw/zstd', '~/image_raw/zstd'),
            ],
        )

    def _make_color_camera(self) -> LaunchFile.Node:
        """Kinova color camera streaming node."""
        parameters = {
            'camera_type': 'color',
            'camera_name': 'color',
            'camera_info_url_default':
                'package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini',
            'camera_info_url_user': '',
            'stream_config': 'rtspsrc location=rtsp://'
                + self.arm.ip
                + '/color latency=30'
                + ' ! rtph264depay ! avdec_h264 ! videoconvert',
            'frame_id': f'{self.arm.name}_camera_color_frame',
            'max_pub_rate': 30.0,
        }
        return LaunchFile.Node(
            name=f'{self.arm.name}_color_camera',
            package='kinova_vision',
            executable='kinova_vision_node',
            namespace=f'{self.namespace}/manipulators',
            parameters=[parameters],
            remappings=[
                ('camera_info', '~/camera_info'),
                ('image_raw', '~/image_raw'),
                ('image_raw/compressed', '~/image_raw/compressed'),
                ('image_raw/compressedDepth', '~/image_raw/compressedDepth'),
                ('image_raw/theora', '~/image_raw/theora'),
                ('image_raw/ffmpeg', '~/image_raw/ffmpeg'),
                ('image_raw/zstd', '~/image_raw/zstd'),
            ],
        )

    def _make_pointcloud_container(self) -> LaunchFile.ComposableNodeContainer:
        """Composable-node container: depth registration + xyzrgb pointcloud."""
        arm_name = self.arm.name
        namespace = self.namespace
        return LaunchFile.ComposableNodeContainer(
            name=f'{arm_name}_depth_proc_container',
            namespace=f'{namespace}/manipulators',
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ],
            composable_node_descriptions=[
                LaunchFile.ComposableNode(
                    name=f'{arm_name}_depth_upsampled',
                    package='depth_image_proc',
                    plugin='depth_image_proc::RegisterNode',
                    namespace=f'{namespace}/manipulators',
                    parameters=[{'fill_upsampling_holes': True}],
                    remappings=[
                        ('rgb/camera_info',
                            f'{arm_name}_color_camera/camera_info'),
                        ('depth/camera_info',
                            f'{arm_name}_depth_camera/camera_info'),
                        ('depth/image_rect',
                            f'{arm_name}_depth_camera/image_raw'),
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
                    ],
                ),
                LaunchFile.ComposableNode(
                    name=f'{arm_name}_pointcloud_node',
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    namespace=f'{namespace}/manipulators',
                    remappings=[
                        ('depth_registered/camera_info',
                            f'{arm_name}_depth_upsampled/camera_info'),
                        ('depth_registered/image_rect',
                            f'{arm_name}_depth_upsampled/image_rect'),
                        ('depth_registered/image_rect/compressed',
                            f'{arm_name}_depth_upsampled/image_rect/compressed'),
                        ('depth_registered/image_rect/compressedDepth',
                            f'{arm_name}_depth_upsampled/image_rect/compressedDepth'),
                        ('depth_registered/image_rect/ffmpeg',
                            f'{arm_name}_depth_upsampled/image_rect/ffmpeg'),
                        ('depth_registered/image_rect/theora',
                            f'{arm_name}_depth_upsampled/image_rect/theora'),
                        ('depth_registered/image_rect/zstd',
                            f'{arm_name}_depth_upsampled/image_rect/zstd'),
                        ('rgb/camera_info',
                            f'{arm_name}_color_camera/camera_info'),
                        ('depth/camera_info',
                            f'{arm_name}_depth_camera/camera_info'),
                        ('rgb/image_rect_color',
                            f'{arm_name}_color_camera/image_raw'),
                        ('depth/image_rect',
                            f'{arm_name}_depth_camera/image_raw'),
                        ('points',
                            f'{arm_name}_depth_camera/color/points'),
                    ],
                ),
            ],
        )


ManipulatorLaunch.register(KinovaManipulatorLaunch)
