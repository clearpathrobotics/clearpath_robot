# Software License Agreement (BSD)
#
# @author    Hilary Luo <hluo@clearpathrobotics.com>
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('in_raw', default_value='image'),
    DeclareLaunchArgument('out_ffmpeg', default_value='ffmpeg'),
    DeclareLaunchArgument('encoding', default_value='libx264'),
    DeclareLaunchArgument('qmax', default_value='40'),
    DeclareLaunchArgument('preset', default_value='superfast'),
    DeclareLaunchArgument('tune', default_value='zerolatency'),
    DeclareLaunchArgument('bit_rate', default_value='1000000'),
    DeclareLaunchArgument('gop_size', default_value='15'),
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    in_raw = LaunchConfiguration('in_raw')
    out_ffmpeg = LaunchConfiguration('out_ffmpeg')

    ffmpeg_transport_node = Node(
        name='image_raw_to_ffmpeg',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in', in_raw),
            ('out/ffmpeg', out_ffmpeg),
        ],
        arguments=['raw', 'ffmpeg'],
        parameters=[
            {'ffmpeg_image_transport.encoding': LaunchConfiguration('encoding')},
            {'ffmpeg_image_transport.qmax': LaunchConfiguration('qmax')},
            {'ffmpeg_image_transport.preset': LaunchConfiguration('preset')},
            {'ffmpeg_image_transport.tune': LaunchConfiguration('tune')},
            {'ffmpeg_image_transport.bit_rate': LaunchConfiguration('bit_rate')},
            {'ffmpeg_image_transport.gop_size': LaunchConfiguration('gop_size')}],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ffmpeg_transport_node)
    return ld
