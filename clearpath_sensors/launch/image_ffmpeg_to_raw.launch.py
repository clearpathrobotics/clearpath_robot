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


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    in_ffmpeg = LaunchConfiguration('in_ffmpeg')
    out_raw = LaunchConfiguration('out_raw')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    arg_in_ffmpeg = DeclareLaunchArgument(
        'in_ffmpeg',
        default_value='ffmpeg'
    )

    arg_out_raw = DeclareLaunchArgument(
        'out_raw',
        default_value='image'
    )

    ffmpeg_transport_node = Node(
        name='image_ffmpeg_to_raw',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in/ffmpeg', in_ffmpeg),
            ('out', out_raw),
        ],
        arguments=['ffmpeg', 'raw'],
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_in_ffmpeg)
    ld.add_action(arg_out_raw)
    ld.add_action(ffmpeg_transport_node)
    return ld
