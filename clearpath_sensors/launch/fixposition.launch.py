# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
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

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    robot_namespace = LaunchConfiguration('robot_namespace')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('clearpath_sensors'),
          'config',
          'fixposition.yaml'
        ]))

    fixposition_node = Node(
        package='fixposition_driver_ros2',
        executable='fixposition_driver_ros2_exec',
        name='fixposition_driver',
        namespace=namespace,
        output='screen',
        parameters=[parameters],
        remappings=[
            # General platform topics
            # these go in the robot's root namespace
            ('/diagnostics', PathJoinSubstitution(['/', robot_namespace, 'diagnostics'])),
            ('/tf', PathJoinSubstitution(['/', robot_namespace, 'tf'])),
            ('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])),

            # Core API sensor topics
            # they get their own numbered sensor API namespace where appropriate
            ('/fixposition/gnss1', 'gps_0/fix'),
            ('/fixposition/gnss2', 'gps_1/fix'),
            ('/fixposition/fpa/corrimu', 'imu_0/data'),
            ('/fixposition/odometry_enu', 'odom'),
            ('/odometry', PathJoinSubstitution(['/', robot_namespace, 'platform/odometry'])),

            # Fixposition-specific topics
            # not part of the core INS API, so keep them contained in the 'fixposition' namespace
            ('/fixposition/fpa/eoe', 'fixposition/fpa/eoe'),
            ('/fixposition/fpa/gnssant', 'fixposition/fpa/gnssant'),
            ('/fixposition/fpa/gnsscorr', 'fixposition/fpa/gnsscorr'),
            ('/fixposition/fpa/imubias', 'fixposition/fpa/imubias'),
            ('/fixposition/fpa/llh', 'fixposition/fpa/llh'),
            ('/fixposition/fpa/odomenu', 'fixposition/fpa/odomenu'),
            ('/fixposition/fpa/odometry', 'fixposition/fpa/odometry'),
            ('/fixposition/fpa/odomsh', 'fixposition/fpa/odomsh'),
            ('/fixposition/fpa/odomstatus', 'fixposition/fpa/odomstatus'),
            ('/fixposition/fpa/rawimu', 'fixposition/fpa/rawimu'),
            ('/fixposition/fpa/text', 'fixposition/fpa/text'),
            ('/fixposition/fpa/tp', 'fixposition/fpa/tp'),
            ('/fixposition/fusion', 'fixposition/fusion'),
            ('/fixposition/imu_ypr', 'fixposition/imu_ypr'),
            ('/fixposition/nmea', 'fixposition/nmea'),
            ('/fixposition/nmea/gga', 'fixposition/nmea/gga'),
            ('/fixposition/nmea/gll', 'fixposition/nmea/gll'),
            ('/fixposition/nmea/gsa', 'fixposition/nmea/gsa'),
            ('/fixposition/nmea/gst', 'fixposition/nmea/gst'),
            ('/fixposition/nmea/gsv', 'fixposition/nmea/gsv'),
            ('/fixposition/nmea/hdt', 'fixposition/nmea/hdt'),
            ('/fixposition/nmea/rmc', 'fixposition/nmea/rmc'),
            ('/fixposition/nmea/vtg', 'fixposition/nmea/vtg'),
            ('/fixposition/nmea/zda', 'fixposition/nmea/zda'),
            ('/fixposition/novb/inspvax', 'fixposition/novb/inspvax'),
            ('/fixposition/odometry_ecef', 'fixposition/odometry_ecef'),
            ('/fixposition/odometry_llh', 'fixposition/odometry_llh'),
            ('/fixposition/odometry_enu_smooth', 'fixposition/odometry_enu_smooth'),
            ('/fixposition/odometry_smooth', 'fixposition/odometry_smooth'),
            ('/fixposition/poiimu', 'fixposition/poiimu'),
            ('/fixposition/rawimu', 'fixposition/rawimu'),
            ('/fixposition/speed', 'fixposition/speed'),
            ('/fixposition/ypr', 'fixposition/ypr'),
            ('/rtcm', 'rtcm'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_namespace)
    ld.add_action(arg_parameters)
    ld.add_action(fixposition_node)
    return ld
