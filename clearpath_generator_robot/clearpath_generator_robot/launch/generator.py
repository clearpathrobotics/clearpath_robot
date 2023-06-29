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

from clearpath_generator_common.common import LaunchFile, Package
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_robot.launch.sensors import SensorLaunch

from clearpath_config.platform.platform import Platform

import os


class RobotLaunchGenerator(LaunchGenerator):
    def generate_sensors(self) -> None:
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        for sensor in sensors:
            if sensor.get_launch_enabled():
                sensor_launch = SensorLaunch(
                        sensor,
                        self.namespace,
                        self.sensors_launch_path,
                        self.sensors_params_path)
                sensor_writer = LaunchWriter(sensor_launch.get_launch_file())
                # Add default sensor launch file
                sensor_writer.add_launch_file(sensor_launch.get_default_launch_file())
                # Generate sensor launch file
                sensor_writer.generate_file()
                # Add sensor to top level sensors launch file
                sensors_service_launch_writer.add_launch_file(sensor_launch.get_launch_file())

        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add_launch_file(self.platform_launch_file)

        if self.platform_model == Platform.J100:
            # Add micro ros agent
            uros_node = LaunchFile.Node(
                name='micro_ros_agent',
                package='micro_ros_agent',
                executable='micro_ros_agent',
                namespace=self.namespace,
                arguments=[
                    'serial', '--dev', '/dev/clearpath/j100'
                ])
            platform_service_launch_writer.add_node(uros_node)

            # Set domain id and namespace on MCU
            configure_mcu = LaunchFile.Process(
                name='configure_mcu',
                cmd=[
                    ['export ROS_DOMAIN_ID=0;'],
                    [LaunchFile.Variable('FindExecutable(name=\'ros2\')'),
                     ' service call platform/mcu/configure',
                     ' clearpath_platform_msgs/srv/ConfigureMcu',
                     ' \"{{domain_id: {0},'.format(self.clearpath_config.system.get_domain_id()),
                     ' robot_namespace: \\\'{0}\\\'}}\"'.format(self.namespace)]
                ])
            platform_service_launch_writer.add_process(configure_mcu)

            # IMU filter
            imu_filter_config = LaunchFile.LaunchArg(
                'imu_filter',
                default_value=os.path.join(self.platform_params_path, 'imu_filter.yaml')
            )
            platform_service_launch_writer.declare_launch_arg(imu_filter_config)

            imu_filter_node = LaunchFile.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_node',
                namespace=self.namespace,
                parameters=[LaunchFile.Variable('imu_filter')],
                remappings=[
                  ('imu/data_raw', 'platform/sensors/imu_0/data_raw'),
                  ('imu/mag', 'platform/sensors/imu_0/magnetic_field'),
                  ('imu/data', 'platform/sensors/imu_0/data'),
                  ('/tf', 'tf'),
                ],
            )
            platform_service_launch_writer.add_node(imu_filter_node)

            # Wireless watcher
            wireless_watcher_node = LaunchFile.Node(
                package='wireless_watcher',
                executable='wireless_watcher',
                name='wireless_watcher',
                namespace=self.namespace,
                parameters=[{
                  'hz': 1,
                  'dev': '',
                  'connected_topic': 'platform/wifi_connected',
                  'connection_topic': 'platform/wifi_status'
                }],
            )

            platform_service_launch_writer.add_node(wireless_watcher_node)


        # Static transform from <namespace>/odom to odom
        # See https://github.com/ros-controls/ros2_controllers/pull/533
        tf_namespaced_odom_publisher = LaunchFile.get_static_tf_node(
            name='namespaced_odom',
            namespace=self.namespace,
            parent_link='odom',
            child_link=self.namespace + '/odom',
            use_sim_time=False
        )

        # Static transform from <namespace>/base_link to base_link

        tf_namespaced_base_link_publisher = LaunchFile.get_static_tf_node(
            name='namespaced_base_link',
            namespace=self.namespace,
            parent_link=self.namespace + '/base_link',
            child_link='base_link',
            use_sim_time=False
        )

        if self.namespace not in ('', '/'):
            platform_service_launch_writer.add_node(tf_namespaced_odom_publisher)
            platform_service_launch_writer.add_node(tf_namespaced_base_link_publisher)

        platform_service_launch_writer.generate_file()
