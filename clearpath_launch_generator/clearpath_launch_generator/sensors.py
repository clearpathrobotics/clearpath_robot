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

from clearpath_config.sensors.base import BaseSensor
from clearpath_config.sensors.lidars_2d import HokuyoUST10, SickLMS1XX
from clearpath_config.sensors.lidars_3d import VelodyneLidar
from clearpath_config.sensors.cameras import IntelRealsense
from clearpath_config.sensors.imu import Microstrain
from clearpath_config.sensors.gps import SwiftNavDuro
from clearpath_config.parser import ClearpathConfigParser

from clearpath_launch_generator.launch_writer import LaunchFile, Package, ParameterFile


import os
import yaml


class SensorLaunch():
    class BaseLaunch():
        CLEARPATH_SENSORS = 'clearpath_sensors'

        TOPIC_NAMESPACE = 'platform/sensors/'

        # Launch arguments
        PARAMETERS = 'parameters'
        NAMESPACE = 'namespace'

        def __init__(self, sensor: BaseSensor, output_path: str = '/etc/clearpath/sensors/') -> None:
            self.sensor = sensor

            # Defaults
            self.default_sensor_package = Package(self.CLEARPATH_SENSORS)
            self.default_sensor_parameters_file = ParameterFile(
                self.get_model(),
                package=self.default_sensor_package)
            self.default_sensor_parameters = ClearpathConfigParser.read_yaml(
                self.default_sensor_parameters_file.get_full_path())

            # Generated
            self.sensor_launch_file = LaunchFile(
                self.get_name(),
                path=os.path.join(output_path, 'launch'))
            self.sensor_parameters_file = ParameterFile(self.get_name(), path=os.path.join(output_path, 'config'))
            self.sensor_parameters = sensor.get_ros_parameters()

            self.launch_args = {
                self.PARAMETERS: self.sensor_parameters_file,
                self.NAMESPACE: self.get_namespace()
            }

            # Set launch args for default launch file
            self.default_sensor_launch_file = LaunchFile(
                self.get_model(),
                package=self.default_sensor_package,
                args=self.launch_args)

            self.generate_config()

        @staticmethod
        def copy_parameters(default_params: dict, params: dict) -> dict:
            modified_params = default_params
            for p in params:
                if p in default_params:
                    modified_params[p] = params[p]
            return modified_params

        def generate_config(self):
            namespace = self.get_namespace()
            node_key = list(self.default_sensor_parameters.keys())[0]
            default_parameters = self.default_sensor_parameters[node_key]['ros__parameters']

            parameters = {
              namespace: {
                node_key: {
                  'ros__parameters': self.copy_parameters(default_parameters, self.sensor_parameters)
                }
              }
            }

            with open(self.sensor_parameters_file.get_full_path(), 'w+') as f:
                yaml.dump(parameters, f, yaml.SafeDumper)

            print('Generated config: {0}'.format(self.sensor_parameters_file.get_full_path()))

        def get_launch_file(self) -> LaunchFile:
            return self.sensor_launch_file

        def get_parameters(self) -> dict:
            return self.sensor_parameters

        def get_parameter(self, parameter: str) -> str:
            return self.sensor_parameters[parameter]

        def get_parameter_file(self) -> ParameterFile:
            return self.sensor_parameters_file

        def get_name(self) -> str:
            return self.sensor.get_name()

        def get_model(self) -> str:
            return self.sensor.SENSOR_MODEL

        def get_default_sensor_package(self) -> str:
            return self.default_sensor_package

        def get_launch_args(self) -> dict:
            return self.launch_args

        def get_namespace(self) -> str:
            return self.TOPIC_NAMESPACE + self.sensor.get_name()

    class VelodyneLidarLaunch(BaseLaunch):
        def __init__(self, sensor: VelodyneLidar, output_path: str = '/etc/clearpath/sensors/') -> None:
            super().__init__(sensor, output_path)
            self.generate_config()

        def generate_config(self):
            namespace = self.get_namespace()
            driver_key = 'velodyne_driver_node'
            pointcloud_key = 'velodyne_convert_node'
            laserscan_key = 'velodyne_laserscan_node'
            default_driver_parameters = self.default_sensor_parameters[driver_key]['ros__parameters']
            default_pointcloud_parameters = self.default_sensor_parameters[pointcloud_key]['ros__parameters']
            default_laserscan_parameters = self.default_sensor_parameters[laserscan_key]['ros__parameters']

            parameters = {
                namespace: {
                  driver_key: {
                    'ros__parameters': self.copy_parameters(default_driver_parameters, self.sensor_parameters)
                  },
                  pointcloud_key: {
                    'ros__parameters': self.copy_parameters(default_pointcloud_parameters, self.sensor_parameters)
                  },
                  laserscan_key: {
                    'ros__parameters': self.copy_parameters(default_laserscan_parameters, self.sensor_parameters)
                  },
                }
            }

            with open(self.sensor_parameters_file.get_full_path(), 'w+') as f:
                yaml.dump(parameters, f, yaml.SafeDumper)

            print('Generated config: {0}'.format(self.sensor_parameters_file.get_full_path()))

    MODEL = {
        HokuyoUST10.SENSOR_MODEL: BaseLaunch,
        SickLMS1XX.SENSOR_MODEL: BaseLaunch,
        IntelRealsense.SENSOR_MODEL: BaseLaunch,
        Microstrain.SENSOR_MODEL: BaseLaunch,
        VelodyneLidar.SENSOR_MODEL: VelodyneLidarLaunch,
        SwiftNavDuro.SENSOR_MODEL: BaseLaunch
    }

    def __new__(cls, sensor: BaseSensor, output_path: str = '/etc/clearpath/sensors/') -> BaseLaunch:
        return SensorLaunch.MODEL[sensor.SENSOR_MODEL](sensor, output_path)
