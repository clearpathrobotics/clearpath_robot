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
from clearpath_config.sensors.lidars_2d import BaseLidar2D, HokuyoUST10, SickLMS1XX
from clearpath_config.sensors.lidars_3d import BaseLidar3D, VelodyneLidar
from clearpath_config.sensors.cameras import BaseCamera, IntelRealsense
from clearpath_config.sensors.imu import BaseIMU, Microstrain
from clearpath_config.sensors.gps import BaseGPS, SwiftNavDuro

from clearpath_generator_common.common import LaunchFile, Package, ParamFile


class SensorLaunch():
    class BaseLaunch():
        CLEARPATH_SENSORS = 'clearpath_sensors'
        TOPIC_NAMESPACE = '/platform/sensors/'

        # Launch arguments
        PARAMETERS = 'parameters'
        NAMESPACE = 'namespace'

        def __init__(self, sensor: BaseSensor,
                     namespace: str,
                     launch_path: str,
                     param_path: str) -> None:
            self.sensor = sensor
            self.namespace = namespace
            self.parameters = ParamFile(self.get_name(), path=param_path)
            # Defaults
            self.default_sensor_package = Package(self.CLEARPATH_SENSORS)

            # Generated
            self.sensor_launch_file = LaunchFile(
                self.get_name(),
                path=launch_path)

            self.launch_args = [
                (self.PARAMETERS, self.parameters.get_full_path()),
                (self.NAMESPACE, self.get_namespace())
            ]

            # Set launch args for default launch file
            self.default_sensor_launch_file = LaunchFile(
                self.get_model(),
                package=self.default_sensor_package,
                args=self.launch_args)

        def get_launch_file(self) -> LaunchFile:
            return self.sensor_launch_file

        def get_default_launch_file(self) -> LaunchFile:
            return self.default_sensor_launch_file

        def get_name(self) -> str:
            return self.sensor.get_name()

        def get_model(self) -> str:
            return self.sensor.SENSOR_MODEL

        def get_default_sensor_package(self) -> str:
            return self.default_sensor_package

        def get_launch_args(self) -> list:
            return self.launch_args

        def get_namespace(self) -> str:
            return self.namespace + self.TOPIC_NAMESPACE + self.sensor.get_name()

    MODEL = {
        HokuyoUST10.SENSOR_MODEL: BaseLaunch,
        SickLMS1XX.SENSOR_MODEL: BaseLaunch,
        IntelRealsense.SENSOR_MODEL: BaseLaunch,
        Microstrain.SENSOR_MODEL: BaseLaunch,
        VelodyneLidar.SENSOR_MODEL: BaseLaunch,
        SwiftNavDuro.SENSOR_MODEL: BaseLaunch
    }

    def __new__(cls, sensor: BaseSensor, namespace: str, launch_path: str, param_path: str) -> BaseLaunch:
        return SensorLaunch.MODEL[sensor.SENSOR_MODEL](sensor, namespace, launch_path, param_path)
