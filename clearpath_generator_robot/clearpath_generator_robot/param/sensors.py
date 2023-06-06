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

from clearpath_generator_common.common import ParamFile, Package
from clearpath_generator_common.param.writer import ParamWriter


class SensorParam():
    class BaseParam():
        CLEARPATH_SENSORS = 'clearpath_sensors'

        TOPIC_NAMESPACE = '/platform/sensors/'

        def __init__(self,
                     sensor: BaseSensor,
                     namespace: str,
                     param_path: str) -> None:
            self.sensor = sensor
            self.param_path = param_path

            # Clearpath Sensors Package
            self.clearpath_sensors_package = Package(self.CLEARPATH_SENSORS)

            # Default parameter file for the sensor
            self.default_param_file = ParamFile(
                name=self.sensor.get_sensor_model(),
                package=self.clearpath_sensors_package)
            self.default_param_file.read()

            # Parameter file to generate
            self.param_file = ParamFile(
                name=self.sensor.get_name(),
                namespace=namespace + self.TOPIC_NAMESPACE + self.sensor.get_name(),
                path=self.param_path)

            self.update_parameters()

        def update_parameters(self) -> None:
            node: ParamFile.Node
            for node in self.default_param_file.nodes:
                sensor_parameters = self.sensor.get_ros_parameters()
                updated_parameters = node.get_parameters()
                for p in sensor_parameters:
                    if p in updated_parameters:
                        updated_parameters[p] = sensor_parameters[p]
                self.param_file.add_node(node.get_name(), updated_parameters)

        def generate_config(self):
            sensor_writer = ParamWriter(self.param_file)
            sensor_writer.write_file()
            print('Generated config: {0}'.format(self.param_file.get_full_path()))

    MODEL = {
        HokuyoUST10.SENSOR_MODEL: BaseParam,
        SickLMS1XX.SENSOR_MODEL: BaseParam,
        IntelRealsense.SENSOR_MODEL: BaseParam,
        Microstrain.SENSOR_MODEL: BaseParam,
        VelodyneLidar.SENSOR_MODEL: BaseParam,
        SwiftNavDuro.SENSOR_MODEL: BaseParam
    }

    def __new__(cls,
                sensor: BaseSensor,
                namespace: str,
                param_path: str) -> BaseParam:
        return SensorParam.MODEL[sensor.SENSOR_MODEL](sensor, namespace, param_path)
