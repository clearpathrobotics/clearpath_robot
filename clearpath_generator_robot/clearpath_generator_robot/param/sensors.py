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

from clearpath_config.common.utils.dictionary import merge_dict
from clearpath_config.sensors.types.cameras import BaseCamera
from clearpath_config.sensors.types.sensor import BaseSensor

from clearpath_generator_common.common import Package, ParamFile
from clearpath_generator_common.param.writer import ParamWriter


class SensorParam():

    CLEARPATH_SENSORS = 'clearpath_sensors'

    TOPIC_NAMESPACE = 'sensors'

    def __init__(
            self,
            sensor: BaseSensor,
            namespace: str,
            param_path: str
            ) -> None:
        self.sensor = sensor
        self.param_path = param_path
        if namespace in ('', '/'):
            self.namespace = f'{self.TOPIC_NAMESPACE}/{self.sensor.name}'
        else:
            self.namespace = f'{namespace}/{self.TOPIC_NAMESPACE}/{self.sensor.name}'

        # Clearpath Sensors Package
        self.clearpath_sensors_package = Package(self.CLEARPATH_SENSORS)

        # Default parameter file for the sensor
        self.default_param_file = ParamFile(
            name=self.sensor.get_sensor_model(),
            package=self.clearpath_sensors_package,
            parameters={})
        self.default_param_file.read()

        default_parameters = self.default_param_file.parameters

        # Camera republishers
        if self.sensor.get_sensor_type() == BaseCamera.get_sensor_type():
            for republisher in self.sensor._republishers:
                name = 'image_%s' % republisher.TYPE
                rename = 'image_%s_%s' % (republisher.TYPE, republisher.input)
                republisher_file = ParamFile(
                    name=name,
                    package=self.clearpath_sensors_package,
                    parameters={})
                republisher_file.read()
                republisher_file.parameters[rename] = republisher_file.parameters.pop(name)
                default_parameters = merge_dict(default_parameters,
                                                republisher_file.parameters)

        # Parameter file to generate
        self.param_file = ParamFile(
            name=self.sensor.name,
            namespace=self.namespace,
            path=self.param_path,
            parameters=default_parameters)

        self.param_file.update(self.sensor.get_ros_parameters())

        if 'luxonis_oakd' in self.param_file.parameters:
            luxonis_params = self.param_file.parameters['luxonis_oakd']
            self.param_file.parameters[self.sensor.name] = luxonis_params
            self.param_file.parameters.pop('luxonis_oakd')

    def generate_config(self):
        sensor_writer = ParamWriter(self.param_file)
        sensor_writer.write_file()
        print('Generated config: {0}'.format(self.param_file.full_path))
