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

from clearpath_generator_common.common import LaunchFile, Package
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_robot.launch import manipulators  # noqa: F401
from clearpath_generator_robot.launch import platforms  # noqa: F401
from clearpath_generator_robot.launch.manipulator import ManipulatorLaunch
from clearpath_generator_robot.launch.platform import PlatformLaunch
from clearpath_generator_robot.launch.sensors import SensorLaunch


class RobotLaunchGenerator(LaunchGenerator):
    """
    Concrete launch generator for physical Clearpath robots.

    Emits the three top-level service launch files consumed by
    `clearpath_robot` at runtime:

    * ``sensors-service.launch.py``      - per-sensor launch includes
    * ``platform-service.launch.py``     - MCU, common and platform-specific
      components, composed via the :class:`PlatformLaunch` registry
    * ``manipulators-service.launch.py`` - arm/gripper drivers and vision
      pipelines
    """

    def generate_sensors(self) -> None:
        """Generate the per-sensor launch files and the top-level sensors service launch."""
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
        """
        Generate the platform service and platform-extras service launch files.

        The concrete per-platform component set is resolved through the
        :class:`PlatformLaunch` registry using ``self.platform_model``; any
        user-declared ``platform.extras.launch`` entries are appended to a
        separate extras launch file.
        """
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
        """
        Generate the manipulators service launch file.

        Iterates the configured arms and dispatches each to the matching
        :class:`ManipulatorLaunch` subclass for any vendor-specific helper
        nodes (UR tool communication, Franka gripper controller, Kinova
        vision/pointcloud pipeline). When at least one manipulator is
        configured the shared `manipulators.launch.py` is included with a
        common `control_delay` of `1.0` to give vendor drivers time to come
        up before the controllers start.
        """
        manipulator_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        arms = self.clearpath_config.manipulators.get_all_arms()
        for arm in arms:
            try:
                manipulator_launch_cls = ManipulatorLaunch.get(arm.MANIPULATOR_MODEL)
            except KeyError:
                print(f'No manipulator launch found for model "{arm.MANIPULATOR_MODEL}"; skipping manipulator "{arm.name}"')  # noqa:E501
                continue
            manipulator_launch = manipulator_launch_cls(arm, self.namespace)
            for component in manipulator_launch.get_components():
                manipulator_service_launch_writer.add(component)

        if self.clearpath_config.manipulators.get_all_manipulators():
            control_delay = ('control_delay', '1.0')
            if control_delay not in self.manipulators_launch_file.args:
                self.manipulators_launch_file.args.append(control_delay)
            manipulator_service_launch_writer.add(self.manipulators_launch_file)
        manipulator_service_launch_writer.generate_file()
