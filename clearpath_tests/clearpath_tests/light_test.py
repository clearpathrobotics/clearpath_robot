#!/usr/bin/env python3
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
import threading

from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import BaseGenerator
from clearpath_platform_msgs.msg import Lights, RGB
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

import rclpy
from rclpy.qos import qos_profile_system_default


class LightTestNode(ClearpathTestNode):

    def __init__(self, light_zones=4, setup_path='/etc/clearpath'):
        super().__init__('Lights', 'light_test', setup_path)

        self.light_zones = self.get_parameter_or('n_lights', light_zones)

        if self.platform == Platform.A300:
            self.front_left = [Lights.A300_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.A300_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.A300_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.A300_LIGHTS_REAR_RIGHT]

            self.top_row = [
                Lights.A300_LIGHTS_FRONT_LEFT,
                Lights.A300_LIGHTS_FRONT_RIGHT,
                Lights.A300_LIGHTS_REAR_LEFT,
                Lights.A300_LIGHTS_REAR_RIGHT,
            ]
            self.bottom_row = []
        elif self.platform == Platform.W200:
            self.front_left = [Lights.W200_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.W200_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.W200_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.W200_LIGHTS_REAR_RIGHT]

            self.top_row = [
                Lights.W200_LIGHTS_FRONT_LEFT,
                Lights.W200_LIGHTS_FRONT_RIGHT,
                Lights.W200_LIGHTS_REAR_LEFT,
                Lights.W200_LIGHTS_REAR_RIGHT,
            ]
            self.bottom_row = []
        elif self.platform == Platform.R100:
            self.front_left = [
                Lights.R100_LIGHTS_FRONT_PORT_UPPER,
                Lights.R100_LIGHTS_FRONT_PORT_LOWER
            ]
            self.front_right = [
                Lights.R100_LIGHTS_FRONT_STARBOARD_UPPER,
                Lights.R100_LIGHTS_FRONT_STARBOARD_LOWER
            ]
            self.back_left = [
                Lights.R100_LIGHTS_REAR_PORT_UPPER,
                Lights.R100_LIGHTS_REAR_PORT_LOWER
            ]
            self.back_right = [
                Lights.R100_LIGHTS_REAR_STARBOARD_UPPER,
                Lights.R100_LIGHTS_REAR_STARBOARD_LOWER
            ]

            self.top_row = [
                Lights.R100_LIGHTS_FRONT_PORT_UPPER,
                Lights.R100_LIGHTS_FRONT_STARBOARD_UPPER,
                Lights.R100_LIGHTS_REAR_PORT_UPPER,
                Lights.R100_LIGHTS_REAR_STARBOARD_UPPER,
            ]
            self.bottom_row = [
                Lights.R100_LIGHTS_FRONT_PORT_LOWER,
                Lights.R100_LIGHTS_FRONT_STARBOARD_LOWER,
                Lights.R100_LIGHTS_REAR_PORT_LOWER,
                Lights.R100_LIGHTS_REAR_STARBOARD_LOWER,
            ]
        elif (
            self.platform == Platform.DD100 or
            self.platform == Platform.DO100
        ):
            self.front_left = [Lights.D100_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.D100_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.D100_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.D100_LIGHTS_REAR_RIGHT]

            self.top_row = [
                Lights.D100_LIGHTS_FRONT_LEFT,
                Lights.D100_LIGHTS_FRONT_RIGHT,
                Lights.D100_LIGHTS_REAR_LEFT,
                Lights.D100_LIGHTS_REAR_RIGHT,
            ]
            self.bottom_row = []
        elif (
            self.platform == Platform.DD150 or
            self.platform == Platform.DO150
        ):
            self.front_left = [Lights.D150_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.D150_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.D150_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.D150_LIGHTS_REAR_RIGHT]

            self.top_row = [
                Lights.D150_LIGHTS_FRONT_LEFT,
                Lights.D150_LIGHTS_FRONT_RIGHT,
                Lights.D150_LIGHTS_REAR_LEFT,
                Lights.D150_LIGHTS_REAR_RIGHT,
            ]
            self.bottom_row = []
        else:
            self.get_logger().warning(
                f'Unsupported platform {self.platform}: unable to determine lighting indices'
            )
            self.front_left = []
            self.front_right = []
            self.back_left = []
            self.back_right = []

            self.top_row = []
            self.bottom_row = []

        # Params
        self.lights_topic = self.get_parameter_or('lights_topic', f'/{self.namespace}/platform/cmd_lights')  # noqa: E501
        self.publish_rate = self.get_parameter_or('publish_rate', 10)

        # Publisher and message
        self.msg = Lights()
        for i in range(self.light_zones):
            self.msg.lights.append(RGB())
        self.publisher = self.create_publisher(Lights, self.lights_topic, qos_profile_system_default)  # noqa: E501

        self.test_in_progress = False
        self.colour = 0

    def publish_callback(self):
        # cycle through blue/green/red/white/off forever
        if not self.test_in_progress:
            if self.colour == 0:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(10)
                    self.msg.lights[i].green = int(0)
                self.colour = 1
            elif self.colour == 1:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(10)
                self.colour = 2
            elif self.colour == 2:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(10)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(0)
                self.colour = 3
            elif self.colour == 3:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(5)
                    self.msg.lights[i].blue = int(5)
                    self.msg.lights[i].green = int(5)
                self.colour = 4
            elif self.colour == 4:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(0)
                self.colour = 0

        # Publish the message
        self.publisher.publish(self.msg)

    def start(self):
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def run_test(self):
        # kick out if the lights are in an uncontrolled state
        user_input = self.promptYN("Are all e-stops cleared, the robot's battery charged, and the front lights white & rear lights red?")  # noqa: E501
        if user_input == 'N':
            self.results = [ClearpathTestResult(
                None,
                self.test_name,
                'Robot in error state; cannot control lights',
            )]
            return self.results

        self.results = []
        self.test_in_progress = True
        self.start()

        self.test_done = False
        ui_thread = threading.Thread(target=self.run_ui)
        ui_thread.start()
        while not self.test_done:
            rclpy.spin_once(self)
        ui_thread.join()
        return self.results

    def run_ui(self):
        results = self.results

        # turn off all lights
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 0
        user_input = self.promptYN('Are all lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'All lights off', notes))
        else:
            results.append(ClearpathTestResult(True, 'All lights off', None))

        # turn all lights red
        for i in range(self.light_zones):
            self.msg.lights[i].red = 128
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 0
        user_input = self.promptYN('Are all lights red?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'All lights red', notes))
        else:
            results.append(ClearpathTestResult(True, 'All lights red', None))

        # turn all lights green
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 128
            self.msg.lights[i].blue = 0
        user_input = self.promptYN('Are all lights green?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'All lights green', notes))
        else:
            results.append(ClearpathTestResult(True, 'All lights green', None))

        # turn all lights blue
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 128
        user_input = self.promptYN('Are all lights blue?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'All lights blue', notes))
        else:
            results.append(ClearpathTestResult(True, 'All lights blue', None))

        # turn all lights white
        for i in range(self.light_zones):
            self.msg.lights[i].red = 128
            self.msg.lights[i].green = 128
            self.msg.lights[i].blue = 128
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights white?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'All lights white', notes))
        else:
            results.append(ClearpathTestResult(True, 'All lights white', None))

        # test each corner
        for i in range(self.light_zones):
            if i in self.front_left:
                self.msg.lights[i].red = 128
                self.msg.lights[i].green = 128
                self.msg.lights[i].blue = 128
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        user_input = self.promptYN('Is the front-left light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'Front left white', notes))
        else:
            results.append(ClearpathTestResult(True, 'Front left white', None))

        for i in range(self.light_zones):
            if i in self.front_right:
                self.msg.lights[i].red = 128
                self.msg.lights[i].green = 128
                self.msg.lights[i].blue = 128
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        user_input = self.promptYN('Is the front-right light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'Front right white', notes))
        else:
            results.append(ClearpathTestResult(True, 'Front right white', None))

        for i in range(self.light_zones):
            if i in self.back_left:
                self.msg.lights[i].red = 128
                self.msg.lights[i].green = 128
                self.msg.lights[i].blue = 128
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        user_input = self.promptYN('Is the back-left light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'Back left white', notes))
        else:
            results.append(ClearpathTestResult(True, 'Back left white', None))

        for i in range(self.light_zones):
            if i in self.back_right:
                self.msg.lights[i].red = 128
                self.msg.lights[i].green = 128
                self.msg.lights[i].blue = 128
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        user_input = self.promptYN('Is the back-right light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(ClearpathTestResult(False, 'Back right white', notes))
        else:
            results.append(ClearpathTestResult(True, 'Back right white', None))

        # This test only applies to R100, since it's the only platform (currently)
        # with two rows of independently-controlled lights
        if len(self.bottom_row) > 0 and len(self.top_row) > 0:
            # test each row of lights
            for i in range(self.light_zones):
                if i in self.top_row:
                    self.msg.lights[i].red = 128
                    self.msg.lights[i].green = 128
                    self.msg.lights[i].blue = 128
                else:
                    self.msg.lights[i].red = 0
                    self.msg.lights[i].green = 0
                    self.msg.lights[i].blue = 0
            user_input = self.promptYN('Is the top row of lights white and bottom row off?')
            if user_input == 'N':
                notes = input('Briefly describe the problem > ')
                results.append(ClearpathTestResult(False, 'Top row white', notes))
            else:
                results.append(ClearpathTestResult(True, 'Top row white', None))

            for i in range(self.light_zones):
                if i in self.bottom_row:
                    self.msg.lights[i].red = 128
                    self.msg.lights[i].green = 128
                    self.msg.lights[i].blue = 128
                else:
                    self.msg.lights[i].red = 0
                    self.msg.lights[i].green = 0
                    self.msg.lights[i].blue = 0
            user_input = self.promptYN('Is the bottom row of lights white and top row off?')
            if user_input == 'N':
                notes = input('Briefly describe the problem > ')
                results.append(ClearpathTestResult(False, 'Bottom row white', notes))
            else:
                results.append(ClearpathTestResult(True, 'Bottom row white', None))

        self.test_done = True


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    lt = LightTestNode(setup_path=setup_path)

    try:
        lt.start()
        rclpy.spin(lt)
    except KeyboardInterrupt:
        pass

    lt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
