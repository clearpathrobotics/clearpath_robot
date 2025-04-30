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
from clearpath_config.common.types.platform import Platform
from clearpath_motor_msgs.msg import (
    LynxMultiFeedback,
    LynxSystemProtection,
    PumaMultiFeedback,
)
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class MobilityTestNode(ClearpathTestNode):
    """Generic class for all tests that involve driving the robot with odometry feedback."""

    def __init__(self, test_name, node_name, setup_path='/etc/clearpath'):
        super().__init__(test_name, node_name, setup_path)

        # The velocity command to output
        self.cmd_vel = TwistStamped()

        # The latest odometry received
        self.latest_odom = None

        # has the current test started?
        self.test_in_progress = False

        # is the current test finished?
        self.ne = False

        # the results of the test, possibly collected from multiple sources
        self.test_results = []

        # have we received an error with the test?
        # this gets set if the test needs to be aborted because of an external condition
        # e.g. a lynx safety warning
        self.test_error = False

        # message explaining the error above
        self.test_error_msg = ''

        self.odom_topic = self.get_parameter_or('odom_topic', 'platform/odom/filtered')
        self.publish_rate = self.get_parameter_or('publish_rate', 30)
        self.drive_topic = self.get_parameter_or('drive_topic', 'cmd_vel')
        self.base_link = self.get_parameter_or('base_link', 'base_link')

        if not self.odom_topic.startswith('/'):
            self.odom_topic = f'/{self.namespace}/{self.odom_topic}'

        if not self.drive_topic.startswith('/'):
            self.drive_topic = f'/{self.namespace}/{self.drive_topic}'

    def start(self):
        self.motor_currents = []
        if self.platform == Platform.A300:
            self.motor_fb_sub = self.create_subscription(
                LynxMultiFeedback,
                f'/{self.namespace}/platform/motors/feedback',
                self.on_lynx_status,
                qos_profile_sensor_data,
            )
        else:
            self.motor_fb_sub = self.create_subscription(
                PumaMultiFeedback,
                f'/{self.namespace}/platform/motors/feedback',
                self.on_puma_status,
                qos_profile_sensor_data,
            )

        self.start_time = self.get_clock().now()
        self.odom_timeout = Duration(seconds=10)
        self.initial_position = None
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'Subscribing to odometry on {self.odom_topic}...')

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            self.drive_topic,
            qos_profile_system_default,
        )
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

        if self.clearpath_config.get_platform_model() == Platform.A300:
            self.lynx_feedback_sub = self.create_subscription(
                LynxSystemProtection,
                f'/{self.namespace}/platform/motors/system_protection',
                self.lynx_callback,
                qos_profile_sensor_data,
            )

    def on_puma_status(self, puma_status: PumaMultiFeedback):
        if not self.test_in_progress or self.test_error or self.test_done:
            return

        currents = []
        for driver in puma_status.drivers_feedback:
            currents.append(driver.current)
        self.motor_currents.append(currents)

    def on_lynx_status(self, lynx_status: LynxMultiFeedback):
        if not self.test_in_progress or self.test_error or self.test_done:
            return

        currents = []
        for driver in lynx_status.drivers:
            currents.append(driver.current)
        self.motor_currents.append(currents)

    def calculate_average_motor_currents(self):
        if len(self.motor_currents) == 0:
            return []

        average_currents = []
        for i in range(len(self.motor_currents[0])):
            average_currents.append(0.0)
            for sample in self.motor_currents:
                average_currents[i] += sample[i]
            average_currents[i] /= len(self.motor_currents)

        return average_currents

    def get_test_result_details(self):
        details = ''
        details += '\n#### Average motor current draw during test\n\n'
        avg = self.calculate_average_motor_currents()
        for amps in avg:
            details += f'* {amps:0.3f}A\n'
        return details

    def odom_callback(self, odom_msg):
        if self.latest_odom is None:
            self.get_logger().info('Received initial odometry pose')
        self.latest_odom = odom_msg

    def publish_callback(self):
        self.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def lynx_callback(self, lynx_status: LynxSystemProtection):
        if not self.test_in_progress or self.test_done or self.test_error:
            # kick out if we're not running the tests
            return

        for i in range(len(lynx_status.motor_states)):
            state = lynx_status.motor_states[i]

            position = 'Unknown'
            if i == LynxSystemProtection.A300_MOTOR_FRONT_LEFT:
                position = 'Front-Left'
            elif i == LynxSystemProtection.A300_MOTOR_FRONT_RIGHT:
                position = 'Front-Right'
            elif i == LynxSystemProtection.A300_MOTOR_REAR_LEFT:
                position = 'Rear-Left'
            elif i == LynxSystemProtection.A300_MOTOR_REAR_RIGHT:
                position = 'Rear-Right'

            if state == LynxSystemProtection.THROTTLED:
                self.test_error = True
                self.test_error_msg = f'{position} motor is throttled'
                self.test_results.append(ClearpathTestResult(
                    False,
                    self.test_name,
                    self.test_error_msg,
                ))
            elif state == LynxSystemProtection.OVERHEATED:
                self.test_error = True
                self.test_error_msg = f'{position} motor overheated'
                self.test_results.append(ClearpathTestResult(
                    False,
                    self.test_name,
                    self.test_error_msg,
                ))
            elif state == LynxSystemProtection.ERROR:
                self.test_error = True
                self.test_error_msg = f'{position} motor error'
                self.test_results.append(ClearpathTestResult(
                    False,
                    self.test_name,
                    self.test_error_msg,
                ))
