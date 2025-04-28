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
import math

from clearpath_generator_common.common import BaseGenerator

from clearpath_tests.mobility_test import MobilityTestNode
from clearpath_tests.test_node import ClearpathTestResult

from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.duration import Duration


class DriveTestNode(MobilityTestNode):
    """
    Use odometry to drive a fixed distance forwards and then stop.

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(
        self,
        setup_path='/etc/clearpath',
        direction='Forwards',
        default_speed_x=0.1,  # 10cm/s; nice and safe
        default_speed_y=0.0,
        default_distance=5.0,
    ):
        super().__init__(
            f'Drive Fixed Distance ({direction})',
            f'drive_test_{direction.lower()}',
            setup_path,
        )

        self.goal_distance = self.get_parameter_or('distance', default_distance)
        self.max_speed_x = self.get_parameter_or('max_speed_x', default_speed_x)
        self.max_speed_y = self.get_parameter_or('max_speed_y', default_speed_y)
        self.error_margin = self.get_parameter_or('error_margin', 0.05)  # +/-5%

        self.initial_position = None
        self.current_displacement = None
        self.twist_msg = TwistStamped()
        self.twist_msg.twist.linear.x = self.max_speed_x
        self.twist_msg.twist.linear.y = self.max_speed_y

    def publish_callback(self):
        if self.initial_position is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise TimeoutError('Timed out waiting for odometry')
        else:
            if self.current_displacement >= self.goal_distance:
                self.cmd_vel.twist.linear.x = 0.0
                self.cmd_vel.twist.linear.y = 0.0
            else:
                self.cmd_vel.twist.linear.x = self.max_speed_x
                self.cmd_vel.twist.linear.y = self.max_speed_y
            super().publish_callback()

            if self.current_displacement >= self.goal_distance:
                self.get_logger().info('Reached goal')
                self.test_done = True

    def odom_callback(self, msg: Odometry):
        super().odom_callback(msg)

        if self.initial_position is None:
            self.initial_position = Point()
            self.initial_position.x = msg.pose.pose.position.x
            self.initial_position.y = msg.pose.pose.position.y
            self.initial_position.z = msg.pose.pose.position.z
            self.current_displacement = 0.0
        else:
            current_position = msg.pose.pose.position
            self.current_displacement = math.sqrt(
                math.pow(current_position.x - self.initial_position.x, 2.0) +
                math.pow(current_position.y - self.initial_position.y, 2.0)
            )

    def run_test(self):
        self.test_in_progress = True
        test_name = f'Drive {self.goal_distance:0.1f}m'

        user_response = self.promptYN(f"""The robot will drive forwards approximately {self.goal_distance}m
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")  # noqa: E501
        if user_response == 'N':
            return [ClearpathTestResult(False, test_name, 'User skipped')]

        self.get_logger().info('Starting drive test')
        self.start()
        start_time = self.get_clock().now()
        while not self.test_done and not self.test_error:
            rclpy.spin_once(self)
        end_time = self.get_clock().now()

        # ensure we stop the robot
        # the publising timer is still running
        # we need to support omni robots, so set linear X and Y!
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0

        results = self.test_results

        if self.test_error:
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
        else:
            expected_duration = Duration(
                seconds=self.goal_distance /
                math.sqrt(self.max_speed_x ** 2 + self.max_speed_y ** 2)
            )
            test_duration = end_time - start_time

            time_accuracy = (
                min(
                    expected_duration.nanoseconds,
                    test_duration.nanoseconds) /
                max(
                    expected_duration.nanoseconds,
                    test_duration.nanoseconds
                )
            )
            if time_accuracy < 0.75:
                results.append(ClearpathTestResult(
                    False,
                    f'{test_name} (duration)',
                    f'Robot took {test_duration.nanoseconds / 1000000000:0.2f}s to drive {self.goal_distance}m vs {expected_duration.nanoseconds / 1000000000:0.2f}s expected (err={1.0 - time_accuracy:0.4f})'  # noqa: E501
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{test_name} (duration)',
                    None
                ))

            user_response = self.promptYN(
                f"""Test complete.
    Measure the robot's actual displacement.
    Is it between {self.goal_distance * (1.0 - self.error_margin):0.2f}m and {self.goal_distance * (1.0 + self.error_margin):0.2f}m?"""  # noqa: E501
            )
            if user_response == 'N':
                measured_distance = input('How far did the robot actually drive (in meters)? ')
                results.append(ClearpathTestResult(
                    False,
                    f'{test_name} (accuracy)',
                    f'Incorrect distance: {measured_distance}'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{test_name} (accuracy)',
                    None
                ))

        return results


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        dt = DriveTestNode(setup_path)
        dt.start()
        try:
            while not dt.test_done:
                rclpy.spin_once(dt)
            dt.get_logger().info('Test complete')
        except KeyboardInterrupt:
            dt.get_logger().info('User aborted! Cleaning up & exiting...')
        dt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
