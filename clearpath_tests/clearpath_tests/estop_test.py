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
import time

from clearpath_generator_common.common import BaseGenerator
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from std_msgs.msg import Bool


class EstopTestNode(ClearpathTestNode):
    """Ensure e-stop works correctly."""

    def __init__(
        self,
        estop_location,
        setup_path='/etc/clearpath',
        optional=False,
        estop_type='E-Stop',
    ):
        super().__init__(
            f'{estop_type} ({estop_location})',
            f'estop_test_{estop_location.strip().lower().replace(" ", "_").replace("-", "_")}',
            setup_path
        )
        self.optional = optional
        self.estop_engaged = None
        self.test_in_progress = False
        self.estop_location = estop_location
        self.estop_type = estop_type

    def estop_callback(self, msg: Bool):
        self.estop_engaged = msg.data

        if not self.test_in_progress:
            if self.estop_engaged is None:
                state = 'unknown'
            elif self.estop_engaged:
                state = 'stopped'
            else:
                state = 'clear'
            self.get_logger().info(f'{self.estop_type} state: {state}')

    def start(self):
        self.estop_sub = self.create_subscription(
            Bool,
            f'/{self.namespace}/platform/emergency_stop',
            self.estop_callback,
            qos_profile_sensor_data
        )

    def run_test(self):
        if self.optional:
            user_input = self.promptYN(
                f'Does this robot have a {self.estop_location} {self.estop_type}?',
                default='N',
            )
            if user_input == 'N':
                return [ClearpathTestResult(None, self.test_name, 'Skipped; component not installed')]  # noqa: E501

        self.test_in_progress = True
        self.start()

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            f'/{self.namespace}/cmd_vel',
            qos_profile_system_default
        )
        self.cmd_vel = TwistStamped()
        self.cmd_vel_timer = self.create_timer(0.1, self.cmd_vel_timer_callback)  # 10Hz

        self.results = []
        self.test_done = False
        ui_thread = threading.Thread(target=self.run_ui)
        ui_thread.start()
        while not self.test_done:
            rclpy.spin_once(self)
        ui_thread.join()
        return self.results

    def cmd_vel_timer_callback(self):
        self.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(self.cmd_vel)

    def run_ui(self):
        results = self.results

        user_input = self.promptYN(
            """The robot will be commanded to drive forwards at 0.1m/s for 2s multiple
times during this test.
Ensure the robot is either on blocks with the wheels not on the ground
or that it is safe for the robot to drive forwards.
Safe to continue?"""
        )
        if user_input == 'N':
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                'User aborted; unsafe setup',
            ))
            self.test_done = True
            return

        # wait until we know the state of the e-stop
        start_time = self.get_clock().now()
        timeout = Duration(seconds=10)
        print(f'Getting {self.estop_type} status...')
        while (
            self.estop_engaged is None and
            (self.get_clock().now() - start_time) < timeout
        ):
            time.sleep(0.1)

        if self.estop_engaged is None:
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                f'Timed out waiting for {self.estop_type} state',
            ))
            self.test_done = True
            return

        if self.estop_engaged:
            print(f'{self.estop_type} engaged. Disengage {self.estop_type} now')
            if not self.wait_for_estop(False, 60):
                results.append(ClearpathTestResult(
                    False,
                    self.test_name,
                    f'Timed out waiting for user to clear {self.estop_type}'
                ))
                self.test_done = True
                return

        print(f'Engage the {self.estop_location} {self.estop_type} now.')
        if not self.wait_for_estop(True, 30):
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                f'{self.estop_type} failed to engage',
            ))
        else:
            results.append(ClearpathTestResult(
                True,
                self.test_name,
                f'{self.estop_type} engaged',
            ))

        user_input = self.promptYN('Will now command the wheels to turn\nSafe to proceed?')
        if user_input == 'N':
            safe_to_drive = False
            results.append(ClearpathTestResult(
                None,
                f'{self.test_name} (wheel rotation)',
                'User skipped; unsafe setup',
            ))
        else:
            safe_to_drive = True
            self.command_wheels()
            user_input = self.promptYN('Did the wheels rotate?', default='N')
            if user_input == 'Y':
                results.append(ClearpathTestResult(
                    False,
                    f'{self.test_name} (wheel rotation a)',
                    f'Wheels turned while {self.estop_type} engaged'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{self.test_name} (wheel rotation a)',
                    f'Wheels did not turn while {self.estop_type} engaged'
                ))

        print(f'Clear the {self.estop_location} emergency stop now.')
        if not self.wait_for_estop(False, 30):
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                f'{self.estop_type} failed to clear',
            ))
        else:
            results.append(ClearpathTestResult(
                True,
                self.test_name,
                f'{self.estop_type} cleared',
            ))

        if safe_to_drive:
            # wait 2s after clearing the e-stop to allow CAN connections to come back up
            # if needed; this affects Ridgeback primarily
            time.sleep(2.0)

            self.command_wheels()
            user_input = self.promptYN('Did the wheels rotate?')
            if user_input == 'N':
                results.append(ClearpathTestResult(
                    False,
                    f'{self.test_name} (wheel rotation b)',
                    f'Wheels did not turn while {self.estop_type} clear'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{self.test_name} (wheel rotation b)',
                    f'Wheels turned while {self.estop_type} clear'
                ))

        self.test_done = True

    def command_wheels(self):
        self.cmd_vel.twist.linear.x = 0.1

        start_time = self.get_clock().now()
        duration = Duration(seconds=2)
        while (self.get_clock().now() - start_time) < duration:
            pass

        self.cmd_vel.twist.linear.x = 0.0

    def wait_for_estop(self, state, timeout_seconds=10):
        """
        Wait for the e-stop state to enter the specified state.

        @param state  The desired state of the e-stop
        @param timeout_seconds  The maximum number of seconds to wait

        @return  True if the e-stop state is in the desired state, otherwise False
        """
        start_time = self.get_clock().now()
        now = self.get_clock().now()
        timeout = Duration(seconds=timeout_seconds)
        while (
            self.estop_engaged != state and
            (now - start_time) < timeout
        ):
            now = self.get_clock().now()

        return self.estop_engaged == state


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    st = EstopTestNode('', setup_path=setup_path)

    try:
        st.start()
        rclpy.spin(st)
    except KeyboardInterrupt:
        pass

    st.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
