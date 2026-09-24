#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
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
"""
Assert that a zero cmd_vel following a non-zero, timed-out cmd_vel holds the robot still.

Motivating regression (ros2_controllers, fix/mecanum-zero-all-wheels
ef3d2732 + the analogous fix in diff_drive_controller): the platform velocity
controller's rate-limiter kept its `previous_two_commands_` history populated
while the reference timed out into NaN and the safety branch zeroed the
wheels. When the operator re-tapped the deadman with the stick centered, the
limiter saw the stale non-zero previous command and slewed from it toward
zero under the deceleration bound, briefly commanding the wheels non-zero.

The test asserts the observable property ("zero commanded -> robot stationary")
rather than any specific internal mechanism, so it catches any future
regression in `ros2_controllers`, `twist_mux`, or the Clearpath reference
pipeline that reintroduces motion after a zero command.
"""

from clearpath_tests.mobility_test import MobilityTestNode
from clearpath_tests.test_node import ClearpathTestResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# Motion detected in the ZERO phase above these thresholds fails the test.
# Chosen conservatively above expected odom/EKF noise but well below the
# observed burst (~0.10-0.15 m/s body, ~1-3 rad/s wheel on DO150).
_ODOM_LINEAR_THRESHOLD = 0.03  # m/s
_ODOM_ANGULAR_THRESHOLD = 0.05  # rad/s
_WHEEL_VELOCITY_THRESHOLD = 0.3  # rad/s


class _Phase:
    WARMUP = 'warmup'
    SILENT = 'silent'
    ZERO = 'zero'
    DONE = 'done'


class ZeroSpeedCommandHoldTestNode(MobilityTestNode):
    """Drive briefly, stop publishing long enough for the safety branch to fire, then command zero and assert the robot stays still."""  # noqa: E501

    def __init__(
        self,
        setup_path='/etc/clearpath',
        warmup_speed_x=0.15,
        warmup_speed_y=0.0,
        warmup_duration_s=1.5,
        silent_duration_s=1.0,
        zero_duration_s=1.0,
    ):
        """Configure warmup/silent/zero phase durations and reference speed."""
        super().__init__(
            'Zero speed command hold',
            'zero_speed_command_hold_test',
            setup_path,
        )

        self.warmup_speed_x = self.get_parameter_or('warmup_speed_x', warmup_speed_x)
        self.warmup_speed_y = self.get_parameter_or('warmup_speed_y', warmup_speed_y)
        self.warmup_duration = Duration(
            seconds=self.get_parameter_or('warmup_duration_s', warmup_duration_s)
        )
        self.silent_duration = Duration(
            seconds=self.get_parameter_or('silent_duration_s', silent_duration_s)
        )
        self.zero_duration = Duration(
            seconds=self.get_parameter_or('zero_duration_s', zero_duration_s)
        )

        self.phase = _Phase.WARMUP
        self.phase_start = None

        # Peak absolute values observed during the ZERO phase.
        self.peak_odom_linear = 0.0
        self.peak_odom_angular = 0.0
        # Map joint name -> peak |velocity| observed during the ZERO phase.
        self.peak_wheel_velocities = {}

        self.joint_state_topic = f'/{self.namespace}/platform/joint_states'
        self.joint_state_sub = None

    def start(self):
        """Subscribe to joint_states in addition to the base mobility subscriptions."""
        super().start()
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.phase = _Phase.WARMUP
        # phase_start is initialized once odometry is flowing, so time spent
        # waiting for the first odom message doesn't eat into the WARMUP window.
        self.phase_start = None

    def _in_zero_phase(self) -> bool:
        return self.phase == _Phase.ZERO

    def _elapsed_in_phase(self) -> Duration:
        return self.get_clock().now() - self.phase_start

    def publish_callback(self):
        """Advance the WARMUP/SILENT/ZERO state machine and publish cmd_vel accordingly."""
        if self.latest_odom is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error(
                    'Timed out waiting for odometry. Terminating test'
                )
                raise TimeoutError('Timed out waiting for odometry')
            return

        if self.phase_start is None:
            self.phase_start = self.get_clock().now()

        if self.phase == _Phase.WARMUP:
            self.cmd_vel.twist.linear.x = self.warmup_speed_x
            self.cmd_vel.twist.linear.y = self.warmup_speed_y
            self.cmd_vel.twist.angular.z = 0.0
            super().publish_callback()
            if self._elapsed_in_phase() >= self.warmup_duration:
                self.phase = _Phase.SILENT
                self.phase_start = self.get_clock().now()
                self.get_logger().info(
                    'WARMUP -> SILENT: pausing cmd_vel to trigger controller safety branch'
                )

        elif self.phase == _Phase.SILENT:
            # Intentionally do not publish: force the controller reference to
            # time out into NaN so the safety branch (and the fix under test)
            # actually run.
            if self._elapsed_in_phase() >= self.silent_duration:
                self.cmd_vel.twist.linear.x = 0.0
                self.cmd_vel.twist.linear.y = 0.0
                self.cmd_vel.twist.angular.z = 0.0
                self.phase = _Phase.ZERO
                self.phase_start = self.get_clock().now()
                self.get_logger().info(
                    'SILENT -> ZERO: resuming cmd_vel with all zeros; watching for burst'
                )

        elif self.phase == _Phase.ZERO:
            self.cmd_vel.twist.linear.x = 0.0
            self.cmd_vel.twist.linear.y = 0.0
            self.cmd_vel.twist.angular.z = 0.0
            super().publish_callback()
            if self._elapsed_in_phase() >= self.zero_duration:
                self.phase = _Phase.DONE
                self.test_done = True

    def odom_callback(self, msg):
        """Track peak body twist observed during the ZERO phase."""
        super().odom_callback(msg)
        if not self._in_zero_phase():
            return
        twist = msg.twist.twist
        # Full 2D body speed (not just linear.x) so mecanum lateral bursts also count.
        linear = (twist.linear.x**2 + twist.linear.y**2) ** 0.5
        if linear > self.peak_odom_linear:
            self.peak_odom_linear = linear
        angular = abs(twist.angular.z)
        if angular > self.peak_odom_angular:
            self.peak_odom_angular = angular

    def _joint_state_callback(self, msg: JointState):
        if not self._in_zero_phase():
            return
        for name, velocity in zip(msg.name, msg.velocity):
            if 'wheel' not in name:
                continue
            speed = abs(velocity)
            if speed > self.peak_wheel_velocities.get(name, 0.0):
                self.peak_wheel_velocities[name] = speed

    def get_test_result_details(self) -> str:
        """Append peak odom and wheel velocities during the ZERO phase to the report."""
        details = super().get_test_result_details() or ''
        details += '\n#### Peak motion during ZERO phase\n\n'
        details += (
            f'* Odom linear: {self.peak_odom_linear:0.4f} m/s '
            f'(threshold {_ODOM_LINEAR_THRESHOLD:0.2f})\n'
        )
        details += (
            f'* Odom angular: {self.peak_odom_angular:0.4f} rad/s '
            f'(threshold {_ODOM_ANGULAR_THRESHOLD:0.2f})\n'
        )
        if self.peak_wheel_velocities:
            details += f'* Wheel |velocity| (threshold {_WHEEL_VELOCITY_THRESHOLD:0.2f} rad/s):\n'
            for name, speed in sorted(self.peak_wheel_velocities.items()):
                details += f'  * {name}: {speed:0.4f} rad/s\n'
        else:
            details += '* No wheel joint velocities were sampled.\n'
        return details

    def run_test(self):
        """Prompt the operator, run the phase machine, and return pass/fail results."""
        self.test_in_progress = True

        user_response = self.promptYN(
            'The robot will drive forwards ~15 cm, pause briefly, then hold still.\n'
            'It must be on the ground, all e-stops cleared, and at least 1 m of\n'
            'clear space in front.\n'
            'Are all these conditions met?'
        )
        if user_response == 'N':
            return [ClearpathTestResult(False, self.test_name, 'User skipped')]

        self.get_logger().info('Starting zero-speed-command-hold regression test')
        self.start()
        while not self.test_done and not self.test_error:
            rclpy.spin_once(self)

        # Belt-and-braces: leave the cmd_vel state at zero.
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0

        results = list(self.test_results)

        if self.test_error:
            self.get_logger().warning(
                f'Test aborted due to an error: {self.test_error_msg}'
            )
            return results

        odom_ok = (
            self.peak_odom_linear < _ODOM_LINEAR_THRESHOLD
            and self.peak_odom_angular < _ODOM_ANGULAR_THRESHOLD
        )
        wheels_ok = all(
            speed < _WHEEL_VELOCITY_THRESHOLD
            for speed in self.peak_wheel_velocities.values()
        )

        if odom_ok and wheels_ok:
            results.append(ClearpathTestResult(True, self.test_name, None))
        else:
            reasons = []
            if not odom_ok:
                reasons.append(
                    f'odom peak linear {self.peak_odom_linear:0.3f} m/s / '
                    f'angular {self.peak_odom_angular:0.3f} rad/s'
                )
            if not wheels_ok:
                offenders = [
                    f'{name} {speed:0.2f} rad/s'
                    for name, speed in self.peak_wheel_velocities.items()
                    if speed >= _WHEEL_VELOCITY_THRESHOLD
                ]
                reasons.append('wheels: ' + ', '.join(offenders))
            results.append(
                ClearpathTestResult(
                    False,
                    self.test_name,
                    'Robot moved after commanding zero (' + '; '.join(reasons) + ')',
                )
            )

        return results
