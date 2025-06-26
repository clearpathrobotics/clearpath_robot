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
import re

from clearpath_config.common.types.platform import Platform
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult
from clearpath_tests.timer import Timeout

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import rclpy
from rclpy.qos import qos_profile_system_default


PLATFORM_ANY = '*'


# Allowed warnings & errors that we can silently drop on each platform
# Key is status.name, value is an array of regexes or strings we match against
allowed_errors_by_platform = {
    PLATFORM_ANY: {
        'joy_node: Joystick Driver Status': [
            re.compile(r'.*Joystick not open.*'),
        ],
        'controller_manager: Hardware Components Activity': [
            'High execution jitter or mean error',
        ],
        'controller_manager: Controllers Activity': [
            'High execution jitter or mean error',
        ],
    },
    Platform.A200: {
        # no A200-specific exceptions
    },
    Platform.A300: {
        # no A300-specific exceptions
    },
    Platform.DD100: {
        # no DD100-specific exceptions
    },
    Platform.DD150: {
        # no DD150-specific exceptions
    },
    Platform.DO100: {
        # required until ros2_controllers PR 1772 is fully released
        'ekf_node: Filter diagnostic updater': [
            'Potentially erroneous data or settings detected for a robot_localization state',
        ]
    },
    Platform.DO150: {
        # required until ros2_controllers PR 1772 is fully released
        'ekf_node: Filter diagnostic updater': [
            'Potentially erroneous data or settings detected for a robot_localization state',
        ]
    },
    Platform.GENERIC: {
        # no generic-specific exceptions
    },
    Platform.J100: {
        # no J100-specific exceptions
    },
    Platform.R100: {
        # required until ros2_controllers PR 1772 is fully released
        'ekf_node: Filter diagnostic updater': [
            'Potentially erroneous data or settings detected for a robot_localization state',
        ]
    },
    Platform.W200: {
        # no W200-specific exceptions
    },
}


class DiagnosticTestNode(ClearpathTestNode):
    """Monitors diagnostic topics to make sure there aren't any warnings."""

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Diagnostics', 'diagnostic_test', setup_path)
        self.test_in_progress = False
        self.warnings = {}
        self.errors = {}
        self.stale = {}
        self.allowed_errors = {}

    def log_error(self, status, pool):
        key = f'{status.name}/{status.message}'

        if key not in pool.keys():
            allowed = False
            patterns = (
                allowed_errors_by_platform[PLATFORM_ANY].get(status.name, []) +
                allowed_errors_by_platform[self.clearpath_config.get_platform_model()].get(status.name, [])  # noqa: E501
            )
            for p in patterns:
                if (
                    (type(p) is str and p in status.message)
                    or (type(p) is re.Pattern and re.match(p, status.message))
                ):
                    allowed = True
                    break

            if not allowed:
                pool[key] = status
                if not self.test_in_progress:
                    self.get_logger().warning(
                        f'Diagnostics: {status.name} ({status.level}): {status.message}'
                    )
            else:
                self.allowed_errors[key] = status

    def diagnostic_callback(self, diagnostic_array):
        """
        Check the statuses in the array for warnings & errors.

        @param diagnostic_array  The message received on the diagnostic topic
        """
        for status in diagnostic_array.status:
            if status.level == DiagnosticStatus.OK:
                pass
            elif status.level == DiagnosticStatus.WARN:
                self.log_error(status, self.warnings)
            elif status.level == DiagnosticStatus.ERROR:
                self.log_error(status, self.errors)
            elif status.level == DiagnosticStatus.STALE:
                pass

    def diagnostic_agg_callback(self, diagnostic_array):
        """
        Check the statuses in the array for warnings & errors.

        @param diagnostic_array  The message received on the diagnostic topic
        """
        for status in diagnostic_array.status:
            if status.level == DiagnosticStatus.STALE:
                self.log_error(status, self.stale)

    def run_test(self):
        results = []
        self.test_in_progress = True

        # collect 30s worth of data
        self.get_logger().info('Collecting 30 seconds of diagnostic data...')
        self.diagnostc_sub = self.create_subscription(
            DiagnosticArray,
            f'/{self.namespace}/diagnostics',
            self.diagnostic_callback,
            qos_profile_system_default
        )
        self.diagnostc_agg_sub = self.create_subscription(
            DiagnosticArray,
            f'/{self.namespace}/diagnostics_agg',
            self.diagnostic_agg_callback,
            qos_profile_system_default
        )
        timeout = Timeout(self, 30)
        while not timeout.elapsed:
            rclpy.spin_once(self, timeout_sec=1.0)

        if (len(self.stale) == 0 and len(self.warnings) == 0 and
                len(self.errors) == 0 and len(self.allowed_errors) == 0):
            results.append(ClearpathTestResult(True, 'Diagnostics', 'No errors, no warnings'))
        elif len(self.stale) == 0 and len(self.warnings) == 0 and len(self.errors) == 0:
            results.append(ClearpathTestResult(
                True,
                'Diagnostics',
                f'{len(self.allowed_errors)} allowed errors/warnings',
            ))
        elif len(self.stale) == 0 and len(self.errors) == 0:
            results.append(ClearpathTestResult(
                False,
                'Diagnostics',
                f'No stale, no errors, {len(self.warnings)} warnings, {len(self.allowed_errors)} allowed errors/warnings',  # noqa: E501
            ))
        else:
            results.append(ClearpathTestResult(
                False,
                'Diagnostics',
                f'{len(self.stale)} stale, {len(self.errors)} errors, {len(self.warnings)} warnings, {len(self.allowed_errors)} allowed errors/warnings',  # noqa: E501
            ))

        return results

    def get_test_result_details(self):
        details = None

        if len(self.stale) > 0:
            details = ''
            details += '\n#### Stale diagnostics recorded\n\n'
            for err in self.stale.values():
                details += f'* {err.name}\n'

        if len(self.errors) > 0:
            details = ''
            details += '\n#### Errors recorded\n\n'
            for err in self.errors.values():
                details += f'* {err.name}: {err.message}\n'

        if len(self.warnings) > 0:
            if details is None:
                details = ''
            details += '\n#### Warnings recorded\n\n'
            for warn in self.warnings.values():
                details += f'* {warn.name}: {warn.message}\n'

        if len(self.allowed_errors) > 0:
            if details is None:
                details = ''
            details += '\n#### Ignored/allowed warnings & errors\n\n'
            for warn in self.allowed_errors.values():
                details += f'* {warn.name}: {warn.message}\n'

        return details
