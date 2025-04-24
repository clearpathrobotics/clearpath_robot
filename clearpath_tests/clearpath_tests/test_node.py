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
import os

from clearpath_config.clearpath_config import ClearpathConfig
from rclpy.node import Node


class ClearpathTestNode(Node):
    """
    Generic superclass for test nodes.

    These nodes can either be run independently or can be run as part of a
    multi-element testing suite
    """

    def __init__(self, test_name: str, node_name: str, setup_path: str = '/etc/clearpath'):
        super().__init__(node_name)
        self.test_name = test_name

        self.test_done = False

        # Define paths
        self.setup_path = setup_path
        self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        # Parse YAML into config
        self.clearpath_config = ClearpathConfig(self.config_path)
        self.platform = self.clearpath_config.platform.get_platform_model()
        self.namespace = self.clearpath_config.get_namespace()

    def __str__(self):
        return self.test_name

    def run_test(self):
        """
        Run this node interactively, prompting the user for input.

        This function must be implemented by all children

        @return An array of TestResults describing the results
        """
        raise NotImplementedError()

    def get_test_result_details(self) -> str:
        """
        Get additional details about the test results.

        These details get included in the report, but are not printed out.

        If the test does not support additional details, simply return None
        """
        return None

    def start(self):
        """
        Run this node normally as a standard ROS node without any user interaction.

        This function must be implemented by all children
        """
        raise NotImplementedError()

    @staticmethod
    def promptYN(message: str, default: str = 'Y'):
        """
        Ask the user a yes/no question and return their response.

        @param message  The prompt to give the user
        @param default  Either Y or N, indicating the default response if the user just pressed
                        ENTER without any other characters
        @return Either "Y" or "N"
        """
        if default:
            default = default.upper()
        if default != 'N' and default != 'Y':
            raise ValueError(f'Invalid default: "{default}"')

        if default == 'Y':
            hint = '(Y/n)'
        else:
            hint = '(y/N)'

        prompt = f'\n{message} {hint} > '

        user_input = None
        while (
            user_input is None or (
                user_input != 'Y' and
                user_input != 'YES' and
                user_input != 'N' and
                user_input != 'NO' and
                user_input != ''
            )
        ):
            user_input = input(prompt).strip().upper()

        if len(user_input) == 0:
            return default
        else:
            return user_input[0].upper()


class ClearpathTestResult:
    def __init__(self, success: bool, name: str, message: str):
        """
        Create a test result we can log.

        @param success  Was the test successful?
        @param name  What is the name of this test (used for table headers)
        @param message  Any additional information about the test
        """
        if message is None:
            self.message = ''
        else:
            self.message = message

        self.success = success
        self.name = name

    def __str__(self):
        if self.success is None:
            pass_fail = 'n/a'
        elif self.success:
            pass_fail = 'pass'
        else:
            pass_fail = 'fail'

        if self.message:
            return f'{self.name}: {pass_fail} ({self.message})'
        else:
            return f'{self.name}: {pass_fail}'
