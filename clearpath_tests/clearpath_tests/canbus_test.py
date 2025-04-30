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
import subprocess

from clearpath_generator_common.common import BaseGenerator
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

import rclpy


class CanbusTestNode(ClearpathTestNode):
    """Check the output of candump and count the number if IDs."""

    def __init__(
        self,
        can_interface='',
        n_devices=0,
        expected_msg_length=0,
        setup_path='/etc/clearpath'
    ):
        super().__init__(
            'CAN',
            f'canbus_test_{can_interface}',
            setup_path
        )

        self.can_interface = self.get_parameter_or('can_interface', can_interface)
        self.n_devices = self.get_parameter_or('n_devices', n_devices)
        self.msg_length = self.get_parameter_or('msg_len', expected_msg_length)

        if len(self.can_interface) == 0:
            self.get_logger().warning('No CAN interface specified; falling back to vcan0')
            self.can_interface = 'vcan0'

        if self.n_devices <= 0:
            self.get_logger().warning('Permssive number of devices; any number of IDs will be accepted')  # noqa: E501
            self.n_devices = 0

        if self.msg_length <= 0:
            self.get_logger().warning('Permissive message length; all messages will be accepted')

        self.test_name = f'CAN {self.can_interface}'

    def start(self):
        while True:
            result = self.read_can_log()

            if result.success:
                self.get_logger().info(result.message)
            else:
                self.get_logger().warning(result.message)

    def run_test(self):
        self.get_logger().info(f'Collecting CAN messages on {self.can_interface}...')
        result = self.read_can_log()
        return [result]

    def read_can_log(self):
        """
        Process the output of candump and check for the number of unique IDs.

        Output looks like

          vcan1  631   [8]  80 00 00 00 00 00 04 05
          vcan1  000   [2]  01 31
          vcan1  631   [8]  40 00 D0 20 00 00 00 00
          vcan1  631   [8]  80 00 00 00 00 00 04 05
          vcan1  632   [8]  40 00 18 01 00 00 00 00
          vcan1  632   [8]  80 00 00 00 00 00 04 05
          vcan1  000   [2]  01 32
          vcan1  632   [8]  40 00 D0 20 00 00 00 00
          vcan1  632   [8]  80 00 00 00 00 00 04 05
          vcan1  631   [8]  40 00 18 01 00 00 00 00
          vcan1  631   [8]  80 00 00 00 00 00 04 05
          vcan1  000   [2]  01 31
          vcan1  631   [8]  40 00 D0 20 00 00 00 00

        Second column is the message, last character of message is the ID

        @return  A ClearpathTestResult object indicating the results
        """
        candump = subprocess.Popen([
                'timeout',
                '5s',
                'candump',
                self.can_interface,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        (stdout, stderr) = candump.communicate()
        stdout = stdout.decode()
        stderr = stderr.decode()

        if 'no such device' in stderr.lower():
            return ClearpathTestResult(
                False,
                f'CAN {self.can_interface}',
                f'{self.can_interface} does not exist'
            )
        else:
            multiple_whitespace = re.compile(r'\s\s+')
            can_ids = set()
            for line in stdout.splitlines():
                line = line.strip()
                tokens = re.split(multiple_whitespace, line)
                interface = tokens[0]
                msg = tokens[1]
                length = int(tokens[2].replace('[', '').replace(']', ''))

                # CANopen IDs are the last 7 bits
                # TODO: some devices (e.g. Puma, Autec, Wiferion) may not use CANopen
                # and will have their IDs incorrectly identified using this method
                # Eventually we will likely need to filter messages by ID and do some
                # distinct processing/filtering to get their IDs
                can_id = msg[-2:]
                can_id = int(can_id, 16) & 0b1111111

                if (
                    interface == self.can_interface and
                    (length == self.msg_length or self.msg_length <= 0)
                ):
                    can_ids.add(can_id)

            self.detected_ids = can_ids

            if (
                len(can_ids) == self.n_devices or
                self.n_devices <= 0
            ):
                return ClearpathTestResult(
                    True,
                    f'CAN {self.can_interface}',
                    f'{len(can_ids)} detected on network'
                )
            else:
                return ClearpathTestResult(
                    False,
                    f'CAN {self.can_interface}',
                    f'{len(can_ids)} detected on network; expected {self.n_devices}'
                )

    def get_test_result_details(self):
        details = '\n#### Detected CANopen device IDs\n\n'

        ids = list(self.detected_ids)
        ids.sort()
        for can_id in ids:
            details += f'* {can_id}\n'

        details += '\nDevices that do not use CANopen may be incorrectly identified in the list above'  # noqa: E501
        return details


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    ct = CanbusTestNode(setup_path=setup_path)

    try:
        ct.start()
        rclpy.spin(ct)
    except KeyboardInterrupt:
        pass

    ct.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
