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
import re
import subprocess
import time

from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import BaseGenerator
from clearpath_platform_msgs.msg import Status
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration


MCU_IP = 1
MCU_SERIAL = 2

IP_RE = re.compile(r'^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$')
SERIAL_RE = re.compile(r'^/dev/.+')


class McuTestNode(ClearpathTestNode):
    """Check that the MCU is connected."""

    def __init__(
        self,
        mode=None,
        address=None,
        setup_path='/etc/clearpath'
    ):
        """
        Create the test node.

        @param mode  One of MCU_IP or MCU_SERIAL indicating how the communication occur
        @param address  Either the IP address or serial interface file for the MCU, depending
                        on the specified mode

        @exception ValueError if the address does not match the expectations for the given mode
                   or of the specified mode is not from the given list
        """
        super().__init__(
            'MCU',
            'mcu_test',
            setup_path
        )

        # get the defaults for the platform if nothing was passed-in
        if mode is None:
            self.get_logger().info(f'Getting default MCU mode for platform {self.platform}')
            if (
                self.platform == Platform.A200 or
                self.platform == Platform.J100
            ):
                mode = MCU_SERIAL
            elif (
                self.platform == Platform.A300 or
                self.platform == Platform.DD100 or
                self.platform == Platform.DD150 or
                self.platform == Platform.DO100 or
                self.platform == Platform.DO150 or
                self.platform == Platform.R100 or
                self.platform == Platform.W200
            ):
                mode = MCU_IP

        if address is None:
            self.get_logger().info(f'Getting default MCU address for platform {self.platform}')
            if self.platform == Platform.A200:
                address = '/dev/clearpath/prolific'
            elif self.platform == Platform.J100:
                address = '/dev/clearpath/j100'
            elif (
                self.platform == Platform.A300 or
                self.platform == Platform.DD100 or
                self.platform == Platform.DD150 or
                self.platform == Platform.DO100 or
                self.platform == Platform.DO150 or
                self.platform == Platform.R100 or
                self.platform == Platform.W200
            ):
                address = '192.168.131.2'

        self.mode = mode
        self.address = address

        if self.mode == MCU_IP:
            if not re.match(IP_RE, self.address):
                raise ValueError(f'Address {self.address} is not an IP address')
        elif self.mode == MCU_SERIAL:
            if not re.match(SERIAL_RE, self.address):
                raise ValueError(f'Path {self.address} is not a valid serial handle')
        else:
            raise ValueError(f'Unknown mode {self.mode}')

    def ping_ip(self, address):
        """
        Attempt to ping an IP address.

        We ping 10 times, and accept a 50% response as the minimum

        @param address  The IP address to ping

        @return  True if the address is pinged successfully, otherwise False
        """
        N_PINGS = 10
        n_recvd_re = re.compile(r'[0-9]+ received')

        ping_proc = subprocess.Popen([
                'ping',
                '-c',
                f'{N_PINGS}',
                self.address,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        (stdout, stderr) = ping_proc.communicate()

        stdout = stdout.decode()
        search_result = re.search(n_recvd_re, stdout)
        if search_result is None:
            return False

        search_result = search_result.string[search_result.span()[0]:search_result.span()[1]]
        n = int(search_result.split()[0])
        return n >= (N_PINGS / 2)

    def check_serial_exists(self, device):
        """
        Check that a serial device exists on the system.

        Also checks that we have read/write permissions to the device

        @param device  The serial device handle (e.g. /dev/ttyUSB0)

        @return True if the handle exists, otherwise False
        """
        return os.path.isfile(device) or os.path.islink(device)

    def check_serial_permissions(self, device):
        """
        Check  that we have read/write permissions to the device.

        @param device  The serial device handle (e.g. /dev/ttyUSB0)

        @return True if we have RW permissions, otherwise False
        """
        exists = self.check_serial_exists(device)
        perms_ok = False

        if exists:
            perms_ok = os.access(device, os.R_OK) and os.access(device, os.W_OK)
        return exists and perms_ok

    def get_firmware_version(self):
        """
        Get the MCU version and hardware ID.

        @return A tuple of the form (hardware_id, firmware_version)
        """
        self.mcu_status = None

        def mcu_callback(status):
            self.mcu_status = status

        mcu_sub = self.create_subscription(
            Status,
            f'/{self.namespace}/platform/mcu/status',
            mcu_callback,
            qos_profile_sensor_data,
        )

        start_at = self.get_clock().now()
        timeout_duration = Duration(seconds=10)
        while self.get_clock().now() - start_at <= timeout_duration and self.mcu_status is None:
            rclpy.spin_once(self)
        mcu_sub.destroy()

        if self.mcu_status is None:
            return (None, None)
        else:
            return (self.mcu_status.hardware_id, self.mcu_status.firmware_version)

    def start(self):
        while True:
            if self.mode == MCU_IP:
                if self.ping_ip(self.address):
                    self.get_logger().info(f'MCU is responded to ping {self.address}')
                else:
                    self.get_logger().warning(f'MCU did not respond to ping {self.address}')
            elif self.mode == MCU_SERIAL:
                if self.check_serial_exists(self.address):
                    if self.check_serial_permissions(self.address):
                        self.get_logger().info(f'MCU handle {self.address} exists with RW permissions')  # noqa: E501
                    else:
                        self.get_logger().warning(f'Invalid permissions for MCU handle {self.address}')  # noqa: E501
                else:
                    self.get_logger().warning(f'MCU handle {self.address} does not exist')
                time.sleep(5)

    def run_test(self):
        if self.mode == MCU_IP:
            if self.ping_ip(self.address):
                return [ClearpathTestResult(
                    True,
                    self.test_name,
                    f'Successfully pinged MCU at {self.address}'
                )]
            else:
                return [ClearpathTestResult(
                    False,
                    self.test_name,
                    f'Failed to ping MCU at {self.address}'
                )]
        elif self.mode == MCU_SERIAL:
            if self.check_serial_exists(self.address):
                if self.check_serial_permissions(self.address):
                    return [ClearpathTestResult(
                        True,
                        self.test_name,
                        f'MCU handle {self.address} exists with RW permissions'
                    )]
                else:
                    return [ClearpathTestResult(
                        False,
                        self.test_name,
                        f'MCU handle {self.address} has invalid permissions'
                    )]
            else:
                return [ClearpathTestResult(
                    False,
                    self.test_name,
                    f'MCU handle {self.address} does not exist'
                )]

    def get_test_result_details(self):
        (hardware_id, firmware_version) = self.get_firmware_version()

        return f"""
#### MCU Details

* Hardware ID: {'unknown' if not hardware_id else hardware_id}
* Version: {'unknown' if not firmware_version else firmware_version}

"""


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    mt = McuTestNode(setup_path=setup_path)

    try:
        mt.start()
        rclpy.spin(mt)
    except KeyboardInterrupt:
        pass

    mt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
