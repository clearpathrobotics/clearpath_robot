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

from clearpath_generator_common.common import BaseGenerator
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult
import rclpy
from wireless_msgs.msg import Connection


class WifiTestNode(ClearpathTestNode):

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Wi-Fi', 'wifi_test', setup_path)

        self.wifis = self.get_wifi_interfaces()

    def get_wifi_interfaces(self):
        """Get the list if wifi interfaces."""
        # look for /sys/class/net/DEVICE/wireless
        wifis = []
        devices = os.listdir('/sys/class/net')
        for d in devices:
            if os.path.exists(os.path.join(
                '/sys/class/net', d, 'wireless'
            )):
                wifis.append(d)
        return wifis

    def check_connection(self, interface):
        """
        Get the connection information for the given wireless device.

        @param interface  The wireless interface to query
        """
        all_whitespace = re.compile(r'\s+')
        multiple_witespace = re.compile(r'\s\s+')

        p = subprocess.Popen([
                'iwconfig',
                interface,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        (stdout, _) = p.communicate()
        stdout = stdout.decode()
        c = Connection()

        for line in stdout.splitlines():
            tokens = re.split(multiple_witespace, line)
            for t in tokens:
                t = t.strip()
                if '=' in t:
                    key_value = t.split('=')
                else:
                    key_value = t.split(':')

                if len(key_value) == 2:
                    key = key_value[0]
                    value = key_value[1]

                    if key == 'ESSID':
                        c.essid = value.lstrip('"').rstrip('"')
                    elif key == 'Tx-Power':
                        c.txpower = int(re.split(all_whitespace, value)[0])
                    elif key == 'Bit Rate':
                        unit = re.split(all_whitespace, value)[-1]
                        multiplier = 1.0
                        if 'Gb/s' in unit:
                            multiplier = 1000.0
                        elif 'kb/s' in unit:
                            multiplier = 0.0001
                        c.bitrate = float(re.split(all_whitespace, value)[0]) * multiplier
                    elif key == 'Link Quality':
                        c.link_quality_raw = value
                        c.link_quality = (
                            float(value.split('/')[0]) /
                            float(value.split('/')[1])
                        )
                    elif key == 'Signal level':
                        c.signal_level = int(re.split(all_whitespace, value)[0])
                    elif key == 'Frequency':
                        c.frequency = float(re.split(all_whitespace, value)[0])

        return c

    def publish_callback(self):
        print('------------------------------')
        for wifi in self.wifis:
            c = self.check_connection(wifi)
            print(wifi)
            print(str(c))

    def start(self):
        self.publish_timer = self.create_timer(1, self.publish_callback)

    def run_test(self):
        # Clearpath internal SSIDs
        # If a robot is connected to these during production tests, raise a warning
        warn_ssids = [
            'BG-123',
            'BG-386',
            'LV-1201',
            'LV-223',
            'LV-426',
            'LV-485',
        ]

        results = []
        self.get_logger().info('Sampling wifi data...')
        n_samples = 5
        samples = {}
        for w in self.wifis:
            samples[w] = []
        for i in range(n_samples):
            for w in self.wifis:
                c = self.check_connection(w)
                samples[w].append(c)

            time.sleep(1)

        for w in self.wifis:
            bitrate = 0.0
            ssid = None
            for s in samples[w]:
                if ssid is None and s.essid and s.essid != 'off/any':
                    ssid = s.essid
                bitrate += s.bitrate / n_samples

            if ssid is None:
                results.append(ClearpathTestResult(
                    False,
                    f'{w} (connection)',
                    'No ESSID. Wifi not configured?'
                ))
            elif bitrate < 10:  # slower than 10Mb/s
                results.append(ClearpathTestResult(
                    False,
                    f'{w} (connection)',
                    f'Connected to {ssid}. Low bitrate {bitrate} Mb/s'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{w} (connection)',
                    f'Connected to {ssid}. Bitrate {bitrate} Mb/s'
                ))

            if ssid in warn_ssids:
                results.append(ClearpathTestResult(
                    None,
                    f'{w} (internal SSID)',
                    'Connected to internal network: do not ship without sanitizing!'
                ))

        return results


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    wt = WifiTestNode(setup_path)

    try:
        wt.start()
        rclpy.spin(wt)
    except KeyboardInterrupt:
        pass

    wt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
