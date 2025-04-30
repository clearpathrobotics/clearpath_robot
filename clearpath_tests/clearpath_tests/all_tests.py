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
"""
This file is the main entrypoint for production tests.

This script will run through each of the tests needed for the configured platform
and log the results to a specified output file.

The output file is in markdown format. Markdown files are human-readable, but
you can also install markdown renderers for Chrome, Firefox, VSCode, and other
browsers & editors.
"""
from datetime import datetime
import os
import subprocess

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.types.platform import Platform
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_tests import (
    canbus_test,
    diagnostic_test,
    drive_test,
    estop_test,
    fan_test,
    imu_test,
    light_test,
    linear_acceleration_test,
    mcu_test,
    rotation_test,
    wifi_test,
)
from clearpath_tests.test_node import (
    ClearpathTestNode,
    ClearpathTestResult,
)

import rclpy
from rclpy.node import Node


class TestingNode(Node):

    def __init__(self, node_name='clearpath_production_test_node'):
        super().__init__(node_name)

        self.setup_path = self.get_parameter_or('setup_path', '/etc/clearpath')
        self.setup_file = os.path.join(self.setup_path, 'robot.yaml')
        self.clearpath_config = ClearpathConfig(read_yaml(self.setup_file))

        self.platform = self.clearpath_config.get_platform_model()

        self.common_tests = [
            mcu_test.McuTestNode(setup_path=self.setup_path),
            diagnostic_test.DiagnosticTestNode(self.setup_path),
            wifi_test.WifiTestNode(self.setup_path),
        ]

        self.driving_tests = [
            drive_test.DriveTestNode(
                setup_path=self.setup_path,
                default_speed_x=0.1,
                direction='Forwards',
            ),
        ]

        # Add any platform-specific tests here
        self.tests_for_platform = []
        if self.platform == Platform.A200:
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear', self.setup_path))
        elif self.platform == Platform.A300:
            self.tests_for_platform.append(fan_test.FanTestNode(4, self.setup_path))
            self.tests_for_platform.append(light_test.LightTestNode(4, self.setup_path))

            self.tests_for_platform.append(estop_test.EstopTestNode('Front', self.setup_path))
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear', self.setup_path))
            self.tests_for_platform.append(
                # rear access hatch should also act as an e-stop
                estop_test.EstopTestNode('Access Panel', self.setup_path)
            )
            self.tests_for_platform.append(
                # wireless e-stop is optional on A300
                estop_test.EstopTestNode('Wireless', self.setup_path, optional=True)
            )

            self.tests_for_platform.append(imu_test.ImuTestNode(0, self.setup_path))

            # vcan0 has the 4 motor drivers
            # check for status messages, which are length 5 and more numerous
            self.tests_for_platform.append(canbus_test.CanbusTestNode('vcan0', 4, 5, self.setup_path))  # noqa: E501

            # vcan1 has batteries, optional e-stop, optional wireless charger
            # so just allow anything here
            self.tests_for_platform.append(canbus_test.CanbusTestNode('vcan1', 0, 0, self.setup_path))  # noqa: E501

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))
        elif (
            self.platform == Platform.DD100 or
            self.platform == Platform.DD150
        ):
            self.tests_for_platform.append(light_test.LightTestNode(4, self.setup_path))

            # Dingo doesn't use CANopen, so the ID counter won't work correctly
            self.tests_for_platform.append(canbus_test.CanbusTestNode('vcan0', 0, 4, self.setup_path))  # noqa: E501

            self.tests_for_platform.append(estop_test.EstopTestNode(
                'Rear',
                setup_path=self.setup_path,
                estop_type='Motor Cutoff',
            ))

            # Dingo has an integral IMU
            self.tests_for_platform.append(imu_test.ImuTestNode(setup_path=self.setup_path))

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))
        elif (
            self.platform == Platform.DO100 or
            self.platform == Platform.DO150
        ):
            self.tests_for_platform.append(light_test.LightTestNode(4, self.setup_path))

            # Dingo doesn't use CANopen, so the ID counter won't work correctly
            self.tests_for_platform.append(canbus_test.CanbusTestNode('vcan0', 0, 4, self.setup_path))  # noqa: E501

            self.tests_for_platform.append(estop_test.EstopTestNode(
                'Rear',
                setup_path=self.setup_path,
                estop_type='Motor Cutoff',
            ))

            # Dingo has an integral IMU
            self.tests_for_platform.append(imu_test.ImuTestNode(setup_path=self.setup_path))

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))

            self.driving_tests.append(
                drive_test.DriveTestNode(
                    setup_path=self.setup_path,
                    default_speed_x=0.0,
                    default_speed_y=0.2,
                    direction='Left',
                    default_distance=2.0,
                )
            )
        elif self.platform == Platform.GENERIC:
            pass
        elif self.platform == Platform.J100:
            self.tests_for_platform.append(imu_test.ImuTestNode(0, self.setup_path))

            self.tests_for_platform.append(estop_test.EstopTestNode(
                'Rear',
                setup_path=self.setup_path,
                estop_type='Motor Cutoff',
            ))

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))
        elif self.platform == Platform.R100:
            self.tests_for_platform.append(light_test.LightTestNode(8))

            # Ridgeback doesn't use CANopen, so the ID counter won't work correctly
            self.tests_for_platform.append(canbus_test.CanbusTestNode('vcan0', 0, 4, self.setup_path))  # noqa: E501

            self.tests_for_platform.append(estop_test.EstopTestNode('Front Left', self.setup_path))
            self.tests_for_platform.append(estop_test.EstopTestNode('Front Right', self.setup_path))  # noqa: E501
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear Left', self.setup_path))
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear Right', self.setup_path))

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))

            self.driving_tests.append(
                drive_test.DriveTestNode(
                    setup_path=self.setup_path,
                    default_speed_x=0.0,
                    default_speed_y=0.2,
                    direction='Left',
                    default_distance=2.0,
                )
            )
        elif self.platform == Platform.W200:
            self.tests_for_platform.append(light_test.LightTestNode(4))
            self.tests_for_platform.append(canbus_test.CanbusTestNode('can0', 4, 0, self.setup_path))  # noqa: E501

            self.tests_for_platform.append(estop_test.EstopTestNode('Front Left', self.setup_path))
            self.tests_for_platform.append(estop_test.EstopTestNode('Front Right', self.setup_path))  # noqa: E501
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear Left', self.setup_path))
            self.tests_for_platform.append(estop_test.EstopTestNode('Rear Right', self.setup_path))

            # Dynamic IMU tests
            self.driving_tests.insert(0, rotation_test.RotationTestNode(
                setup_path=self.setup_path
            ))
            self.driving_tests.insert(0, linear_acceleration_test.LinearAccelerationTestNode(
                setup_path=self.setup_path
            ))
        else:
            raise NotImplementedError(f'{self.platform} tests are not implemented')

        if os.environ['HOME']:
            default_log_dir = os.environ['HOME']
        else:
            self.get_logger().warning('$HOME is undefined; using /tmp as default report location')
            default_log_dir = '/tmp'

        timestamp = datetime.now().strftime('%Y%m%d%H%M')
        self.report_file = self.get_parameter_or(
            'report_file',
            os.path.join(
                default_log_dir,
                f'clearpath_test_results.{timestamp}.md'
            )
        )
        output_directory = os.path.dirname(self.report_file)
        self.bag_file = os.path.join(output_directory, f'clearpath_test_results.{timestamp}.mcap')

        self.test_results = []

    def destroy_node(self):
        """
        Clean up any in-progress tests.

        Also make sure the log file is closed.
        """
        return super().destroy_node()

    def create_summary(self):
        """
        Print the summary table of all tests and appends the same table to the report.

        Also prints out a one/two line summary of the number of tests passed/failed.
        """
        longest_test_name = 'Test'  # initialize t row header
        longest_test_message = 'Notes'  # initialize to row header
        for result in self.test_results:
            if len(result.name) > len(longest_test_name):
                longest_test_name = result.name
            if result.message and len(result.message) > len(longest_test_message):
                longest_test_message = result.message

        test_column_width = len(longest_test_name)
        result_column_width = len('Result')
        message_column_width = len(longest_test_message)

        table_md = f'| {"Test".ljust(test_column_width)} | Result | {"Notes".ljust(message_column_width)} |\n'  # noqa: E501
        table_md = table_md + f'|-{"-"*test_column_width}-|-{"-"*result_column_width}-|-{"-"*message_column_width}-|\n'  # noqa: E501

        n_passed = 0
        n_failed = 0
        for result in self.test_results:
            if result.success is None:
                pass_fail = 'n/a'
            elif result.success:
                pass_fail = 'pass'
                n_passed += 1
            else:
                pass_fail = 'fail'
                n_failed += 1

            table_md = table_md + f'| {result.name.ljust(test_column_width)} | {pass_fail.ljust(result_column_width)} | {(result.message if result.message else "").ljust(message_column_width)} |\n'   # noqa: E501

        with open(self.report_file, 'a') as report:
            report.write('\n## Summary\n')
            report.write(table_md)
            report.write('\n')
            if n_failed == 0:
                report.write('- all tests passed')
            else:
                report.write(f'- {n_passed} tests passed\n')
                report.write(f'- {n_failed} tests failed\n')

        print(f'\n\nSummary:\n{table_md}')
        if n_failed == 0:
            print('\nAll tests passed!\n')
        else:
            print(f'\n{n_passed} tests passed\n{n_failed} tests failed\n')

    def write_header(self):
        """
        Initialize the report file.

        This dumps some initial meta-data and a copy of robot.yaml
        """
        from importlib.metadata import version

        with open(self.report_file, 'w') as report:
            report.write(f"""# Clearpath Test Report

{datetime.now().strftime('%Y-%m-%d %H:%M')}

Using `clearpath_tests` v{version('clearpath_tests')}

Report generated by user `{os.getlogin()}`

Setup path: {self.setup_path}

Platform (serial): {self.clearpath_config.get_platform_model()} ({self.clearpath_config.get_serial_number()})

## robot.yaml
""")  # noqa: E501
        self.copy_file_contents(self.setup_file, 'yaml')

        with open(self.report_file, 'a') as report:
            report.write('\n## Test results\n\n')

    def copy_file_contents(self, path, src_formatting=None):
        """
        Copy the raw contents of a file into a code block.

        @param path  The path of the file to copy
        @param src_formatting  Optional code formatting parameter
                               (e.g. python, bash, yaml, xml, ...)
        """
        with open(path, 'r') as setup_file:
            file_contents = setup_file.readlines()
        self.write_code(file_contents, src_formatting)

    def write_code(self, code, src_formatting=None):
        """
        Add a code block to the report.

        The code block will always have a newline added before and after it

        @param code  Either a string or an array of strings representing the lines of code
        @param src_formatting  The markdown-compatible language
                               (e.g. python, bash, yaml, xml, ...)
        """
        with open(self.report_file, 'a') as report:
            if src_formatting:
                report.write(f'\n```{src_formatting}')
            else:
                report.write('\n```')

            if type(code) is str:
                if not code.startswith('\n'):
                    report.write('\n')
                report.write(code)
                if not code.endswith('\n'):
                    report.write('\n')
            else:
                if not code[0].startswith('\n'):
                    report.write('\n')
                for line in code:
                    report.write(line)
                if not code[-1].endswith('n'):
                    report.write('\n')

            report.write('```\n')

    def log_result(self, test_result: ClearpathTestResult):
        """
        Log the results of a test to the report.

        @param test_result  The result we want to log
        """
        self.test_results.append(test_result)

        with open(self.report_file, 'a') as report:
            report.write(f'{test_result}\n\n')

    def prompt_tests(self):
        """
        Ask the user to select what test(s) to run.

        Returns the array of tests the user selected
        """
        from simple_term_menu import TerminalMenu

        tests_in_order = [
            None  # placeholder for all tests
        ]
        menu_items = [
            'All tests'
        ]
        for test in self.common_tests:
            menu_items.append(f'{test}')
            tests_in_order.append(test)
        for test in self.tests_for_platform:
            menu_items.append(f'{test}')
            tests_in_order.append(test)
        for test in self.driving_tests:
            menu_items.append(f'{test}')
            tests_in_order.append(test)

        # Clearpath Tests -- https://patorjk.com/software/taag/#p=display&v=0&f=Small
        title = """
   ___ _                         _   _      _____       _
  / __| |___ __ _ _ _ _ __  __ _| |_| |_   |_   _|__ __| |_ ___
 | (__| / -_) _` | '_| '_ \\/ _` |  _| ' \\    | |/ -_|_-<  _(_-<
  \\___|_\\___\\__,_|_| | .__/\\__,_|\\__|_||_|   |_|\\___/__/\\__/__/
                     |_|

  Select tests.
  Press Q or Esc to quit.
"""

        main_menu = TerminalMenu(
            title=title,
            menu_entries=menu_items,
            show_multi_select_hint=True,
            multi_select=True,
            clear_screen=True,
        )

        menu_entry_indices = main_menu.show()

        if menu_entry_indices is None or len(menu_entry_indices) == 0:
            return []
        elif 0 in menu_entry_indices:
            tests_in_order.pop(0)
            return tests_in_order
        else:
            tests_to_run = []
            for i in menu_entry_indices:
                tests_to_run.append(tests_in_order[i])
            return tests_to_run

    def run_tests(self):
        """
        Go through the tests one at a time, logging the results as we go.

        The exact tests executed depends on the configured platform
        """
        tests_to_run = self.prompt_tests()

        self.write_header()

        if len(tests_to_run) == 0:
            self.get_logger().warning('No tests selected. Terminating')
            return

        n = 1
        for node in tests_to_run:
            self.get_logger().info(
                f'Starting ({node.test_name}) (test {n} of {len(tests_to_run)})'
            )
            user_input = node.promptYN(f'Ready to run test {node.test_name}?')
            details = None
            results = []
            if user_input == 'N':
                self.get_logger().info(f'User skipped test {node.test_name}')
                with open(self.report_file, 'a') as report:
                    report.write(f'### {node.test_name}\n\n')
                self.log_result(ClearpathTestResult(None, node.test_name, 'Skipped'))
            else:
                try:
                    results = node.run_test()
                    details = node.get_test_result_details()
                except Exception as err:
                    details = None
                    results = [ClearpathTestResult(False, node.test_name, str(err))]

                with open(self.report_file, 'a') as report:
                    report.write(f'### {node.test_name}\n\n')

                for result in results:
                    self.log_result(result)

                if details is not None:
                    with open(self.report_file, 'a') as report:
                        report.write(details)
                        report.write('\n')

            n += 1

        self.create_summary()


def start_bag_recording(test_node: TestingNode):
    FOUR_GiB = 4 * 2**30

    p = subprocess.Popen([
            'ros2',
            'bag',
            'record',
            '-e',
            f'/{test_node.clearpath_config.get_namespace()}/*',
            '--include-hidden-topics',
            '-s',
            'mcap',
            '-b',
            f'{FOUR_GiB}',
            '--disable-keyboard-controls',
            '-o',
            f'{test_node.bag_file}',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.PIPE,
    )
    return p


def main(args=None):
    rclpy.init(args=args)
    test_node = TestingNode()
    bag_proc = None

    try:
        if ClearpathTestNode.promptYN('Record bag during tests?') == 'Y':
            bag_proc = start_bag_recording(test_node)
        test_node.run_tests()

        print(f'\nTests complete. See {test_node.report_file} for full results')

        if bag_proc is not None:
            print(f'MCAP file(s) located at {test_node.bag_file}')
    except FileNotFoundError as err:
        test_node.get_logger().error(f'Failed to write report: {err}')
    except PermissionError as err:
        test_node.get_logger().error(f'Insufficient permissions to write report: {err}')
    except KeyboardInterrupt:
        test_node.get_logger().info('User aborted')
    finally:
        if bag_proc is not None:
            bag_proc.terminate()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
