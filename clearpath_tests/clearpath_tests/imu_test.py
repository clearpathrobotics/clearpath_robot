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

from clearpath_tests.test_node import (
    ClearpathTestNode,
    ClearpathTestResult,
)
from clearpath_tests.tf import ConfigurableTransformListener
from clearpath_tests.timer import Timeout

from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class ImuTestNode(ClearpathTestNode):
    """
    Check that the IMU is publishing, EKF is active, and the IMU orientation is sane.

    Will fail of any of the above is not correct
    """

    def __init__(self, imu_num=0, setup_path='/etc/clearpath'):
        super().__init__(f'IMU (imu_{imu_num})', f'imu_{imu_num}_test', setup_path)
        self.test_in_progress = False
        self.record_data = False
        self.accel_samples = []
        self.gyro_samples = []

        self.base_link = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = ConfigurableTransformListener(
            self.tf_buffer,
            self,
            tf_topic=f'/{self.clearpath_config.get_namespace()}/tf',
            tf_static_topic=f'/{self.clearpath_config.get_namespace()}/tf_static'
        )

    def imu_raw_callback(self, imu_data: Imu):
        imu_frame = imu_data.header.frame_id

        try:
            transformation = self.tf_buffer.lookup_transform(
                imu_frame,
                self.base_link,
                rclpy.time.Time()
            )
        except TransformException as err:
            self.get_logger().warning(f'TF Lookup failure: {err}')
            return

        accel_vector = Vector3Stamped()
        accel_vector.header = imu_data.header
        accel_vector.vector.x = imu_data.linear_acceleration.x
        accel_vector.vector.y = imu_data.linear_acceleration.y
        accel_vector.vector.z = imu_data.linear_acceleration.z
        transformed_accel = do_transform_vector3(accel_vector, transformation)

        gyro_vector = Vector3Stamped()
        gyro_vector.header = imu_data.header
        gyro_vector.vector.x = imu_data.angular_velocity.x
        gyro_vector.vector.y = imu_data.angular_velocity.y
        gyro_vector.vector.z = imu_data.angular_velocity.z
        transformed_gyro = do_transform_vector3(gyro_vector, transformation)

        if not self.test_in_progress:
            self.get_logger().info(f'a ({transformed_accel.vector.x}, {transformed_accel.vector.y}, {transformed_accel.vector.z})')  # noqa: E501
            self.get_logger().info(f'g ({transformed_gyro.vector.x}, {transformed_gyro.vector.y}, {transformed_gyro.vector.z})')  # noqa: E501
            self.get_logger().info('---')

        if self.record_data:
            self.accel_samples.append(transformed_accel)
            self.gyro_samples.append(transformed_gyro)

    def run_test(self):
        def gather_samples():
            print('Gathering 10s worth of IMU data...')
            timeout = Timeout(self, 10.0)
            self.record_data = True
            while not timeout.elapsed:
                rclpy.spin_once(self, timeout_sec=1.0)
            self.record_data = False

        self.record_data = False
        self.test_in_progress = True

        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.namespace}/sensors/imu_0/data_raw',
            self.imu_raw_callback,
            qos_profile=qos_profile_sensor_data,
        )

        results = []

        user_response = self.promptYN('Ensure the robot is on the ground and level.\nOK to proceeed?')  # noqa: E501
        if not user_response == 'Y':
            results.append(ClearpathTestResult(
                None,
                f'{self.test_name} (level)',
                'User skipped'
            ))
        else:
            gather_samples()
            new_results = self.check_gravity('level', 0, 0)
            for r in new_results:
                results.append(r)
            self.accel_samples.clear()
            self.gyro_samples.clear()

        user_response = self.promptYN('Raise the REAR of the robot by 20 degrees.\nOK to proceeed?')  # noqa: E501
        if not user_response == 'Y':
            results.append(ClearpathTestResult(
                None,
                f'{self.test_name} (rear raised)',
                'User skipped'
            ))
        else:
            gather_samples()
            new_results = self.check_gravity('rear raised', math.radians(-20), 0)
            for r in new_results:
                results.append(r)
            self.accel_samples.clear()
            self.gyro_samples.clear()

        user_response = self.promptYN('Raise the LEFT of the robot by 20 degrees.\nOK to proceeed?')  # noqa: E501
        if not user_response == 'Y':
            results.append(ClearpathTestResult(
                None,
                f'{self.test_name} (left raised)',
                'User skipped'
            ))
        else:
            gather_samples()
            new_results = self.check_gravity('left raised', 0, math.radians(20))
            for r in new_results:
                results.append(r)
            self.accel_samples.clear()
            self.gyro_samples.clear()

        return results

    def check_gravity(self, label, x_angle=0.0, y_angle=0.0) -> ClearpathTestResult:
        """
        Analyse the accelerometer data and make sure gravity is properly oriented.

        Only x_angle or y_angle should be non-zero.

        @param label
        @param x_angle  The robot's front/back inclination
        @param y_angle  The robot's left/right inclination

        @return A list of ClearpathTestResults indicating if gravity is OK
        """
        if len(self.accel_samples) < 10:
            return ClearpathTestResult(
                False,
                f'{self.test_name} ({label})',
                f'{len(self.accel_samples)} samples collected; is IMU publishing at the right rate?',  # noqa: E501
            )

        results = []

        g = 9.807

        avg_x = 0
        avg_y = 0
        avg_z = 0
        for sample in self.accel_samples:
            avg_x += sample.vector.x
            avg_y += sample.vector.y
            avg_z += sample.vector.z
        avg_x /= len(self.accel_samples)
        avg_y /= len(self.accel_samples)
        avg_z /= len(self.accel_samples)

        # ensure the magnitude of the vector is about g
        measured_g = math.sqrt(avg_x ** 2 + avg_y ** 2 + avg_z ** 2)
        g_err = min(measured_g, g) / max(measured_g, g)
        results.append(ClearpathTestResult(
            g_err > 0.75,
            f'{self.test_name} {label} (g magnitude)',
            f'Measured gravity: {measured_g:0.2f}m/s^2. Accuracy {g_err:0.2f}',
        ))

        # estimate our actual inclination based on the IMU data
        angle_slop = 10 * math.pi / 180.0  # allow +/- 10 degree measurement error
        calculated_inclination_x = math.asin(avg_x / measured_g)
        calculated_inclination_y = math.asin(avg_y / measured_g)

        if x_angle != 0:
            results.append(ClearpathTestResult(
                (
                    x_angle - angle_slop <= calculated_inclination_x
                    and calculated_inclination_x <= x_angle + angle_slop
                ),
                f'{self.test_name} {label}',
                f'Measured inclination: {calculated_inclination_x * 180.0 / math.pi:0.2f}. Expected: {x_angle * 180 / math.pi:0.2f}',  # noqa:E501
            ))

        if y_angle != 0:
            results.append(ClearpathTestResult(
                (
                    y_angle - angle_slop <= calculated_inclination_y
                    and calculated_inclination_y <= y_angle + angle_slop
                ),
                f'{self.test_name} {label}',
                f'Measured inclination: {calculated_inclination_y * 180.0 / math.pi:0.2f}. Expected: {y_angle * 180 / math.pi:0.2f}',  # noqa:E501
            ))

        return results
