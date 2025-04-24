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
from clearpath_tests.test_node import (
    ClearpathTestNode,
    ClearpathTestResult,
)
from clearpath_tests.tf import ConfigurableTransformListener

from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.duration import Duration
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

    def start(self):
        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.namespace}/sensors/imu_0/data_raw',
            self.imu_raw_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def run_test(self):
        def gather_samples():
            sample_duration = Duration(seconds=10)
            print('Gathering 10s worth of IMU data...')
            start_time = self.get_clock().now()
            self.record_data = True
            while self.get_clock().now() - start_time < sample_duration:
                rclpy.spin_once(self)
            self.record_data = False

        self.record_data = False
        self.test_in_progress = True
        self.start()

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
            results.append(self.check_gravity('level', 0, 0))
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
            results.append(self.check_gravity('rear raised', math.radians(-20), 0))
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
            results.append(self.check_gravity('left raised', 0, math.radians(20)))
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

        @return A ClearpathTestResult indicating if gravity is OK
        """
        if len(self.accel_samples) < 10:
            return ClearpathTestResult(
                False,
                f'{self.test_name} ({label})',
                f'{len(self.accel_samples)} samples collected; is IMU publishing at the right rate?',  # noqa: E501
            )

        g = 9.807
        expected_x = g * math.sin(x_angle)
        expected_y = g * math.sin(y_angle)
        if abs(x_angle) > abs(y_angle):
            expected_z = g * math.cos(x_angle)
        else:
            expected_z = g * math.cos(y_angle)

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

        # allow 20% error on the IMU since the ground may never be completely level
        # and the calibration may not be super accurate for some models
        test_tolerance = 0.2

        x_lower_limit = expected_x - g * test_tolerance
        x_upper_limit = expected_x + g * test_tolerance

        y_lower_limit = expected_y - g * test_tolerance
        y_upper_limit = expected_y + g * test_tolerance

        z_lower_limit = expected_z - g * test_tolerance
        z_upper_limit = expected_z + g * test_tolerance
        if (
            # check that the measurements are witin our error bars
            avg_x >= x_lower_limit and avg_x <= x_upper_limit and
            avg_y >= y_lower_limit and avg_y <= y_upper_limit and
            avg_z >= z_lower_limit and avg_z <= z_upper_limit and

            # ensure gravity is mainly +Z
            avg_z > 5.0 and

            # check our inclination is the right way
            (
                (avg_x > 0 and x_angle > 0) or
                (avg_y > 0 and y_angle > 0) or
                (x_angle == 0 and y_angle == 0)
            )
        ):
            return ClearpathTestResult(
                True,
                f'{self.test_name} ({label})',
                f'Measured gravity vector: ({avg_x:0.2f}, {avg_y:0.2f}, {avg_z:0.2f}) Expected: ({expected_x:0.2f}, {expected_y:0.2f}, {expected_z:0.2f})'  # noqa: E501
            )
        else:
            return ClearpathTestResult(
                False,
                f'{self.test_name} ({label})',
                f'Measured gravity vector: ({avg_x:0.2f}, {avg_y:0.2f}, {avg_z:0.2f}) Expected: ({expected_x:0.2f}, {expected_y:0.2f}, {expected_z:0.2f})'  # noqa: E501
            )


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    it = ImuTestNode(imu_num=0, setup_path=setup_path)

    try:
        it.start()
        rclpy.spin(it)
    except KeyboardInterrupt:
        pass

    it.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
