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

from clearpath_generator_common.common import BaseGenerator
from clearpath_tests.mobility_test import MobilityTestNode
from clearpath_tests.test_node import ClearpathTestResult
from clearpath_tests.tf import ConfigurableTransformListener
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class LinearAccelerationTestNode(MobilityTestNode):
    """
    Accelerate & decelerate smoothly over 10s.

    The IMU should read X acceleration of +/- 0.2m/s/s
    """

    def __init__(self, imu_num=0, setup_path='/etc/clearpath'):
        super().__init__('Linear acceleration', 'linear_acceleration_test', setup_path)

        self.acceleration = self.get_parameter_or('acceleration', 0.2)
        self.acceleration_time = self.get_parameter_or('acceleration_time', 5.0)
        self.record_data = False

        self.imu_num = imu_num
        self.latest_imu = None

        self.base_link = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = ConfigurableTransformListener(
            self.tf_buffer,
            self,
            tf_topic=f'/{self.clearpath_config.get_namespace()}/tf',
            tf_static_topic=f'/{self.clearpath_config.get_namespace()}/tf_static'
        )
        self.accel_samples = []

    def imu_callback(self, imu_data):
        self.latest_imu = imu_data
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

    def start(self):
        super().start()

        # Subscribe to our default IMU
        imu_topic = f'/{self.namespace}/sensors/imu_{self.imu_num}/data'
        self.get_logger().info(f'Subscribing to IMU data on {imu_topic}...')
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def run_test(self):
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.linear.z = 0.0
        self.cmd_vel.twist.angular.x = 0.0
        self.cmd_vel.twist.angular.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0

        self.test_in_progress = True

        # 2(1/2 at^2) -- we accelerate forwards and then decelerate symmetrically
        goal_distance = 2 * 0.5 * self.acceleration * self.acceleration_time ** 2

        user_response = self.promptYN(f"""The robot will accelerate & decelerate for 10s.
Estimated travel distance is {goal_distance:0.1f}m.
Ensure there is sufficient room ahead of the robot.
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [ClearpathTestResult(False, self.test_name, 'User skipped')]

        self.get_logger().info('Starting acceleration test')
        self.start()

        # wait until we get the first IMU message or 10s passes
        start_time = self.get_clock().now()
        timeout_duration = Duration(seconds=10)
        while (
            self.latest_imu is None
            and self.get_clock().now() - start_time <= timeout_duration
            and not self.test_error
        ):
            rclpy.spin_once(self)

        if self.test_error:
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
            return self.test_results
        elif self.latest_imu is None:
            self.get_logger().warning('Timed out waiting for IMU data')
            return [ClearpathTestResult(
                False,
                self.test_name,
                'Timed out waiting for IMU data',
            )]

        # accelerate & decelerate over 5s
        accel_duration = Duration(seconds=self.acceleration_time)
        start_time = self.get_clock().now()
        while (
            not self.test_error
            and self.get_clock().now() - start_time <= accel_duration
        ):
            dt = (self.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            self.cmd_vel.twist.linear.x = self.acceleration * dt
            self.record_data = True
            rclpy.spin_once(self)
        self.record_data = False

        # smoothly decelerate, but stop don't record any more data
        start_time = self.get_clock().now()
        while (
            not self.test_error
            and self.get_clock().now() - start_time <= accel_duration
        ):
            dt = (self.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            self.cmd_vel.twist.linear.x = self.acceleration * (self.acceleration_time - dt)
            rclpy.spin_once(self)

        # stop driving
        # wait 1s to ensure we publish the command
        self.cmd_vel.twist.linear.x = 0.0
        test_wait = Duration(seconds=1)
        start_time = self.get_clock().now()
        while self.get_clock().now() - start_time <= test_wait:
            rclpy.spin_once(self)

        # process the results
        results = self.test_results
        if len(self.accel_samples) <= self.acceleration_time * 10:
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                f'Insufficient IMU data recorded ({len(self.accel_samples)}): is the IMU publishing at the correct rate?',  # noqa: E501
            ))
        else:
            avg_accel = sum(accel.vector.x for accel in self.accel_samples) / len(self.accel_samples)  # noqa: E501
            # min_accuracy = 0.8
            #
            # if self.clearpath_config.platform.get_platform_model() == Platform.J100:
            #     # default Jackal IMU is terrible, so allow wider margins
            #     min_accuracy = 0.6
            #
            # measured_accuracy = min(avg_accel, self.acceleration) / max(avg_accel, self.acceleration)  # noqa: E501
            # results.append(ClearpathTestResult(
            #     measured_accuracy >= min_accuracy,
            #     self.test_name,
            #     f'Recorded linear acceleration: {avg_accel:0.2f}m/s^2 (accuracy: {measured_accuracy:0.2f})'  # noqa: E501
            # ))
            results.append(ClearpathTestResult(
                avg_accel > 0,
                self.test_name,
                'Acceleration oriented correctly'
            ))

        return results


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        rt = LinearAccelerationTestNode(setup_path)
        rt.start()
        try:
            while not rt.test_done:
                rclpy.spin_once(rt)
            rt.get_logger().info('Test complete')
        except KeyboardInterrupt:
            rt.get_logger().info('User aborted! Cleaning up & exiting...')
        rt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
