# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
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

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
"""Franka arm launch."""
from clearpath_config.manipulators.types.arms import Franka
from clearpath_config.manipulators.types.grippers import FrankaGripper
from clearpath_generator_common.common import LaunchFile
from clearpath_generator_robot.launch.manipulator import (
    BaseManipulatorLaunch,
    ManipulatorLaunch,
)


class FrankaManipulatorLaunch(BaseManipulatorLaunch):
    """Franka arms: emits a `franka_gripper_node` when a Franka Hand is attached."""

    MANIPULATOR_MODELS = (Franka.MANIPULATOR_MODEL,)

    def get_components(self) -> list:
        """Return Franka-specific extras: gripper controller node if present."""
        if not self.arm.gripper:
            return []
        if self.arm.gripper.MANIPULATOR_MODEL != FrankaGripper.MANIPULATOR_MODEL:
            return []
        return [self._make_franka_gripper()]

    def _make_franka_gripper(self) -> LaunchFile.Node:
        """Franka Hand controller node, with platform joint-states remapping."""
        gripper = self.arm.gripper
        return LaunchFile.Node(
            name=f'{gripper.name}_controller',
            package='franka_gripper',
            executable='franka_gripper_node',
            namespace=f'{self.namespace}/manipulators',
            parameters=[{
                'robot_ip': self.arm.ip,
                'joint_names': [
                    f'{gripper.name}_{gripper.arm_id}_finger_joint1',
                    f'{gripper.name}_{gripper.arm_id}_finger_joint2',
                ],
                'state_publish_rate': 15,  # [Hz]
                'feedback_publish_rate': 30,  # [Hz]
                'default_speed': 0.1,  # [m/s]
                'default_grasp_epsilon': {
                    'inner': 0.005,  # [m]
                    'outer': 0.005,  # [m]
                },
            }],
            remappings=[
                ('~/joint_states', f'/{self.namespace}/platform/joint_states'),
            ],
        )


ManipulatorLaunch.register(FrankaManipulatorLaunch)
