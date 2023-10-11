#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
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

from ament_index_python.packages import get_package_share_directory

from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        "setup_path",
        default_value="/etc/clearpath/",
        description="Clearpath setup path",
    )
]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_diagnostics = get_package_share_directory("clearpath_diagnostics")

    setup_path = LaunchConfiguration("setup_path")

    analyzer_params_filepath = PathJoinSubstitution(
        [pkg_clearpath_diagnostics, "config", "diagnostics.yaml"]
    )

    # Read robot YAML
    config = read_yaml(setup_path.perform(context) + "robot.yaml")
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    diagnostics = GroupAction(
        [
            PushRosNamespace(namespace),
            # Aggregator
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                output="screen",
                parameters=[analyzer_params_filepath],
                remappings=[
                    ("/diagnostics", "diagnostics"),
                    ("/diagnostics_agg", "diagnostics_agg"),
                    ("/diagnostics_toplevel_state", "diagnostics_toplevel_state"),
                ],
            ),
            # Updater
            Node(
                package="clearpath_diagnostics",
                executable="diagnostics_updater",
                output="screen",
                remappings=[
                    ("/diagnostics", "diagnostics"),
                    ("/diagnostics_agg", "diagnostics_agg"),
                    ("/diagnostics_toplevel_state", "diagnostics_toplevel_state"),
                ],
                arguments=['-s', setup_path]
            ),
        ]
    )

    return [diagnostics]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
