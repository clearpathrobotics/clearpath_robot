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
"""Factory functions that build the individual launch components used by per-platform launches.

Each factory takes only the explicit context it needs (no reads from a generator object) and
returns one logical unit — a `LaunchFile`, `LaunchFile.Node`, `LaunchFile.Process`,
`LaunchFile.LaunchArg`, or a small list of such items when they are always added together or are
mutually exclusive (e.g. battery components). Whether to include a result in the final launch
file is the caller's decision.
"""
import os

from clearpath_config.platform.battery import BatteryConfig
from clearpath_config.platform.wireless import PeplinkRouter
from clearpath_generator_common.common import LaunchFile, Package


# ---------------------------------------------------------------------------
# MCU transports & helpers
# ---------------------------------------------------------------------------
def make_serial_uros_node(namespace: str, device: str = '/dev/clearpath/j100') -> LaunchFile.Node:
    """Build a micro-ROS agent node communicating over a serial device."""
    return LaunchFile.Node(
        name='micro_ros_agent',
        package='micro_ros_agent',
        executable='micro_ros_agent',
        namespace=namespace,
        arguments=['serial', '--dev', device],
    )


def make_udp_uros_node(namespace: str, port: int = 11411) -> LaunchFile.Node:
    """Build a micro-ROS agent node communicating over UDP on the given port."""
    return LaunchFile.Node(
        name='micro_ros_agent',
        package='micro_ros_agent',
        executable='micro_ros_agent',
        namespace=namespace,
        arguments=['udp4', '--port', str(port)],
    )


def make_configure_mcu(namespace: str, clearpath_config) -> LaunchFile.Process:
    """Build the one-shot process that configures the MCU's namespace and ROS domain ID."""
    return LaunchFile.Process(
        name='configure_mcu',
        cmd=[
            ['export ROS_DOMAIN_ID=0;'],
            [LaunchFile.Variable("FindExecutable(name='ros2')"),
             ' service call platform/mcu/configure',
             ' clearpath_platform_msgs/srv/ConfigureMcu',
             ' \"{{domain_id: {0},'.format(clearpath_config.system.domain_id),
             ' robot_namespace: \\\'{0}\\\'}}\"'.format(namespace)]
        ]
    )


def make_proton_launch(namespace: str, platform: str) -> LaunchFile:
    """Build the Proton firmware launch file for the given platform string."""
    return LaunchFile(
        'proton',
        package=Package('clearpath_firmware'),
        args=[
            ('namespace', namespace),
            ('platform', platform),
        ],
    )


# ---------------------------------------------------------------------------
# IMU / GPS
# ---------------------------------------------------------------------------
def make_imu_0_filter_node(namespace: str) -> LaunchFile.Node:
    """Build the Madgwick IMU filter node for the MCU's onboard IMU (imu_0)."""
    return LaunchFile.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        namespace=namespace,
        parameters=[LaunchFile.Variable('imu_filter')],
        remappings=[
            ('imu/data_raw', 'sensors/imu_0/data_raw'),
            ('imu/mag', 'sensors/imu_0/magnetic_field'),
            ('imu/data', 'sensors/imu_0/data'),
            ('/tf', 'tf'),
        ],
    )


def make_imu_0_filter_config(platform_params_path: str) -> LaunchFile.LaunchArg:
    """Build the launch argument pointing at the IMU filter YAML config file."""
    return LaunchFile.LaunchArg(
        'imu_filter',
        default_value=os.path.join(platform_params_path, 'imu_filter.yaml'),
    )


def make_nmea_driver_node(namespace: str) -> LaunchFile.Node:
    """Build the NMEA-to-NavSat topic driver node for the onboard GPS (gps_0)."""
    return LaunchFile.Node(
        package='nmea_navsat_driver',
        executable='nmea_topic_driver',
        name='nmea_topic_driver',
        namespace=namespace,
        remappings=[
            ('nmea_sentence', 'sensors/gps_0/nmea_sentence'),
            ('fix', 'sensors/gps_0/fix'),
            ('heading', 'sensors/gps_0/heading'),
            ('time_reference', 'sensors/gps_0/time_reference'),
            ('vel', 'sensors/gps_0/vel'),
        ],
    )


# ---------------------------------------------------------------------------
# Wireless
# ---------------------------------------------------------------------------
def make_wireless_watcher_node(namespace: str) -> LaunchFile.Node:
    """Build the wireless_watcher node that publishes Wi-Fi connection status."""
    return LaunchFile.Node(
        package='wireless_watcher',
        executable='wireless_watcher',
        name='wireless_watcher',
        namespace=namespace,
        parameters=[
            {
                'hz': 1.0,
                'dev': '',
                'connected_topic': 'platform/wifi_connected',
                'connection_topic': 'platform/wifi_status',
            }
        ],
        remappings=[('/diagnostics', 'diagnostics'),],
    )


def make_wireless_router_node(namespace: str, router):
    """Build the onboard Peplink router driver node, or None if no Peplink is configured."""
    if not isinstance(router, PeplinkRouter):
        return None
    return LaunchFile.Node(
        package='peplink_router_driver',
        executable='peplink_router_node',
        name='router_node',
        namespace=f'{namespace}/network/router',
        parameters=[
            {
                'ip_address': router.ip_address,
                'username': router.username,
                'password': router.password,
                'enable_gps': router.enable_gps,
                'publish_passwords': router.publish_passwords,
            }
        ],
        remappings=[('/diagnostics', 'diagnostics'),],
    )


def make_base_station_node(namespace: str, base_station):
    """Build the base station Peplink router driver node, or None if no Peplink is configured."""
    if not isinstance(base_station, PeplinkRouter):
        return None
    return LaunchFile.Node(
        package='peplink_router_driver',
        executable='peplink_router_node',
        name='base_station_node',
        namespace=f'{namespace}/network/base_station',
        parameters=[
            {
                'ip_address': base_station.ip_address,
                'username': base_station.username,
                'password': base_station.password,
                'enable_gps': base_station.enable_gps,
                'publish_passwords': base_station.publish_passwords,
            }
        ],
        remappings=[('/diagnostics', 'diagnostics'),],
    )


# ---------------------------------------------------------------------------
# Diagnostics & telemetry bridges (cluster factories return lists)
# ---------------------------------------------------------------------------
def make_diagnostics_components(namespace: str, platform_params_path: str) -> list:
    """Build the diagnostics cluster: updater & aggregator launch args plus the launch include."""
    diag_updater_params = LaunchFile.LaunchArg(
        'diagnostic_updater_params',
        default_value=os.path.join(platform_params_path, 'diagnostic_updater.yaml'),
    )
    diag_aggregator_params = LaunchFile.LaunchArg(
        'diagnostic_aggregator_params',
        default_value=os.path.join(platform_params_path, 'diagnostic_aggregator.yaml'),
    )
    diagnostics_launch = LaunchFile(
        'diagnostics',
        package=Package('clearpath_diagnostics'),
        args=[
            ('namespace', namespace),
            ('updater_parameters', LaunchFile.Variable('diagnostic_updater_params')),
            ('aggregator_parameters', LaunchFile.Variable('diagnostic_aggregator_params')),
        ],
    )
    return [diag_updater_params, diag_aggregator_params, diagnostics_launch]


def make_foxglove_components(namespace: str, platform_params_path: str) -> list:
    """Build the Foxglove bridge cluster: parameters launch arg and bridge launch include."""
    foxglove_bridge_params = LaunchFile.LaunchArg(
        'foxglove_bridge_parameters',
        default_value=os.path.join(platform_params_path, 'foxglove_bridge.yaml'),
    )
    foxglove_bridge_launch = LaunchFile(
        'foxglove_bridge',
        package=Package('clearpath_diagnostics'),
        args=[
            ('namespace', namespace),
            ('parameters', LaunchFile.Variable('foxglove_bridge_parameters')),
        ],
    )
    return [foxglove_bridge_params, foxglove_bridge_launch]


# ---------------------------------------------------------------------------
# Battery — single factory owning the BMS / estimator decision
# ---------------------------------------------------------------------------
def _make_battery_state_control(namespace: str, setup_path: str) -> LaunchFile.Node:
    """Build the battery_state_control node (always required)."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='battery_state_control',
        name='battery_state_control',
        namespace=namespace,
        arguments=['-s', setup_path],
    )


def _make_battery_state_estimator(namespace: str, setup_path: str) -> LaunchFile.Node:
    """Build the battery_state_estimator node (used only when no BMS is present)."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='battery_state_estimator',
        name='battery_state_estimator',
        namespace=namespace,
        arguments=['-s', setup_path],
    )


def _make_valence_bms(namespace: str, battery) -> LaunchFile:
    """Build the Valence BMS launch file, applying any per-battery launch argument overrides."""
    launch_args = battery.launch_args
    valence_launch_args = [
        ('robot_namespace', namespace),
        ('namespace', f'{namespace}/platform/bms'),
        ('interface', 'can1'),
        ('bms_id', '0'),
    ]
    for i in range(len(valence_launch_args)):
        key = valence_launch_args[i][0]
        if key in launch_args:
            val = launch_args[key]
            valence_launch_args[i] = (key, str(val))
    return LaunchFile(
        'bms',
        package=Package('valence_bms_driver'),
        args=valence_launch_args,
    )


def _make_inventus_bms(namespace: str, battery) -> LaunchFile:
    """Build the Inventus CANopen BMS launch file, resolving battery count from the config."""
    launch_args = battery.launch_args

    battery_count = 1
    match (battery.configuration):
        case BatteryConfig.S1P2:
            battery_count = 2
        case BatteryConfig.S1P4:
            battery_count = 4
        case BatteryConfig.S1P6:
            battery_count = 6

    inventus_launch_args = [
        ('namespace', f'{namespace}/platform/bms'),
        ('interface', 'vcan1'),
        ('battery_count', str(battery_count)),
        ('master_id', '49'),
        ('battery_0_id', '49'),
        ('battery_1_id', '50'),
        ('battery_2_id', '51'),
        ('battery_3_id', '52'),
        ('battery_4_id', '53'),
        ('battery_5_id', '54'),
    ]
    for i in range(len(inventus_launch_args)):
        key = inventus_launch_args[i][0]
        if key in launch_args:
            val = launch_args[key]
            inventus_launch_args[i] = (key, str(val))
    return LaunchFile(
        'canopen_inventus',
        filename='inventus',
        package=Package('canopen_inventus_bringup'),
        args=inventus_launch_args,
    )


def make_battery_components(namespace: str, clearpath_config, setup_path: str) -> list:
    """
    Build the full set of battery-related launch components for the robot.

    Always includes `battery_state_control`. Inspects the configured battery
    model:
      - Valence U24/U27 12XP   -> append Valence BMS launch file.
      - Inventus S_24V20_U1    -> append Inventus BMS launch file.
      - Otherwise              -> append `battery_state_estimator`.

    The mutual-exclusion rule (BMS XOR estimator) lives only here.
    """
    components = [_make_battery_state_control(namespace, setup_path)]

    battery_model = clearpath_config.platform.battery.model
    battery = clearpath_config.platform.battery

    if battery_model in [BatteryConfig.VALENCE_U24_12XP, BatteryConfig.VALENCE_U27_12XP]:
        components.append(_make_valence_bms(namespace, battery))
    elif battery_model in [BatteryConfig.S_24V20_U1]:
        components.append(_make_inventus_bms(namespace, battery))
    else:
        components.append(_make_battery_state_estimator(namespace, setup_path))

    return components


# ---------------------------------------------------------------------------
# Motor drivers & platform peripherals
# ---------------------------------------------------------------------------
def make_lighting_node(namespace: str, platform_model: str) -> LaunchFile.Node:
    """Build the lighting_node that drives the platform's status / signal lights."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='lighting_node',
        name='lighting_node',
        namespace=namespace,
        parameters=[{'platform': platform_model}],
        remappings=[('/diagnostics', 'diagnostics'),],
    )


def make_pinout_node(namespace: str, platform_model: str) -> LaunchFile.Node:
    """Build the pinout_control_node that exposes the platform's GPIO pinout interface."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='pinout_control_node',
        name='pinout_control_node',
        namespace=namespace,
        parameters=[{'platform': platform_model}],
    )


def make_sevcon_node(namespace: str) -> LaunchFile.Node:
    """Build the Sevcon traction controller node."""
    return LaunchFile.Node(
        package='sevcon_traction',
        executable='sevcon_traction_node',
        name='sevcon_traction_node',
        namespace=namespace,
        remappings=[('/diagnostics', 'diagnostics')],
    )


def make_puma_node(namespace: str, platform_params_path: str) -> LaunchFile.Node:
    """Build the multi-Puma motor driver node loaded with the platform's control.yaml."""
    return LaunchFile.Node(
        package='puma_motor_driver',
        executable='multi_puma_node',
        parameters=[os.path.join(platform_params_path, 'control.yaml')],
        name='puma_control',
        namespace=namespace,
        remappings=[('/diagnostics', 'diagnostics')],
    )


def make_lynx_node(namespace: str, platform_params_path: str) -> LaunchFile.Node:
    """Build the Lynx BLDC motor driver node loaded with the platform's control.yaml."""
    return LaunchFile.Node(
        package='lynx_motor_driver',
        executable='lynx_motor_driver',
        parameters=[os.path.join(platform_params_path, 'control.yaml')],
        name='lynx_control',
        namespace=namespace,
        remappings=[('/diagnostics', 'diagnostics'),],
    )


def make_fan_control(namespace: str, name: str = 'fan_control') -> LaunchFile.Node:
    """Build a fan_control_node; `name` lets the caller pick a platform-specific node name."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='fan_control_node',
        name=name,
        namespace=namespace,
        remappings=[('/diagnostics', 'diagnostics')],
    )


def make_sw_low_soc_cutoff(namespace: str, name: str = 'sw_low_soc_cutoff') -> LaunchFile.Node:
    """Build the sw_low_soc_cutoff_node; `name` lets the caller pick a platform-specific name."""
    return LaunchFile.Node(
        package='clearpath_hardware_interfaces',
        executable='sw_low_soc_cutoff_node',
        name=name,
        namespace=namespace,
    )


# ---------------------------------------------------------------------------
# CAN bridges
# ---------------------------------------------------------------------------
def make_can_bridges(namespace: str, clearpath_config) -> list:
    """
    Build receiver + sender launch pairs for every configured CAN bridge.

    Returns an empty list when no CAN bridges are configured.
    """
    ros2_socketcan_package = Package('clearpath_ros2_socketcan_interface')
    bridges = []
    for can_bridge in clearpath_config.platform.can_bridges.get_all():
        bridges.append(LaunchFile(
            f'{can_bridge.interface}_receiver',
            filename='receiver',
            package=ros2_socketcan_package,
            args=[
                ('namespace', namespace),
                ('interface', can_bridge.interface),
                ('from_can_bus_topic', can_bridge.topic_rx),
                ('enable_can_fd', str(can_bridge.enaled_can_fd).lower()),
                ('interval_sec', str(can_bridge.interval)),
                ('use_bus_time', str(can_bridge.use_bus_time).lower()),
                ('filters', str(can_bridge.filters)),
                ('auto_configure', str(can_bridge.auto_configure).lower()),
                ('auto_activate', str(can_bridge.auto_activate).lower()),
                ('timeout', str(can_bridge.timeout)),
                ('transition_attempts', str(can_bridge.transition_attempts)),
            ],
        ))

        bridges.append(LaunchFile(
            f'{can_bridge.interface}_sender',
            filename='sender',
            package=ros2_socketcan_package,
            args=[
                ('namespace', namespace),
                ('interface', can_bridge.interface),
                ('to_can_bus_topic', can_bridge.topic_tx),
                ('enable_can_fd', str(can_bridge.enaled_can_fd).lower()),
                ('interval_sec', str(can_bridge.interval)),
                ('auto_configure', str(can_bridge.auto_configure).lower()),
                ('auto_activate', str(can_bridge.auto_activate).lower()),
                ('timeout', str(can_bridge.timeout)),
                ('transition_attempts', str(can_bridge.transition_attempts)),
            ],
        ))
    return bridges
