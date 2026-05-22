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
"""Base class and registry for per-platform launch composition.

`BasePlatformLaunch` defines the contract that each concrete platform launch subclass implements;
`PlatformLaunch` is the registry that maps platform `NAME` strings (matching
`BasePlatformConfig.NAME`) to their corresponding launch subclasses. Subclasses live under
`clearpath_generator_robot.launch.platforms` and self-register at import time.
"""
from __future__ import annotations

from clearpath_generator_robot.launch import nodes


# BasePlatformLaunch
# - base class for per-platform launch composition
# - concrete platform launches are defined as subclasses under
#   clearpath_generator_robot.launch.platforms, each setting NAME to match
#   the corresponding BasePlatformConfig.NAME (e.g. 'a200', 'j100')
# - subclasses self-register via PlatformLaunch.register(...) at import time
class BasePlatformLaunch:
    """Base class for per-platform launch composition.

    Subclasses set `NAME` to match the corresponding `BasePlatformConfig.NAME` (e.g. `'a200'`)
    and override `get_platform_components` (and optionally `get_mcu_components`) to declare the
    platform-specific launch components. Subclasses self-register against `PlatformLaunch` at
    import time.
    """

    NAME: str = ''

    # MCU defaults applied by the base get_mcu_components implementation.
    # UROS_TRANSPORT: 'udp' (ethernet micro-ROS agent), 'serial' (serial agent),
    #                 or None (no UROS components for this platform).
    # PROTON_PLATFORM: firmware platform string passed to the Proton launch
    #                  (e.g. 'core', 'd1x0', 'j100'), or None to disable Proton.
    UROS_TRANSPORT: str | None = 'udp'
    PROTON_PLATFORM: str | None = 'core'

    def __init__(
        self,
        clearpath_config,
        platform_params_path: str,
        setup_path: str,
    ) -> None:
        """Store the parsed config and path arguments required to compose launch components."""
        self.clearpath_config = clearpath_config
        self.platform_params_path = platform_params_path
        self.setup_path = setup_path

    @property
    def namespace(self) -> str:
        """ROS namespace, sourced from the clearpath config."""
        return self.clearpath_config.system.namespace

    @property
    def platform_model(self) -> str:
        """Platform model identifier, sourced from the clearpath config."""
        return self.clearpath_config.platform.get_platform_model()

    def get_mcu_components(self) -> list:
        """
        Return launch components that configure the platform MCU.

        Default implementation uses `UROS_TRANSPORT` and `PROTON_PLATFORM` class
        attributes to select the appropriate micro-ROS agent (UDP or serial) plus
        the `configure_mcu` process for the UROS protocol, or a Proton launch for
        the PROTON protocol. Subclasses override either the class attributes or
        this method when finer-grained control is required.
        """
        cfg = self.clearpath_config
        namespace = self.namespace
        mcu = cfg.platform.mcu
        components: list = []
        if mcu.protocol == mcu.UROS:
            if self.UROS_TRANSPORT == 'udp':
                components.append(nodes.make_udp_uros_node(namespace))
                components.append(nodes.make_configure_mcu(namespace, cfg))
            elif self.UROS_TRANSPORT == 'serial':
                components.append(nodes.make_serial_uros_node(namespace))
                components.append(nodes.make_configure_mcu(namespace, cfg))
        elif mcu.protocol == mcu.PROTON:
            if self.PROTON_PLATFORM is not None:
                components.append(nodes.make_proton_launch(namespace, self.PROTON_PLATFORM))
        return components

    def get_common_components(self) -> list:
        """
        Return launch components shared across all platforms.

        Default implementation assembles diagnostics, battery, optional foxglove
        bridge, optional wireless watcher / router / base station, and CAN
        bridges based on the robot configuration. Subclasses normally do not
        override this; an empty implementation is provided by
        `GenericPlatformLaunch`.
        """
        cfg = self.clearpath_config
        namespace = self.namespace
        components: list = []
        components.extend(
            nodes.make_diagnostics_components(namespace, self.platform_params_path)
        )
        components.extend(nodes.make_battery_components(namespace, cfg, self.setup_path))
        if cfg.platform.enable_foxglove_bridge:
            components.extend(
                nodes.make_foxglove_components(namespace, self.platform_params_path)
            )
        if cfg.platform.wireless.enable_wireless_watcher:
            components.append(nodes.make_wireless_watcher_node(namespace))
        router_node = nodes.make_wireless_router_node(namespace, cfg.platform.wireless.router)
        if (
            router_node is not None
            and cfg.platform.wireless.router is not None
            and cfg.platform.wireless.router.launch_enabled
        ):
            components.append(router_node)
        base_station_node = nodes.make_base_station_node(
            namespace, cfg.platform.wireless.base_station
        )
        if (
            base_station_node is not None
            and cfg.platform.wireless.base_station is not None
            and cfg.platform.wireless.base_station.launch_enabled
        ):
            components.append(base_station_node)
        components.extend(nodes.make_can_bridges(namespace, cfg))
        return components

    def get_platform_components(self) -> list:
        """
        Return launch components specific to this platform.

        Subclasses override to add platform-specific extras (e.g. IMU filter,
        lighting, motor drivers, pinout control). Default is no extras.
        """
        return []

    def get_components(self) -> list:
        """Return the full list of launch components for this platform."""
        return (
            self.get_mcu_components()
            + self.get_common_components()
            + self.get_platform_components()
        )


# PlatformLaunch
# - registry of platform launch classes
# - concrete platform launches are defined in BasePlatformLaunch subclasses
#   under clearpath_generator_robot.launch.platforms
class PlatformLaunch:
    """Registry mapping platform `NAME` strings to their concrete `BasePlatformLaunch` subclass."""

    _REGISTRY: dict = {}

    @classmethod
    def register(cls, platform_launch_cls) -> None:
        """Register a concrete BasePlatformLaunch subclass."""
        cls._REGISTRY[platform_launch_cls.NAME] = platform_launch_cls

    @classmethod
    def get(cls, name: str):
        """Return the registered BasePlatformLaunch subclass for the given name."""
        if name not in cls._REGISTRY:
            raise KeyError(
                f'No platform launch registered for "{name}". '
                f'Available: {list(cls._REGISTRY.keys())}'
            )
        return cls._REGISTRY[name]

    @classmethod
    def all_names(cls) -> list:
        """Return list of all registered platform launch names."""
        return list(cls._REGISTRY.keys())
