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

from clearpath_platform_msgs.msg import Power
from sensor_msgs.msg import BatteryState

from clearpath_config.common.types.platform import Platform

from math import nan


# Base Battery

class Battery:
    """ Base Battery class. """

    class Configuration():
        """Battery configuration. Represents number of battery cells in series and parallel."""

        S1P1 = 'S1P1'
        S2P1 = 'S2P1'
        S1P3 = 'S1P3'
        S1P4 = 'S1P4'
        S4P1 = 'S4P1'
        S4P3 = 'S4P3'

        CELL_COUNT = {
            S1P1: 1,
            S2P1: 2,
            S1P3: 3,
            S1P4: 4,
            S4P1: 4,
            S4P3: 12,
        }

    # To be defined in child class
    VALID_CONFIGURATIONS = []
    SYSTEM_CAPACITY = {}
    LUT = []

    def __init__(
        self,
        platform: Platform,
        configuration: Configuration,
        rolling_average_period=30,
    ) -> None:
        self._msg = BatteryState()
        self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._msg.present = True
        self._msg.temperature = nan
        self._readings: list[Power] = []
        self.rolling_average_period = rolling_average_period
        self.platform = platform
        self.configuration = configuration
        assert self.configuration in self.VALID_CONFIGURATIONS, (
          'Invalid Configuration\n' +
          f'Configuration {self.configuration}\n' +
          f'Battery: {self.__class__.__name__}\n' +
          f'Platform : {self.platform}'
        )

        # Which power message indices to use
        self.power_msg_voltage_index: int = 0
        self.power_msg_current_index: int | list[int] = 0
        match self.platform:
            case Platform.J100:
                self.power_msg_voltage_index = Power.JACKAL_MEASURED_BATTERY
                self.power_msg_current_index = Power.JACKAL_TOTAL_CURRENT
            case Platform.A200:
                self.power_msg_voltage_index = Power.A200_BATTERY_VOLTAGE
                self.power_msg_current_index = [
                    Power.A200_MCU_AND_USER_PORT_CURRENT,
                    Power.A200_LEFT_DRIVER_CURRENT,
                    Power.A200_RIGHT_DRIVER_CURRENT,
                ]
            case Platform.J100:
                self.power_msg_voltage_index = Power.WARTHOG_MEASURED_BATTERY
                self.power_msg_current_index = Power.WARTHOG_TOTAL_CURRENT

        # System capacity
        self._msg.capacity = self._msg.design_capacity = self.SYSTEM_CAPACITY[self.configuration]

    @property
    def msg(self) -> BatteryState:
        return self._msg

    def update(self, power_msg: Power):
        # Add new reading to rolling average
        self._readings.append(power_msg)
        self._msg.header = power_msg.header
        # Remove oldest reading
        if len(self._readings) > self.rolling_average_period:
            self._readings.pop(0)

        # Set battery voltage
        self._msg.voltage = power_msg.measured_voltages[self.power_msg_voltage_index]

        # Set battery current
        if isinstance(self.power_msg_current_index, int):
            self._msg.current = power_msg.measured_currents[
                self.power_msg_current_index
            ]
        elif isinstance(self.power_msg_current_index, list):
            self._msg.current = 0.0
            for i in self.power_msg_current_index:
                self._msg.current += power_msg.measured_currents[i]
        # Cells
        self.update_cells()

    def linear_interpolation(self, lut: list[list], v: float) -> float:
        # Check if voltage is below minimum value
        if v <= lut[0][0]:
            return lut[0][1]

        for i in range(0, len(lut)):
            if v < lut[i][0]:
                return (v - lut[i - 1][0]) * (lut[i][1] - lut[i - 1][1]) / (
                    lut[i][0] - lut[i - 1][0]
                ) + lut[i - 1][1]

        # Return maximum value
        return lut[len(lut) - 1][1]

    def update_from_lut(self):
        # Calculate state of charge
        avg_voltage = 0
        for reading in self._readings:
            avg_voltage += reading.measured_voltages[self.power_msg_voltage_index]
        avg_voltage /= len(self._readings)
        self._msg.percentage = self.linear_interpolation(self.LUT, avg_voltage)
        self._msg.charge = self._msg.capacity * self._msg.percentage

        # Power supply status
        if self._msg.percentage == 1.0:
            self._msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL

    def update_cells(self):
        self._msg.cell_voltage = [nan] * Battery.Configuration.CELL_COUNT[
            self.configuration]
        self._msg.cell_temperature = [nan] * Battery.Configuration.CELL_COUNT[
            self.configuration]


# Battery Types

class LiION(Battery):
    """Base Lithium ION battery."""
    LUT = []

    def __init__(
        self,
        platform: Platform,
        configuration: Battery.Configuration,
        rolling_average_period=30,
    ) -> None:
        super().__init__(platform, configuration, rolling_average_period)
        self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    def update(self, power_msg: Power):
        super().update(power_msg)
        self.update_from_lut()


class SLA(Battery):
    """Base Lead Acid battery."""

    # Rough estimate of 12V Lead-Acid SoC from voltage,
    # scaled such that 11.6V is considered discharged.
    # From https://iopscience.iop.org/article/10.1088/1742-6596/1367/1/012077/pdf
    LUT = [
        [11.6,  0.0],
        [11.7,  0.1],
        [11.9,  0.2],
        [12.0,  0.3],
        [12.2,  0.4],
        [12.3,  0.5],
        [12.4,  0.6],
        [12.5,  0.7],
        [12.55, 0.8],
        [12.6,  0.90],
        [12.65, 0.95],
        [12.7,  1.00],
    ]

    def update(self, power_msg: Power):
        super().update(power_msg)
        self.update_from_lut()


class LiFEPO4(Battery):
    """Base LiFEPO4 battery."""
    def __init__(self,
                 platform: Platform,
                 configuration: Battery.Configuration,
                 rolling_average_period=30) -> None:
        super().__init__(platform, configuration, rolling_average_period)
        self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE

    def update(self, power_msg: Power):
        super().update(power_msg)
        # Get BMS data
        pass


# Batteries

class HE2613(LiION):
    LUT = [
        [21.00, 0.0],
        [21.84, 0.1],
        [22.68, 0.2],
        [23.52, 0.3],
        [24.36, 0.4],
        [25.20, 0.5],
        [26.04, 0.6],
        [26.88, 0.7],
        [27.72, 0.8],
        [28.56, 0.9],
        [29.40, 1.0],
    ]

    VALID_CONFIGURATIONS = [
        Battery.Configuration.S1P1,
        Battery.Configuration.S1P3,
        Battery.Configuration.S1P4,
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S1P1: 12.8,
        Battery.Configuration.S1P3: 38.4,
        Battery.Configuration.S1P4: 51.2,
    }


class ES20_12C(SLA):
    VALID_CONFIGURATIONS = [
        Battery.Configuration.S2P1
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S2P1: 20.0,
    }


class U1_35(SLA):
    VALID_CONFIGURATIONS = [
        Battery.Configuration.S4P3
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S4P3: 105,
    }


class NEC_ALM12V35(LiFEPO4):
    VALID_CONFIGURATIONS = [
        Battery.Configuration.S4P3
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S4P3: 105,
    }


class VALENCE_U24_12XP(LiFEPO4):
    VALID_CONFIGURATIONS = [
        Battery.Configuration.S4P1
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S4P1: 118,
    }


class VALENCE_U27_12XP(LiFEPO4):
    VALID_CONFIGURATIONS = [
        Battery.Configuration.S4P1
    ]

    SYSTEM_CAPACITY = {
        Battery.Configuration.S4P1: 144,
    }
