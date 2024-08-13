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
from math import nan

from clearpath_config.common.types.platform import Platform
from clearpath_config.platform.battery import BatteryConfig
from clearpath_platform_msgs.msg import Power
from sensor_msgs.msg import BatteryState


# Base Battery
class Battery:
    class BaseBattery:
        """Base Battery class."""

        """Battery configuration. Represents number of battery cells in series and parallel."""
        CONFIGURATIONS = {
            BatteryConfig.S1P1: (1, 1),
            BatteryConfig.S1P2: (1, 2),
            BatteryConfig.S1P3: (1, 3),
            BatteryConfig.S1P4: (1, 4),
            BatteryConfig.S2P1: (2, 1),
            BatteryConfig.S4P1: (4, 1),
            BatteryConfig.S4P3: (4, 3),
        }

        # To be defined in child class
        CAPACITY = 0.0
        VOLTAGE = 0.0
        LUT = []

        def __init__(
            self,
            platform: Platform,
            configuration: str,
            rolling_average_period=30,
        ) -> None:
            self._rolling_average_period = rolling_average_period
            self._platform = platform
            self._configuration = self.CONFIGURATIONS[configuration]
            self._msg = BatteryState()
            self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            self._msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            self._msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            self._msg.present = True
            self._msg.temperature = nan
            self._readings: list[Power] = []

            # Which power message indices to use
            self.power_msg_voltage_index: int = 0
            self.power_msg_current_index: int | list[int] = 0
            match self._platform:
                case Platform.J100:
                    self.power_msg_voltage_index = Power.J100_MEASURED_BATTERY
                    self.power_msg_current_index = Power.J100_TOTAL_CURRENT
                case Platform.A200:
                    self.power_msg_voltage_index = Power.A200_BATTERY_VOLTAGE
                    self.power_msg_current_index = [
                        Power.A200_MCU_AND_USER_PORT_CURRENT,
                        Power.A200_LEFT_DRIVER_CURRENT,
                        Power.A200_RIGHT_DRIVER_CURRENT,
                    ]
                case Platform.W200:
                    self.power_msg_voltage_index = Power.W200_MEASURED_BATTERY
                    self.power_msg_current_index = Power.W200_TOTAL_CURRENT
                case Platform.DD100 | Platform.DO100:
                    self.power_msg_voltage_index = Power.D100_MEASURED_BATTERY
                    self.power_msg_current_index = Power.D100_TOTAL_CURRENT
                case Platform.DD150 | Platform.DO150:
                    self.power_msg_voltage_index = Power.D150_MEASURED_BATTERY
                    self.power_msg_current_index = Power.D150_TOTAL_CURRENT
                case Platform.R100:
                    self.power_msg_voltage_index = Power.R100_MEASURED_BATTERY
                    self.power_msg_current_index = Power.R100_TOTAL_CURRENT

            # System capacity
            self._msg.capacity = self._msg.design_capacity = self.system_capacity

        @property
        def msg(self) -> BatteryState:
            return self._msg

        @property
        def series(self) -> int:
            return self._configuration[0]

        @property
        def parallel(self) -> int:
            return self._configuration[1]

        @property
        def cell_count(self) -> int:
            return self.series * self.parallel

        @property
        def system_capacity(self) -> float:
            return self.CAPACITY * self.parallel

        @property
        def system_voltage(self) -> float:
            return self.VOLTAGE * self.series

        def update(self, power_msg: Power):
            # Add new reading to rolling average
            self._readings.append(power_msg)
            self._msg.header = power_msg.header
            # Remove oldest reading
            if len(self._readings) > self._rolling_average_period:
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
            # Set charging status
            if power_msg.charger_connected == 1:
                self._msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                self._msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
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

        def update_cells(self):
            self._msg.cell_voltage = [self._msg.voltage / self.series] * self.cell_count
            self._msg.cell_temperature = [nan] * self.cell_count

    # Battery Types

    class LiION(BaseBattery):
        """Base Lithium ION battery."""

        LUT = []

        def __init__(
            self,
            platform: Platform,
            configuration: str,
            rolling_average_period=30,
        ) -> None:
            super().__init__(platform, configuration, rolling_average_period)
            self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            # Multiply LUT voltage by number of batteries in series
            self.LUT = [(i * self.series, j) for (i, j) in self.LUT]

        def update(self, power_msg: Power):
            super().update(power_msg)
            self.update_from_lut()

    class SLA(BaseBattery):
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

        def __init__(self,
                     platform: Platform,
                     configuration: str,
                     rolling_average_period=30) -> None:
            super().__init__(platform, configuration, rolling_average_period)
            # Multiply LUT voltage by number of batteries in series
            self.LUT = [(i * self.series, j) for (i, j) in self.LUT]

        def update(self, power_msg: Power):
            super().update(power_msg)
            self.update_from_lut()

    # Batteries

    class HE2613(LiION):
        CAPACITY = 12.8
        VOLTAGE = 25.9
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

    class ES20_12C(SLA):
        CAPACITY = 20.0
        VOLTAGE = 12.0

    class U1_35(SLA):
        CAPACITY = 35.0
        VOLTAGE = 12.0

    class TLV1222(SLA):
        CAPACITY = 22.0
        VOLTAGE = 12.0

    class RB20(LiION):
        CAPACITY = 20.0
        VOLTAGE = 12.8
        LUT = [
            [10.5, 0.0],
            [12.0, 0.05],
            [13.0, 0.1],
            [13.25, 0.2],
            [13.3, 0.3],
            [13.4, 0.4],
            [13.45, 0.5],
            [13.5, 0.6],
            [13.6, 0.7],
            [13.7, 0.8],
            [13.8, 0.9],
            [14.5, 1.0],
        ]

    class DTM8A31(SLA):
        CAPACITY = 50.0
        VOLTAGE = 24.0
        LUT = [
            [23.2, 0.0],
            [23.4, 0.1],
            [23.8, 0.2],
            [24.0, 0.3],
            [24.4, 0.4],
            [24.6, 0.5],
            [24.8, 0.6],
            [25.0, 0.7],
            [25.1, 0.8],
            [25.2, 0.90],
            [25.3, 0.95],
            [25.4, 1.00],
        ]

    # Match battery name to class
    BATTERIES = {
        BatteryConfig.HE2613: HE2613,
        BatteryConfig.ES20_12C: ES20_12C,
        BatteryConfig.U1_35: U1_35,
        BatteryConfig.TLV1222: TLV1222,
        BatteryConfig.RB20: RB20,
        BatteryConfig.DTM8A31: DTM8A31,
    }

    def __new__(cls,
                battery: str,
                platform: Platform,
                configuration: str,
                rolling_average_period=30) -> BaseBattery:
        return Battery.BATTERIES.setdefault(battery, Battery.BaseBattery)(
            platform, configuration, rolling_average_period)
