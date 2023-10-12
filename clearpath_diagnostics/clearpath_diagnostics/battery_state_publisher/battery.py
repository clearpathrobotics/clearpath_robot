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


class Battery:
    def __init__(self, rolling_average_period=30) -> None:
        self._msg = BatteryState()
        self._msg.header.frame_id = "battery_link"
        self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._readings: list[Power] = []
        self.rolling_average_period = rolling_average_period

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

    def linear_interpolation(self, lut: list[list], v: float) -> float:
        # Check if voltage is below minimum value
        if v <= lut[0][0]:
            return lut[0][1]

        for i in range(0, len(lut)):
            if v < lut[i][0]:
                return (v - lut[i - 1][0]) * (lut[i][1] - lut[i - 1][1]) / (
                    lut[i][0] - lut[i - 1][0]) + lut[i - 1][1]

        # Return maximum value
        return lut[len(lut) - 1][1]


class HE2613(Battery):
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

    def __init__(self) -> None:
        super().__init__()
        self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    def update(self, power_msg: Power):
        super().update(power_msg)
        self._msg.voltage = power_msg.measured_voltages[Power.JACKAL_MEASURED_BATTERY]
        self._msg.current = power_msg.measured_currents[Power.JACKAL_TOTAL_CURRENT]

        avg_voltage = 0
        for reading in self._readings:
            avg_voltage += reading.measured_voltages[Power.JACKAL_MEASURED_BATTERY]
        avg_voltage /= len(self._readings)
        self._msg.percentage = self.linear_interpolation(self.LUT, avg_voltage)
