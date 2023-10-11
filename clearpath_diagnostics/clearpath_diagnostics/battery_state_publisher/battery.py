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

from builtin_interfaces.msg import Time
from sensor_msgs.msg import BatteryState
from enum import Enum


class Battery:
    class Type(Enum):
        HE2613 = 0

    class Calculation(Enum):
        LINEAR_INTERPOLATION = 0
        BMS = 1

    # Lookup table
    LUT = {
        Type.HE2613: [
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
    }

    def __init__(self, battery_type: Type) -> None:
        self.battery_type = battery_type
        self._msg = BatteryState()
        self._msg.header.frame_id = 'battery_link'

        match self.battery_type:
            case Battery.Type.HE2613:
                self.calculation = Battery.Calculation.LINEAR_INTERPOLATION
                self.lut = Battery.LUT[Battery.Type.HE2613]
                self._msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    @property
    def msg(self) -> BatteryState:
        return self._msg

    def update(self, voltage: float, current: float):
        self._msg.voltage = voltage
        self._msg.current = current
        match self.calculation:
            case Battery.Calculation.LINEAR_INTERPOLATION:
                self._msg.percentage = self.linear_extrapolation(self.lut, voltage)
            case Battery.Calculation.BMS:
                pass

    def stamp(self, stamp: Time):
        self._msg.header.stamp = stamp

    def linear_extrapolation(self, lut: list[list], v: float) -> float:
        # Check if voltage is below minimum value
        if v <= lut[0][0]:
            return lut[0][1]

        for i in range(0, len(lut)):
            if v < lut[i][0]:
                return (v - lut[i - 1][0]) * (lut[i][1] - lut[i - 1][1]) / \
                       (lut[i][0] - lut[i - 1][0]) + lut[i - 1][1]

        # Return maximum value
        return lut[len(lut) - 1][1]
