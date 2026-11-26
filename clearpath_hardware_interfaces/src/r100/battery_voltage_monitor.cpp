/**
Software License Agreement (BSD)
\file      battery_voltage_monitor.cpp
\copyright Copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "clearpath_hardware_interfaces/r100/software_low_voltage_cutoff.hpp"

#include <cmath>

namespace r100_sw_low_voltage_cutoff
{

namespace
{

constexpr std::chrono::seconds VOLTAGE_CONFIRMATION_TIME(30);

}  // namespace

bool get_voltage_thresholds(uint8_t technology, VoltageThresholds & thresholds)
{
  if (technology == sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_VRLA) {
    thresholds = {23.0F, 22.0F};
    return true;
  }

  if (technology == sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE) {
    thresholds = {24.0F, 23.0F};
    return true;
  }

  return false;
}

BatteryVoltageMonitor::Decision BatteryVoltageMonitor::update(
  const sensor_msgs::msg::BatteryState & battery_state,
  std::chrono::steady_clock::time_point now,
  VoltageThresholds * output_thresholds)
{
  VoltageThresholds thresholds{};
  if (!battery_state.present || !std::isfinite(battery_state.voltage) ||
    battery_state.voltage <= 0.0F ||
    !get_voltage_thresholds(battery_state.power_supply_technology, thresholds))
  {
    warning_voltage_timing_ = false;
    cutoff_voltage_timing_ = false;
    return Decision::IGNORE;
  }

  if (output_thresholds != nullptr) {
    *output_thresholds = thresholds;
  }

  if (battery_state.voltage <= thresholds.warning) {
    if (!warning_voltage_timing_) {
      warning_voltage_timing_ = true;
      warning_voltage_since_ = now;
    }

    if (battery_state.voltage <= thresholds.cutoff) {
      if (!cutoff_voltage_timing_) {
        cutoff_voltage_timing_ = true;
        cutoff_voltage_since_ = now;
      }
    } else {
      cutoff_voltage_timing_ = false;
    }

    if (cutoff_voltage_timing_ && now - cutoff_voltage_since_ >= VOLTAGE_CONFIRMATION_TIME) {
      return Decision::SHUTDOWN;
    }

    if (now - warning_voltage_since_ >= VOLTAGE_CONFIRMATION_TIME) {
      return Decision::WARNING;
    }

    return battery_state.voltage <= thresholds.cutoff ? Decision::LOW : Decision::NORMAL;
  }

  warning_voltage_timing_ = false;
  cutoff_voltage_timing_ = false;
  return Decision::NORMAL;
}

}  // namespace r100_sw_low_voltage_cutoff
