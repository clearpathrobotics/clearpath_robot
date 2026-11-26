/**
Software License Agreement (BSD)
\file      battery_cutoff_monitor.cpp
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

#include "clearpath_hardware_interfaces/common/software_battery_cutoff.hpp"

#include <cmath>

namespace software_battery_cutoff
{

namespace
{

constexpr std::chrono::seconds BATTERY_CONFIRMATION_TIME(30);
constexpr float SOC_WARNING_THRESHOLD = 12.0F;
constexpr float SOC_CUTOFF_THRESHOLD = 8.0F;

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

BatteryCutoffMonitor::Decision BatteryCutoffMonitor::update(
  const sensor_msgs::msg::BatteryState & battery_state,
  std::chrono::steady_clock::time_point now,
  bool use_voltage_cutoff,
  VoltageThresholds * output_thresholds)
{
  VoltageThresholds thresholds{};
  float battery_level = 0.0F;
  if (use_voltage_cutoff) {
    if (!battery_state.present || !std::isfinite(battery_state.voltage) ||
      battery_state.voltage <= 0.0F ||
      !get_voltage_thresholds(battery_state.power_supply_technology, thresholds))
    {
      warning_timing_ = false;
      cutoff_timing_ = false;
      return Decision::IGNORE;
    }

    battery_level = battery_state.voltage;
  } else {
    if (!battery_state.present || !std::isfinite(battery_state.percentage) ||
      battery_state.percentage < 0.0F || battery_state.percentage > 1.0F)
    {
      warning_timing_ = false;
      cutoff_timing_ = false;
      return Decision::IGNORE;
    }

    thresholds = {SOC_WARNING_THRESHOLD, SOC_CUTOFF_THRESHOLD};
    battery_level = battery_state.percentage * 100.0F;
  }

  if (output_thresholds != nullptr) {
    *output_thresholds = thresholds;
  }

  if (battery_level <= thresholds.warning) {
    if (!warning_timing_) {
      warning_timing_ = true;
      warning_since_ = now;
    }

    if (battery_level <= thresholds.cutoff) {
      if (!cutoff_timing_) {
        cutoff_timing_ = true;
        cutoff_since_ = now;
      }
    } else {
      cutoff_timing_ = false;
    }

    if (cutoff_timing_ && now - cutoff_since_ >= BATTERY_CONFIRMATION_TIME) {
      return Decision::SHUTDOWN;
    }

    if (now - warning_since_ >= BATTERY_CONFIRMATION_TIME) {
      return Decision::WARNING;
    }

    return battery_level <= thresholds.cutoff ? Decision::LOW : Decision::NORMAL;
  }

  warning_timing_ = false;
  cutoff_timing_ = false;
  return Decision::NORMAL;
}

}  // namespace software_battery_cutoff
