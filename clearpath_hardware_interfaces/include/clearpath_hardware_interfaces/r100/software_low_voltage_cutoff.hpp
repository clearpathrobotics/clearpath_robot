/**
Software License Agreement (BSD)
\file      software_low_voltage_cutoff.hpp
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

#ifndef CLEARPATH_HARDWARE_INTERFACES_R100_SOFTWARE_LOW_VOLTAGE_CUTOFF_HPP_
#define CLEARPATH_HARDWARE_INTERFACES_R100_SOFTWARE_LOW_VOLTAGE_CUTOFF_HPP_

#include <chrono>
#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace r100_sw_low_voltage_cutoff
{

struct VoltageThresholds
{
  float warning;
  float cutoff;
};

bool get_voltage_thresholds(uint8_t technology, VoltageThresholds & thresholds);

class BatteryVoltageMonitor
{
public:
  enum class Decision
  {
    IGNORE,
    NORMAL,
    WARNING,
    LOW,
    SHUTDOWN
  };

  Decision update(
    const sensor_msgs::msg::BatteryState & battery_state,
    std::chrono::steady_clock::time_point now,
    VoltageThresholds * thresholds = nullptr);

private:
  bool warning_voltage_timing_ = false;
  std::chrono::steady_clock::time_point warning_voltage_since_;
  bool cutoff_voltage_timing_ = false;
  std::chrono::steady_clock::time_point cutoff_voltage_since_;
};

class SoftwareLowVoltageCutoff : public rclcpp::Node
{
public:
  SoftwareLowVoltageCutoff();

private:
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void request_shutdown();

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client_;
  BatteryVoltageMonitor battery_monitor_;
  bool shutdown_request_pending_ = false;
  bool shutdown_requested_ = false;
};

}  // namespace r100_sw_low_voltage_cutoff

#endif  // CLEARPATH_HARDWARE_INTERFACES_R100_SOFTWARE_LOW_VOLTAGE_CUTOFF_HPP_
