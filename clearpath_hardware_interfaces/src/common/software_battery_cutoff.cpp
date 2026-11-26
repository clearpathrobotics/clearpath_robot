/**
Software License Agreement (BSD)
\file      software_battery_cutoff.cpp
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

namespace software_battery_cutoff
{

SoftwareBatteryCutoff::SoftwareBatteryCutoff() :
  Node("software_battery_cutoff")
{
  use_voltage_cutoff_ = declare_parameter<bool>("use_voltage_cutoff", false);
  battery_subscriber_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "platform/bms/state", rclcpp::SensorDataQoS(),
    std::bind(&SoftwareBatteryCutoff::battery_callback, this, std::placeholders::_1));
  shutdown_client_ = create_client<std_srvs::srv::Trigger>("platform/mcu/cmd_shutdown");
}

void SoftwareBatteryCutoff::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  VoltageThresholds thresholds{};
  const auto decision = battery_monitor_.update(
    *msg, std::chrono::steady_clock::now(), use_voltage_cutoff_, &thresholds);

  switch (decision) {
    case BatteryCutoffMonitor::Decision::IGNORE:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Battery state missing, invalid, or unsupported; battery cutoff is inactive.");
      break;
    case BatteryCutoffMonitor::Decision::NORMAL:
      break;
    case BatteryCutoffMonitor::Decision::WARNING:
      if (use_voltage_cutoff_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Battery voltage low: %.2f V (warning threshold %.1f V).",
          msg->voltage, thresholds.warning);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Battery charge low: %.2f%% (warning threshold %.1f%%).",
          msg->percentage * 100.0F, thresholds.warning);
      }
      break;
    case BatteryCutoffMonitor::Decision::LOW:
      break;
    case BatteryCutoffMonitor::Decision::SHUTDOWN:
      if (!shutdown_requested_ && !shutdown_request_pending_) {
        request_shutdown();
      }
      break;
  }
}

void SoftwareBatteryCutoff::request_shutdown()
{
  if (!shutdown_client_->service_is_ready()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "MCU shutdown service unavailable; retrying while battery voltage remains below cutoff.");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  shutdown_request_pending_ = true;
  shutdown_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      shutdown_request_pending_ = false;
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(get_logger(), "MCU rejected low-voltage shutdown: %s", response->message.c_str());
        return;
      }

      shutdown_requested_ = true;
      RCLCPP_FATAL(get_logger(), "Low-voltage cutoff reached; MCU shutdown sequence initiated.");
    });
}

}  // namespace software_battery_cutoff

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<software_battery_cutoff::SoftwareBatteryCutoff>());
  rclcpp::shutdown();
  return 0;
}
