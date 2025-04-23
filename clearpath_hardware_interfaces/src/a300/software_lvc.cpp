/**
Software License Agreement (BSD)
\file      software_lvc.hpp
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
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

#include "clearpath_hardware_interfaces/a300/software_lvc.hpp"


a300_slvc::SoftwareLowVoltageCutoff::SoftwareLowVoltageCutoff() : Node("software_low_voltage_cutoff")
{
  battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      BATTERY_STATE_TOPIC, 10,
      std::bind(&SoftwareLowVoltageCutoff::battery_callback, this, std::placeholders::_1));
  
  client_ = this->create_client<std_srvs::srv::Empty>("low_battery_service");
}

void a300_slvc::SoftwareLowVoltageCutoff::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (!msg->present)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Battery not present. Ignoring message.");
    return;
  }
  
  float charge_percentage = msg->percentage * 100.0;
  
  if (charge_percentage <= WARNING_THRESHOLD && charge_percentage > CRITICAL_THRESHOLD)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Battery charge low: %.2f%%", charge_percentage);
  }
  else if (charge_percentage <= CRITICAL_THRESHOLD)
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Battery critically low: %.2f%%", charge_percentage);
    this->consecutive_low_readings_++;
    
    // Gather many readings before calling the service
    if (this->consecutive_low_readings_ >= CRITICAL_READINGS_REQUIRED) 
    {
      RCLCPP_FATAL(this->get_logger(),
          "Battery critically low, calling cmd shutdown service on MCU to power off robot.");
      call_low_battery_service();
    }
  }
  else
  {
    this->consecutive_low_readings_ = 0;  // Reset counter if charge goes above critical level
  }
}

void a300_slvc::SoftwareLowVoltageCutoff::call_low_battery_service() 
{
  if (!client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Low battery service unavailable");
    return;
  }
    
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = client_->async_send_request(request);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a300_slvc::SoftwareLowVoltageCutoff>());
  rclcpp::shutdown();
  return 0;
}
