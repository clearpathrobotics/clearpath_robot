/**
Software License Agreement (BSD)
\file      software_low_soc_cutoff.hpp
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

#ifndef CLEARPATH_HARDWARE_INTERFACES_A300_SOFTWARE_LOW_SOC_CUTOFF_HPP_
#define CLEARPATH_HARDWARE_INTERFACES_A300_SOFTWARE_LOW_SOC_CUTOFF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/empty.hpp"

namespace a300_sw_low_soc_cutoff
{

/**
 * @class SoftwareLowSocCutoff
 * @brief Monitors battery state and issues warnings based on charge level.
 *
 * This node subscribes to the "battery_state" topic and checks the battery charge percentage.
 * It logs warnings at 12% and calls a service when the charge drops to 8% or below.
 */
class SoftwareLowSocCutoff : public rclcpp::Node
{
public:
  static constexpr float WARNING_THRESHOLD = 12.0; ///< Warning threshold percentage
  static constexpr float CRITICAL_THRESHOLD = 8.0; ///< Critical threshold percentage
  static constexpr int CRITICAL_READINGS_REQUIRED = 30; ///< Number of CRITICAL_THRESHOLD readings required before issuing the command


  /// Battery state topic name
  const std::string BATTERY_STATE_TOPIC = "platform/bms/state";

  /// Low SOC cuttoff service name
  const std::string SW_LOW_SOC_CUTOFF_SERVICE_NAME = "platform/mcu/cmd_shutdown";

  /**
   * @brief Constructor for SoftwareLowSocCutoff node.
   *
   * Initializes the subscriber to "battery_state" and the service client for low battery handling.
   */
  SoftwareLowSocCutoff();

private:
  /**
   * @brief Callback function for battery state messages.
   *
   * @param msg The battery state message.
   *
   * This function checks if the battery is present and evaluates its charge percentage.
   * Warnings are issued at 12%, and a critical service call is made at 8%.
   */
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  
  /**
   * @brief Calls the low battery service when charge reaches the critical level.
   *
   * Attempts to call the SW_LOW_SOC_CUTOFF_SERVICE_NAME if available.
   */
  void call_low_battery_service();

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_; ///< Battery state subscriber
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_; ///< Client for low battery service
  int consecutive_low_readings_; ///< Counter for consecutive low readings
};

}  // namespace a300_sw_low_soc_cutoff

#endif  // CLEARPATH_HARDWARE_INTERFACES_A300_SOFTWARE_LOW_SOC_CUTOFF_HPP_
