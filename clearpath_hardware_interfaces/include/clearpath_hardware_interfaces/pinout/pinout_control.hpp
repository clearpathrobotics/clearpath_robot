/**
Software License Agreement (BSD)
\file      pinout_control.hpp
\authors   Roni Kreinin <rkreinin@clearpathrobotics.com>
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

#ifndef CLEARPATH_HARDWARE_INTERFACES_PINOUT_CONTROL_HPP
#define CLEARPATH_HARDWARE_INTERFACES_PINOUT_CONTROL_HPP

#include <map>
#include <mutex>

#include <clearpath_platform_msgs/msg/pinout_command.hpp>
#include <clearpath_platform_msgs/msg/pinout_state.hpp>
#include <clearpath_platform_msgs/srv/set_pinout.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace pinout_control
{

using PinoutCommand = clearpath_platform_msgs::msg::PinoutCommand;
using PinoutState = clearpath_platform_msgs::msg::PinoutState;
using SetPinout = clearpath_platform_msgs::srv::SetPinout;
using SetBool = std_srvs::srv::SetBool;

/**
 * @brief Enum of supported platforms
 *
 */
enum Platform
{
  A300,
  D100,
  D150
};

/**
 * @brief Map of supported platform names to enum
 *
 */
static std::map<std::string, Platform> SupportedPlatforms{
  {"a300", Platform::A300},
  {"dd100", Platform::D100},
  {"do100", Platform::D100},
  {"dd150", Platform::D150},
  {"do150", Platform::D150}};

/**
 * @brief List of power rails supported by each platform
 *
 */
static std::map<Platform, std::vector<std::string>> PlatformRails{
  {Platform::A300, std::vector<std::string>{"user_pwr_ctrl"}},
  {Platform::D100, std::vector<std::string>()},
  {Platform::D150, std::vector<std::string>()}};

/**
 * @brief List of output pins supported by each platform
 *
 */
static std::map<Platform, std::vector<std::string>> PlatformOutputs{
  {Platform::A300,
   std::vector<std::string>{"aux_1", "aux_2", "aux_3", "gpo_0", "gpo_1", "gpo_2", "gpo_3"}},
  {Platform::D100, std::vector<std::string>{"aux_1", "aux_2", "aux_3"}},
  {Platform::D150, std::vector<std::string>{"aux_1", "aux_2", "aux_3"}}};

/**
 * @brief Pinout Control Node
 *
 */
class PinoutControlNode : public rclcpp::Node
{
public:
  PinoutControlNode();

  // Output toggle period can be at most 31 bits
  static constexpr uint32_t OUTPUT_TOGGLE_PERIOD_MASK = 0x7FFFFFFF;
  // Minimum toggle period in milliseconds
  static constexpr uint32_t OUTPUT_TOGGLE_PERIOD_MIN_MS = 100;

private:
  Platform platform_;
  std::vector<std::string> rails_;
  std::vector<std::string> outputs_;

  PinoutCommand pinout_command_msg_;
  PinoutState pinout_state_msg_;
  std::shared_ptr<std::promise<PinoutState>> pinout_state_promise_;
  std::mutex pinout_state_promise_mutex_;

  rclcpp::Publisher<PinoutCommand>::SharedPtr pinout_command_pub_;
  rclcpp::Subscription<PinoutState>::SharedPtr pinout_state_sub_;
  std::vector<rclcpp::Service<SetBool>::SharedPtr> rail_services_;
  std::vector<rclcpp::Service<SetPinout>::SharedPtr> output_services_;

  void pinoutCommandPublish();

  void pinoutStateSubscriberCallback(const PinoutState::SharedPtr msg);

  void railServiceCallback(
    int index, const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response);

  void outputServiceCallback(
    int index, const std::shared_ptr<SetPinout::Request> request,
    std::shared_ptr<SetPinout::Response> response);
};

}  // namespace pinout_control

#endif  // CLEARPATH_HARDWARE_INTERFACES_PINOUT_CONTROL_HPP
