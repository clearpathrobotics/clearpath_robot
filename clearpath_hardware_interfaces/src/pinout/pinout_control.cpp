/**
Software License Agreement (BSD)
\file      pinout_control.cpp
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

#include "clearpath_hardware_interfaces/pinout/pinout_control.hpp"

using namespace pinout_control;
using PinoutCommand = clearpath_platform_msgs::msg::PinoutCommand;
using PinoutState = clearpath_platform_msgs::msg::PinoutState;
using SetPinout = clearpath_platform_msgs::srv::SetPinout;
using SetBool = std_srvs::srv::SetBool;

/**
 * @brief Construct a new Pinout Control Node object
 *
 */
PinoutControlNode::PinoutControlNode() : Node("pinout_control")
{
  // Get platform model from platform parameter
  this->declare_parameter("platform", "a300");
  std::string platform = this->get_parameter("platform").as_string();

  try
  {
    platform_ = SupportedPlatforms.at(platform);
  }
  catch (const std::out_of_range & e)
  {
    throw std::out_of_range("Unsupported Platform " + platform);
  }

  // Get vector of rails and output pins
  rails_ = PlatformRails.at(platform_);
  outputs_ = PlatformOutputs.at(platform_);

  // Resize message arrays
  pinout_command_msg_.rails.resize(rails_.size(), true);
  pinout_command_msg_.outputs.resize(outputs_.size());
  pinout_state_msg_.rails.resize(rails_.size());
  pinout_state_msg_.outputs.resize(outputs_.size());
  pinout_state_msg_.output_periods.resize(outputs_.size());

  // Create pinout command publisher
  pinout_command_pub_ =
    this->create_publisher<PinoutCommand>("platform/mcu/_cmd_pinout", rclcpp::SensorDataQoS());

  // Place subscriber in Reentrant callback group such that the callback can be executed in parallel to services
  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;

  // Create pinout state subscriber
  pinout_state_sub_ = this->create_subscription<PinoutState>(
    "platform/mcu/status/pinout", rclcpp::SensorDataQoS(),
    std::bind(&PinoutControlNode::pinoutStateSubscriberCallback, this, std::placeholders::_1),
    options);

  // Initialise rail services
  int i = 0;
  for (auto & rail : rails_)
  {
    rail_services_.push_back(this->create_service<SetBool>(
      "platform/pinout/" + rail, [this, i](
                                   const std::shared_ptr<SetBool::Request> request,
                                   std::shared_ptr<SetBool::Response> response)
      { this->railServiceCallback(i, request, response); }));
    i++;
  }

  // Initialise output services
  i = 0;
  for (auto & output : outputs_)
  {
    output_services_.push_back(this->create_service<SetPinout>(
      "platform/pinout/" + output, [this, i](
                                     const std::shared_ptr<SetPinout::Request> request,
                                     std::shared_ptr<SetPinout::Response> response)
      { this->outputServiceCallback(i, request, response); }));
    i++;
  }
}

/**
 * @brief Helper function to publish pinout command message
 *
 */
void PinoutControlNode::pinoutCommandPublish()
{
  pinout_command_pub_->publish(pinout_command_msg_);
}

/**
 * @brief Pinout state message callback.
 * Copies message and sends a copy as a promise to any pending services
 *
 * @param msg
 */
void PinoutControlNode::pinoutStateSubscriberCallback(const PinoutState::SharedPtr msg)
{
  pinout_state_msg_ = *msg;

  std::lock_guard<std::mutex> lock(pinout_state_promise_mutex_);
  if (pinout_state_promise_)
  {
    pinout_state_promise_->set_value(*msg);
    pinout_state_promise_.reset(); // Prevent repeatedly setting the value
  }
}

/**
 * @brief Callback for any rail service
 *
 * @param index Index of rail in rail list
 * @param request Request message.
 * @param response Response message.
 */
void PinoutControlNode::railServiceCallback(
  int index, const std::shared_ptr<SetBool::Request> request,
  std::shared_ptr<SetBool::Response> response)
{
  bool state = request->data;

  // Check if rail is already in requested state
  if (state == pinout_state_msg_.rails.at(index))
  {
    response->success = true;
    response->message = std::string("Rail is already ") + (std::string)(state ? "enabled" : "disabled");
  }
  else
  {
    // Change rail state and publish command
    pinout_command_msg_.rails[index] = state;
    pinoutCommandPublish();

    std::promise<PinoutState> pinout_state_promise;
    std::future<PinoutState> pinout_state_future = pinout_state_promise.get_future();

    {
      std::lock_guard<std::mutex> lock(pinout_state_promise_mutex_);
      pinout_state_promise_ =
        std::make_shared<std::promise<PinoutState>>(std::move(pinout_state_promise));
    }

    // Wait for future from pinout state subscriber
    auto status = pinout_state_future.wait_for(std::chrono::milliseconds(1000));

    if (status == std::future_status::ready)
    {
      // Pinout state received and rail state has been updated
      if (pinout_state_future.get().rails[index] == state)
      {
        response->success = true;
        response->message = rails_.at(index) + (state ? " enabled" : " disabled");
      }
      else // Pinout state received but rail state has not been updated
      {
        response->success = false;
        response->message = "Failed to command MCU";
      }
    }
    else
    {
      // Timed out waiting for topic from MCU
      response->success = false;
      response->message = "No response received from MCU";
    }
  }
}

/**
 * @brief Callback for any output service
 *
 * @param index Index of output in output list
 * @param request Request message.
 * @param response Response message.
 */
void PinoutControlNode::outputServiceCallback(
  int index, const std::shared_ptr<SetPinout::Request> request,
  std::shared_ptr<SetPinout::Response> response)
{
  bool state = request->state;
  uint32_t period = request->toggle_period & OUTPUT_TOGGLE_PERIOD_MASK;

  if (period > 0 && period < OUTPUT_TOGGLE_PERIOD_MIN_MS)
  {
    response->success = false;
    response->message = "Invalid period " + std::to_string(period) + ". Minimum period is " + std::to_string(OUTPUT_TOGGLE_PERIOD_MIN_MS);
    return;
  }

  // Check if output is already in requested state
  if (
    period == pinout_state_msg_.output_periods.at(index) && period == 0 &&
    state == pinout_state_msg_.outputs.at(index))
  {
    response->success = true;
    response->message = std::string("Output state is already ") + (std::string)(state ? "enabled" : "disabled");
  }
  else
  {
    // Change output state and period, and publish command
    pinout_command_msg_.outputs[index] = period << 1 | state;
    pinoutCommandPublish();

    std::promise<PinoutState> pinout_state_promise;
    std::future<PinoutState> pinout_state_future = pinout_state_promise.get_future();

    {
      std::lock_guard<std::mutex> lock(pinout_state_promise_mutex_);
      pinout_state_promise_ =
        std::make_shared<std::promise<PinoutState>>(std::move(pinout_state_promise));
    }

    // Wait for future from pinout state subscriber
    auto status = pinout_state_future.wait_for(std::chrono::milliseconds(1000));

    if (status == std::future_status::ready)
    {
      // Pinout state received and output state and period have been updated
      if (
        (period == 0 && pinout_state_future.get().outputs[index] == state) ||
        (period != 0 && pinout_state_future.get().output_periods[index] == period))
      {
        response->success = true;
        response->message = outputs_.at(index) + (state ? " enabled." : " disabled.") +
                            " Period set to " + std::to_string(period);
      }
      else // Pinout state received but output state or period have not been updated
      {
        response->success = false;
        response->message = "Failed to command MCU";
      }
    }
    else
    {
      // Timed out waiting for topic from MCU
      response->success = false;
      response->message = "No response received from MCU";
    }
  }
}

/**
 * @brief Executable entry point
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor required to use callback groups
  rclcpp::executors::MultiThreadedExecutor executor;

  auto pinout_control_node = std::make_shared<PinoutControlNode>();

  executor.add_node(pinout_control_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
