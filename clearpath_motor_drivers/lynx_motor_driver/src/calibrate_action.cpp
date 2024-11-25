/**
Software License Agreement (BSD)

\authors   Roni Kreinin <rkreinin@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

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

#include "lynx_motor_driver/lynx_motor_node.hpp"

/**
 * @brief Handle a Calibrate action goal request
 * 
 * @param uuid Goal UUID
 * @param goal Goal pointer
 * @return rclcpp_action::GoalResponse 
 */
rclcpp_action::GoalResponse LynxMotorNode::handleCalibrateGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Calibrate::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received calibration request");
  (void)uuid;
  (void)goal;

  // Reject calibration request if currently updating
  if (updating_)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot calibrate while updating");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handle a Calibrate goal cancel request
 * 
 * @param goal_handle Pointer to goal handle
 * @return rclcpp_action::CancelResponse 
 */
rclcpp_action::CancelResponse LynxMotorNode::handleCalibrateCancel(
  const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel calibration");
  (void)goal_handle;

  // Send calibration cancel
  for (auto & driver : drivers_)
  {
    driver.sendCalibrationCancel();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Handle a Calibrate goal being accepted
 * 
 * @param goal_handle Pointer to goal handle
 */
void LynxMotorNode::handleCalibrateAccepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  // Send calibration request to each driver
  for (auto & driver : drivers_)
  {
    driver.sendCalibrationRequest();
  }

  // Spin thread to handle calibration process
  std::thread{std::bind(&LynxMotorNode::executeCalibrateAction, this, std::placeholders::_1), goal_handle}.detach();
}

/**
 * @brief Execute calibration action. Runs in a separate thread.
 * 
 * @param goal_handle Pointer to goal handle
 */
void LynxMotorNode::executeCalibrateAction(const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Calibrate::Feedback>();
  feedback->iteration.resize(drivers_.size());

  auto result = std::make_shared<Calibrate::Result>();
  result->offset.resize(drivers_.size());

  float offset = 0.0f;
  uint16_t iteration = 0;
  uint8_t offset_received = 0;

  RCLCPP_INFO(this->get_logger(), "Executing calibration");

  // Loop until all drivers have finished calibrating
  while(offset_received < drivers_.size())
  {
    // Cancel requested
    if (goal_handle->is_canceling()) {
      // Result is zero, cancel action
      for (std::size_t i = result->offset.size(); i < drivers_.size(); i++)
      {
        result->offset.push_back(0.0f);
      }
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Calibration canceled");
      return;
    }

    uint8_t i = 0;

    for (auto & driver : drivers_)
    {
      // If offset received, this driver has finished calibrating
      if (driver.tryGetOffset(offset))
      {
        result->offset.at(i) = offset;
        offset_received++;
      }
      // Get iteration number, report as feedback
      else if (driver.tryGetIteration(iteration))
      {
        feedback->iteration.at(i) = iteration;
      }

      i++;
    }

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
