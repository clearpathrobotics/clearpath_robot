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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <filesystem>
#include <queue>
#include <glob.h>

// Binary files will have 15 reserved bytes that should be ignored
static constexpr uint8_t BINARY_FILE_RESERVED_BYTE_START_INDEX = 3;
static constexpr uint8_t BINARY_FILE_RESERVED_BYTE_END_INDEX = 18;
static constexpr uint8_t ALIVE_CHECK_ATTEMPTS = 100;

/**
 * @brief Read
 * 
 * @param filename 
 * @return std::queue<uint8_t> 
 */
std::queue<uint8_t> LynxMotorNode::readBinaryFile(const std::string filename)
{
  std::ifstream file(filename, std::ios::binary);

  // Don't skip white space
  file.unsetf(std::ios::skipws);

  std::string byte, line;
  std::queue<uint8_t> bytes;
  // Index from 1
  int64_t count = 1;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    while (getline(ss, byte, ' '))
    {
      try {
        // Discard reserved bytes
        if (count < BINARY_FILE_RESERVED_BYTE_START_INDEX || count > BINARY_FILE_RESERVED_BYTE_END_INDEX)
        {
          bytes.push(static_cast<uint8_t>(std::stoi(byte, NULL, 16)));
        }
        count++;
      }
      // Ignore invalid characters
      catch (std::invalid_argument const&)
      {
        continue;
      }
    }
  }

  return bytes;
}

/**
 * @brief Handle Update goal request
 * 
 * @param uuid UUID
 * @param goal Goal pointer
 * @return rclcpp_action::GoalResponse 
 */
rclcpp_action::GoalResponse LynxMotorNode::handleUpdateGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Update::Goal> goal)
{
  (void)uuid;
  std::string filename = goal->file;
  std::queue<uint8_t> app;

  RCLCPP_INFO(this->get_logger(), "Received update request");

  // Check for custom file path
  if (!filename.empty())
  {
    std::ifstream file(filename);
    // Check that the file exists and is a .txt file
    if (!file.good() || filename.find(".txt") == std::string::npos)
    {
      // Invalid file
      RCLCPP_ERROR(this->get_logger(), "Invalid file received %s", filename.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  else // Use default binary
  {
    // Find default binary in lynx_motor_driver package
    std::filesystem::path dir(ament_index_cpp::get_package_share_directory("lynx_motor_driver"));
    std::filesystem::path folder("bin");
    std::filesystem::path file("lynx_firmware-release-*.txt");
    filename = dir / folder / file;

    // Find file name from wildcard
    glob_t globResult;
    int globStatus = glob(filename.c_str(), 0, nullptr, &globResult);

    if (globStatus == 0)
    {
      // Use first file
      filename = std::string(globResult.gl_pathv[0]);
      globfree(&globResult);
    }
    else
    {
      // Can't find default binary
      RCLCPP_ERROR(this->get_logger(), "Default binary file not found");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  
  app = readBinaryFile(filename);
  RCLCPP_INFO(this->get_logger(), "Uploading file %s len %ld", filename.c_str(), app.size());
  // Copy binary to each driver
  for (auto & driver : drivers_)
  {
    driver.copyApplication(app);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handle update goal cancelled
 * 
 * @param goal_handle Pointer to goal handle
 * @return rclcpp_action::CancelResponse 
 */
rclcpp_action::CancelResponse LynxMotorNode::handleUpdateCancel(
  const std::shared_ptr<GoalHandleUpdate> goal_handle)
{
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel update");

  updating_ = false;

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Handle update goal being accepted
 * 
 * @param goal_handle Pointer to goal handle
 */
void LynxMotorNode::handleUpdateAccepted(const std::shared_ptr<GoalHandleUpdate> goal_handle)
{
  // Set updating state
  updating_ = true;

  // Execute update action in a new thread
  std::thread{std::bind(&LynxMotorNode::executeUpdateAction, this, std::placeholders::_1), goal_handle}.detach();
}

/**
 * @brief Execute update action
 * 
 * @param goal_handle Pointer to goal handle
 */
void LynxMotorNode::executeUpdateAction(const std::shared_ptr<GoalHandleUpdate> goal_handle)
{
  rclcpp::Rate loop_rate(1);

  auto feedback = std::make_shared<Update::Feedback>();
  feedback->progress.resize(drivers_.size());

  auto result = std::make_shared<Update::Result>();
  result->success.resize(drivers_.size());

  float progress = 0.0f;
  float last_progress = 0.0f;
  uint8_t i = 0;
  uint16_t counter = 0;

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  for (auto & driver : drivers_)
  {
    driver.updateReset();

    // Send Boot request
    RCLCPP_INFO(this->get_logger(), "Send boot request to %s", driver.getJointName().c_str());
    driver.sendBootRequest();
  }

  // Allow time for Lynx to enter bootloader
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Update each driver sequentially
  for (auto & driver : drivers_)
  {
    last_progress = 0.0f;
    counter = 0;

    RCLCPP_INFO(this->get_logger(), "Send alive check to %s", driver.getJointName().c_str());
    // Wait for Lynx Alive response
    do {
      // Exit if action is cancelled
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled while updating %s", driver.getJointName().c_str());
        return;
      }

      // Send alive check
      driver.sendBootAliveCheck();

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!driver.tryGetUpdateAlive() && counter++ < ALIVE_CHECK_ATTEMPTS);

    // Lynx responded
    if (counter < ALIVE_CHECK_ATTEMPTS)
    {
      RCLCPP_INFO(this->get_logger(), "Driver %s is in bootloader", driver.getJointName().c_str());

      do {
        // Exit if action is cancelled
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled while updating %s", driver.getJointName().c_str());
          return;
        }

        // Run an iteration of the update process
        progress = driver.updateApp();

        // Report progress as feedback
        if (progress - last_progress >= 0.01f)
        {
          feedback->progress.at(i) = std::round(progress * 100);
          goal_handle->publish_feedback(feedback);
          last_progress = progress;
        }
      } while (progress < 1.0f);
      result->success.at(i) = true;
    }
    else // Lynx did not respond in time
    {
      RCLCPP_INFO(this->get_logger(), "Driver %s is unresponsive, skipping", driver.getJointName().c_str());
      result->success.at(i) = false;
    }

    // Send 100% on last feedback message
    feedback->progress.at(i) = 100.0;
    goal_handle->publish_feedback(feedback);    

    i++;
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    updating_ = false;
  }
}
