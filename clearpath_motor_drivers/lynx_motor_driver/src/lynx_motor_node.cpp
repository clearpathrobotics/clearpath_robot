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
 * @brief Construct a new LynxMotorNode object
 * 
 * @param node_name Node name
 */
LynxMotorNode::LynxMotorNode(const std::string node_name) :
 Node(node_name),
 updating_(false),
 debugging_(false)
{
  // Declare parameters
  this->declare_parameter("can_bus", "can0");
  this->declare_parameter("feedback_hz", 50);
  this->declare_parameter("status_hz", 20);
  this->declare_parameter("debug_hz", 20);
  this->declare_parameter("joint_can_ids", std::vector<int64_t>());
  this->declare_parameter("joint_directions", std::vector<int64_t>());
  this->declare_parameter("joint_names", std::vector<std::string>());

  // Get parameters
  this->get_parameter("can_bus", can_bus_);
  this->get_parameter("feedback_hz", feedback_hz_);
  this->get_parameter("status_hz", status_hz_);
  this->get_parameter("debug_hz", debug_hz_);
  joint_can_ids_ = this->get_parameter("joint_can_ids").as_integer_array();
  joint_directions_ = this->get_parameter("joint_directions").as_integer_array();
  joint_names_ = this->get_parameter("joint_names").as_string_array();

  RCLCPP_INFO(
    this->get_logger(),
    "CANBus Device %s",
    can_bus_.c_str()
  );

  for (uint8_t i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Joint %s : ID %ld Direction %ld",
      joint_names_[i].c_str(), joint_can_ids_[i], joint_directions_[i]
    );
  }

  // Validate Parameters
  if (joint_names_.size() != joint_can_ids_.size() || joint_names_.size() != joint_directions_.size())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Length of joint_name list must match length of joint_can_ids list and joint_directions list.");
    return;
  }

  if (joint_names_.size() == 0)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "At least one driver must be specified.");
    return;
  }

  // Node handle
  node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

  // CAN interface
  can_interface_ = std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface>(
    new clearpath_ros2_socketcan_interface::SocketCANInterface(
      can_bus_,
      node_handle_,
      std::bind(&LynxMotorNode::canRxCallback, this, std::placeholders::_1)));

  // Initialise drivers
  for (uint8_t i = 0; i < joint_names_.size(); i++)
  {
    drivers_.push_back(lynx_motor_driver::LynxMotorDriver(
      joint_can_ids_[i],
      joint_names_[i],
      joint_directions_[i],
      can_interface_
    ));
  }

  // Subsciber
  cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "platform/motors/cmd",
    rclcpp::SensorDataQoS(),
    std::bind(&LynxMotorNode::cmdCallback, this, std::placeholders::_1));

  // Publishers
  feedback_pub_ = this->create_publisher<clearpath_motor_msgs::msg::LynxMultiFeedback>(
    "platform/motors/feedback",
    rclcpp::SensorDataQoS());
  status_pub_ = this->create_publisher<clearpath_motor_msgs::msg::LynxMultiStatus>(
    "platform/motors/status",
    rclcpp::SensorDataQoS());
  system_protection_pub_ = this->create_publisher<clearpath_motor_msgs::msg::LynxSystemProtection>(
    "platform/motors/system_protection",
    rclcpp::SensorDataQoS());

  // Actions
  calibrate_action_server_ = rclcpp_action::create_server<Calibrate>(
    this,
    "platform/motors/calibrate",
    std::bind(&LynxMotorNode::handleCalibrateGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LynxMotorNode::handleCalibrateCancel, this, std::placeholders::_1),
    std::bind(&LynxMotorNode::handleCalibrateAccepted, this, std::placeholders::_1));
  
  update_action_server_ = rclcpp_action::create_server<Update>(
    this,
    "platform/motors/update",
    std::bind(&LynxMotorNode::handleUpdateGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LynxMotorNode::handleUpdateCancel, this, std::placeholders::_1),
    std::bind(&LynxMotorNode::handleUpdateAccepted, this, std::placeholders::_1));

  // Resize message vectors
  feedback_msg_.drivers.resize(drivers_.size());
  status_msg_.drivers.resize(drivers_.size());
  debug_msg_.drivers.resize(drivers_.size());
  system_protection_msg_.header.frame_id = "base_link";
  system_protection_msg_.motor_states.resize(drivers_.size());
  system_protection_msg_.system_state = clearpath_motor_msgs::msg::LynxSystemProtection::NORMAL;

  // Start timers
  feedback_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<uint64_t>(1000 / feedback_hz_)), std::bind(&LynxMotorNode::runFeedback, this));
  
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<uint64_t>(1000 / status_hz_)), std::bind(&LynxMotorNode::runStatus, this));
  
  debug_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<uint64_t>(1000 / debug_hz_)), std::bind(&LynxMotorNode::runDebug, this));
  
  protection_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&LynxMotorNode::sendSystemState, this));
}


/**
 * @brief Update MultiDebug message
 * 
 */
void LynxMotorNode::updateDebug()
{
  int index = 0;
  for (auto & driver : drivers_)
  {
    if (driver.debugMessageReady())
    {
      debug_msg_.drivers[index] = driver.getDebugMessage();
    }
    index++;
  }

  debug_msg_.header.stamp = this->get_clock()->now();
}

/**
 * @brief Update MultiStatus message
 * 
 */
void LynxMotorNode::updateStatus()
{
  int index = 0;
  for (auto & driver : drivers_)
  {
    if (driver.statusMessageReady())
    {
      status_msg_.drivers[index] = driver.getStatusMessage();
    }
    index++;
  }

  status_msg_.header.stamp = this->get_clock()->now();
}

/**
 * @brief Update MultiFeedback message
 * 
 */
void LynxMotorNode::updateFeedback()
{
  int index = 0;
  for (auto & driver : drivers_)
  {
    if (driver.feedbackMessageReady())
    {
      feedback_msg_.drivers[index] = driver.getFeedbackMessage();
    }
    index++;
  }

  feedback_msg_.header.stamp = this->get_clock()->now();
}

/**
 * @brief Update debug message, then publish it
 * 
 */
void LynxMotorNode::publishDebug()
{
  updateDebug();
  debug_pub_->publish(debug_msg_);
}

/**
 * @brief Update feedback message, then publish it
 * 
 */
void LynxMotorNode::publishFeedback()
{
  updateFeedback();
  feedback_pub_->publish(feedback_msg_);
}

/**
 * @brief Update status message, then publish it
 * 
 */
void LynxMotorNode::publishStatus()
{
  updateStatus();
  status_pub_->publish(status_msg_);
}

/**
 * @brief Callback to driver command subscriber.
 * Send commanded velocity to driver with matching joint name
 * 
 * @param msg 
 */
void LynxMotorNode::cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Don't send motor commands when updating
  if (updating_)
  {
    return;
  }

  for (int i = 0; i < static_cast<int>(msg->name.size()); i++)
  {
    for (auto & driver : drivers_)
    {
      if (msg->name[i] == driver.getJointName())
      {
        driver.sendVelocity(msg->velocity[i]);
        break;
      }
    }
  }
}

/**
 * @brief CAN Rx callback.
 * Process message on receive
 * 
 * @param msg 
 */
void LynxMotorNode::canRxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  lynx_motor_driver::Message recv_msg(msg);

  for (auto & driver : drivers_)
  {
    if (driver.processMessage(recv_msg))
    {
      break;
    }
  }
}

/**
 * @brief Feedback timer callback.
 * Publish feedback and update system protection state
 * 
 */
void LynxMotorNode::runFeedback()
{
  if (updating_)
  {
    return;
  }

  updateSystemState();
  publishFeedback();
}

/**
 * @brief Status timer callback.
 * Publish status
 * 
 */
void LynxMotorNode::runStatus()
{
  if (updating_)
  {
    return;
  }

  publishStatus();
}

/**
 * @brief Debug timer callback.
 * Publish debug if any of the drivers are running debug firmware.
 * 
 */
void LynxMotorNode::runDebug()
{
  if (updating_)
  {
    return;
  }

  if (!debugging_)
  {
    for (auto & driver : drivers_)
    {
      if (driver.getDebug())
      {
        startDebug();
        break;
      }
    }
  }

  if (debugging_)
  {
    publishDebug();
  }
}

/**
 * @brief Start debug publisher
 * 
 */
void LynxMotorNode::startDebug()
{
  RCLCPP_INFO(this->get_logger(), "Debugging enabled");

  debug_pub_ = this->create_publisher<clearpath_motor_msgs::msg::LynxMultiDebug>(
    "platform/motors/debug",
    rclcpp::SensorDataQoS());
  
  debugging_ = true;
}
