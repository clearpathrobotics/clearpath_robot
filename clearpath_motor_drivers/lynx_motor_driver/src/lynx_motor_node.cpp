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
 debugging_(false),
 updater_(this)
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

  // Setup diagnostics
  updater_.setHardwareID("Lynx");
  updater_.add("Lynx Motor Driver Summary", this, &LynxMotorNode::protectionDiagnostic);
  firmware_version_expected_ = parseFirmwareVersion(getDefaultFirmware());

  // Initialise drivers
  for (uint8_t i = 0; i < joint_names_.size(); i++)
  {
    drivers_.emplace_back(
      joint_can_ids_[i],
      joint_names_[i],
      joint_directions_[i],
      can_interface_
    );

    // Add diagnostic tasks
    std::string name = "Lynx Motor Driver " + std::to_string(i + 1) + " (" + joint_names_[i] + ")";
    updater_.add(name, std::bind(&LynxMotorNode::driverDiagnostic, this, std::placeholders::_1, i));
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

/**
 * @brief Diagnostic task to report details for each motor driver
 *
 * @param i Driver index number
 */
void LynxMotorNode::driverDiagnostic(DiagnosticStatusWrapper & stat, int i)
{
  bool display_data = false;
  if (!updating_) {  // don't run frequency status if actively updating firmware
    drivers_[i].runFreqStatus(stat);
    // Error from the frequency status means that no messages are being received
    display_data = (stat.level != DiagnosticStatusWrapper::ERROR);
  }

  // Display fimware update process diagnostics if update has been attempted this boot
  if (update_started_) {
    drivers_[i].driverUpdateDiagnostics(stat, updating_);  // Overwrites previous summary messages
    if (updating_) {
      return;  // If actively updating show nothing else
    }
  }


  if (!display_data) {
    // therefore return instead of reporting on old data
    return;
  }

  stat.add("CAN ID", (int)status_msg_.drivers[i].can_id);
  stat.add("Joint Name", status_msg_.drivers[i].joint_name);
  stat.add("Firmware Version", status_msg_.drivers[i].firmware_version);
  stat.add("Motor Temperature (C)", status_msg_.drivers[i].motor_temperature);
  stat.add("BLDC MCU Temperature (C)", status_msg_.drivers[i].mcu_temperature);
  stat.add("BLDC PCB Temperature (C)", status_msg_.drivers[i].pcb_temperature);
  stat.add("Hardware Triggered Stop", (status_msg_.drivers[i].stopped ? "True" : "False"));

  try {
    // Set message to warn if motor is in overheated or throttled states
    if ((system_protection_msg_.motor_states[i] == LynxSystemProtection::THROTTLED) ||
        (system_protection_msg_.motor_states[i] == LynxSystemProtection::OVERHEATED)) {
      stat.mergeSummary(DiagnosticStatusWrapper::WARN,
                        LYNX_PROTECTION_LABELS_.at(system_protection_msg_.motor_states[i]));
    }
  } catch(const std::out_of_range & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Unknown Lynx system protection message motor state value with no string description: %s",
      e.what());
  }

  // Status flags
  for (auto label : STATUS_FLAG_LABELS_) {
    // isolate and save the bit associated with the flag
    bool flag = (status_msg_.drivers[i].status_flags >> label.first) & 0x1;
    stat.add(label.second, flag ? "True" : "False");
    if (flag) {
      if (label.first == LynxStatus::STATUS_FLAG_CALIBRATION_REQUESTED) {
        // Okay state message only gets added if all previous messages were okay
        stat.mergeSummary(DiagnosticStatusWrapper::OK, label.second);
      }
      else if (label.first == LynxStatus::STATUS_FLAG_ESTOPPED){
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, label.second);
      }
    }
  }

  // Warning flags
  for (auto label : WARNING_FLAG_LABELS_) {
    // isolate and save the bit associated with the flag
    bool flag = (status_msg_.drivers[i].warning_flags >> label.first) & 0x1;
    stat.add(label.second, flag ? "True" : "False");
    if (flag) {
      // Flag any active warnings in the summary and set level to warning
      stat.mergeSummary(DiagnosticStatusWrapper::WARN, label.second);
    }
  }

  // Error flags
  for (auto label : ERROR_FLAG_LABELS_) {
    bool flag = (status_msg_.drivers[i].error_flags >> label.first) & 0x1;
    stat.add(label.second, flag ? "True" : "False");
    if (flag) {
      // Flag any active errors in the summary and set level to error
      stat.mergeSummary(DiagnosticStatusWrapper::ERROR, label.second);
    }
  }

  std::string firmware_current = status_msg_.drivers[i].firmware_version;
  size_t pos = status_msg_.drivers[i].firmware_version.find(' ');
  if (pos != std::string::npos) {
    firmware_current = status_msg_.drivers[i].firmware_version.substr(0, pos);
  }

  // Firmware check
  if (firmware_version_expected_ != firmware_current) {
    stat.mergeSummaryf(DiagnosticStatusWrapper::ERROR,
                       "Firmware mismatch (\"%s\" is installed but expected \"%s\")",
                       firmware_current.c_str(),
                       firmware_version_expected_.c_str());
  }
}

/**
 * @brief Diagnostic task to report overal system protection status for all drivers
 *
 */
void LynxMotorNode::protectionDiagnostic(DiagnosticStatusWrapper & stat)
{
  // Fimware update process
  if (update_started_) {
    // Ignore everything else if actively updating the firmware, messages are not valid
    stat.summary(DiagnosticStatusWrapper::WARN, firmware_update_status_);
    return;
  }

  try {
    // Interpret system protection message
    if (system_protection_msg_.system_state == LynxSystemProtection::NORMAL) {
      stat.summary(DiagnosticStatusWrapper::OK,
                  LYNX_PROTECTION_LABELS_.at(system_protection_msg_.system_state));
    } else if (system_protection_msg_.system_state == LynxSystemProtection::ERROR) {
      stat.summary(DiagnosticStatusWrapper::ERROR,
                  LYNX_PROTECTION_LABELS_.at(system_protection_msg_.system_state));
    } else {
      stat.summary(DiagnosticStatusWrapper::WARN,
                  LYNX_PROTECTION_LABELS_.at(system_protection_msg_.system_state));
    }
  } catch(const std::out_of_range & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Unknown Lynx system protection message system state value with no string description: %s",
      e.what());
  }

  try {
    for (unsigned i = 0; i < system_protection_msg_.motor_states.size(); i++) {
      stat.add(LYNX_MOTOR_LABELS_.at(i) + " Motor State",
              LYNX_PROTECTION_LABELS_.at(system_protection_msg_.motor_states[i]));
    }
  } catch(const std::out_of_range & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Unknown Lynx system protection message motor state value with no string description: %s",
      e.what());
  }
}

std::string LynxMotorNode::parseFirmwareVersion(std::string filename)
{
  std::regex rgx("([Ll]ynx[_-][Ff]irmware[_-][Rr]elease[_-])([0-9]+[.][0-9]+[.][0-9]+)");
  std::smatch match;
  std::string version = "unknown";

  if (std::regex_search(filename, match, rgx)) {
    version = match[2]; // Capture the 2nd subgroup which contains the version number string
  }
  return version;
}