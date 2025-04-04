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
#ifndef LYNX_MOTOR_DRIVER__LYNX_MOTOR_NODE_H
#define LYNX_MOTOR_DRIVER__LYNX_MOTOR_NODE_H

#include <regex>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "can_msgs/msg/frame.hpp"

#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

#include "lynx_motor_driver/lynx_motor_driver.hpp"
#include "lynx_motor_driver/message.hpp"

#include "clearpath_motor_msgs/action/lynx_calibrate.hpp"
#include "clearpath_motor_msgs/action/lynx_update.hpp"
#include "clearpath_motor_msgs/msg/lynx_multi_debug.hpp"
#include "clearpath_motor_msgs/msg/lynx_debug.hpp"
#include "clearpath_motor_msgs/msg/lynx_multi_status.hpp"
#include "clearpath_motor_msgs/msg/lynx_status.hpp"
#include "clearpath_motor_msgs/msg/lynx_multi_feedback.hpp"
#include "clearpath_motor_msgs/msg/lynx_feedback.hpp"
#include "clearpath_motor_msgs/msg/lynx_system_protection.hpp"



class LynxMotorNode
: public rclcpp::Node
{
public:
  // Constructor
  LynxMotorNode(const std::string node_name);

  // Callbacks
  void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void canRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

  // Update messages
  void updateDebug();
  void updateFeedback();
  void updateStatus();

  // Publish messages
  void publishDebug();
  void publishFeedback();
  void publishStatus();

  // Timer run functions
  void runDebug();
  void runFeedback();
  void runStatus();

private:
  using Calibrate = clearpath_motor_msgs::action::LynxCalibrate;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;
  using Update = clearpath_motor_msgs::action::LynxUpdate;
  using GoalHandleUpdate = rclcpp_action::ServerGoalHandle<Update>;
  using LynxStatus = clearpath_motor_msgs::msg::LynxStatus;
  using LynxSystemProtection = clearpath_motor_msgs::msg::LynxSystemProtection;
  using DiagnosticStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;

  // Drivers
  std::vector<lynx_motor_driver::LynxMotorDriver> drivers_;

  // Node
  rclcpp::Node::SharedPtr node_handle_;

  // CAN Interface
  std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> can_interface_;

  // Variables
  bool updating_, update_started_, debugging_;
  int feedback_hz_, status_hz_, debug_hz_;
  std::string can_bus_;
  std::vector<std::string> joint_names_;
  std::vector<int64_t> joint_can_ids_;
  std::vector<int64_t> joint_directions_;
  std::string firmware_version_expected_;
  std::string firmware_update_status_;

  // Messages
  clearpath_motor_msgs::msg::LynxMultiStatus status_msg_;
  clearpath_motor_msgs::msg::LynxMultiFeedback feedback_msg_;
  clearpath_motor_msgs::msg::LynxMultiDebug debug_msg_;
  LynxSystemProtection system_protection_msg_;

  // Action servers
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp_action::Server<Update>::SharedPtr update_action_server_;

  // Publishers
  rclcpp::Publisher<clearpath_motor_msgs::msg::LynxMultiStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<clearpath_motor_msgs::msg::LynxMultiFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<clearpath_motor_msgs::msg::LynxMultiDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<LynxSystemProtection>::SharedPtr system_protection_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::TimerBase::SharedPtr protection_timer_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  // Diagnostic Labels
  const std::map<uint8_t, std::string> STATUS_FLAG_LABELS_ = {
    {LynxStatus::STATUS_FLAG_ADC_CALIBRATED, "ADC Calibrated"},
    {LynxStatus::STATUS_FLAG_FOC_ENABLED, "FOC Enabled"},
    {LynxStatus::STATUS_FLAG_CALIBRATION_REQUESTED, "Calibration Requested"},
    {LynxStatus::STATUS_FLAG_CALIBRATION_CANCELLED, "Calibration Cancelled"},
    {LynxStatus::STATUS_FLAG_ESTOPPED, "E-stopped"},
  };

  const std::map<uint8_t, std::string> WARNING_FLAG_LABELS_ = {
    {LynxStatus::WARNING_FLAG_MOTOR_THERMISTOR, "Motor Thermistor Not Detected"},
    {LynxStatus::WARNING_FLAG_PCB_THERMISTOR, "PCB Thermistor Not Detected"},
    {LynxStatus::WARNING_FLAG_PHASE, "Phase Error"},
    {LynxStatus::WARNING_FLAG_PHASE_A, "Phase A Disconnected"},
    {LynxStatus::WARNING_FLAG_PHASE_B, "Phase B Disconnected"},
    {LynxStatus::WARNING_FLAG_PHASE_C, "Phase C Disconnected"},
    {LynxStatus::WARNING_FLAG_ENCODER_INDEX, "Encoder Index Not Detected"},
    {LynxStatus::WARNING_FLAG_ENCODER_OUTPUT_A, "Encoder Output A Error"},
    {LynxStatus::WARNING_FLAG_ENCODER_OUTPUT_B, "Encoder Output B Error"},
  };

  const std::map<uint8_t, std::string> ERROR_FLAG_LABELS_ = {
    {LynxStatus::ERROR_FLAG_NOT_CALIBRATED, "Not Calibrated"},
    {LynxStatus::ERROR_FLAG_MOTOR_FAULT, "Motor Fault"},
    {LynxStatus::ERROR_FLAG_MOTOR_STALLING, "Motor Stalling"},
    {LynxStatus::ERROR_FLAG_MOTOR_THERMISTOR, "Motor Thermistor Not Detected"},
    {LynxStatus::ERROR_FLAG_PCB_THERMISTOR, "PCB Thermistor Not Detected"},
    {LynxStatus::ERROR_FLAG_PHASE, "Phase Error"},
    {LynxStatus::ERROR_FLAG_PHASE_A, "Phase A Disconnected"},
    {LynxStatus::ERROR_FLAG_PHASE_B, "Phase B Disconnected"},
    {LynxStatus::ERROR_FLAG_PHASE_C, "Phase C Disconnected"},
    {LynxStatus::ERROR_FLAG_ENCODER_POWER, "Encoder Power Error"},
    {LynxStatus::ERROR_FLAG_ENCODER_INDEX, "Encoder Index Not Detected"},
    {LynxStatus::ERROR_FLAG_ENCODER_OUTPUT_A, "Encoder Output A Error"},
    {LynxStatus::ERROR_FLAG_ENCODER_OUTPUT_B, "Encoder Output B Error"},
  };

  const std::map<uint8_t, std::string> LYNX_PROTECTION_LABELS_ = {
    {LynxSystemProtection::NORMAL, "Normal"},
    {LynxSystemProtection::THROTTLED, "Throttled"},
    {LynxSystemProtection::OVERHEATED, "Overheated"},
    {LynxSystemProtection::ERROR, "Error"},
  };

  const std::map<uint8_t, std::string> LYNX_MOTOR_LABELS_ = {
    {LynxSystemProtection::A300_MOTOR_FRONT_LEFT, "Front Left"},
    {LynxSystemProtection::A300_MOTOR_FRONT_RIGHT, "Front Right"},
    {LynxSystemProtection::A300_MOTOR_REAR_LEFT, "Rear Left"},
    {LynxSystemProtection::A300_MOTOR_REAR_RIGHT, "Rear Right"},
  };

  // System Protection
  void updateSystemState();
  void sendSystemState();
  void publishSystemState();

  // Debugging
  void startDebug();

  // Diagnostic Tasks
  void driverDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat, int i);
  void protectionDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Calibrate action
  rclcpp_action::GoalResponse handleCalibrateGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Calibrate::Goal> goal);
  rclcpp_action::CancelResponse handleCalibrateCancel(
    const std::shared_ptr<GoalHandleCalibrate> goal_handle);
  void handleCalibrateAccepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle);
  void executeCalibrateAction(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  // Update action
  std::queue<uint8_t> readBinaryFile(const std::string filename);
  rclcpp_action::GoalResponse handleUpdateGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Update::Goal> goal);
  rclcpp_action::CancelResponse handleUpdateCancel(
    const std::shared_ptr<GoalHandleUpdate> goal_handle);
  void handleUpdateAccepted(const std::shared_ptr<GoalHandleUpdate> goal_handle);
  void executeUpdateAction(const std::shared_ptr<GoalHandleUpdate> goal_handle);
  std::string getDefaultFirmware();

  // Fimware version check
  std::string parseFirmwareVersion(std::string filename);
};

#endif // LYNX_MOTOR_DRIVER__LYNX_MOTOR_NODE_H
