/**
Software License Agreement (BSD)

\authors   Luis Camero <lcamero@clearpathrobotics.com>
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
#ifndef PUMA_MOTOR_DRIVER_MULTI_PUMA_NODE_H
#define PUMA_MOTOR_DRIVER_MULTI_PUMA_NODE_H

#include <map>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "clearpath_motor_msgs/msg/puma_multi_status.hpp"
#include "clearpath_motor_msgs/msg/puma_status.hpp"
#include "clearpath_motor_msgs/msg/puma_multi_feedback.hpp"
#include "clearpath_motor_msgs/msg/puma_feedback.hpp"

#include "can_hardware/common/types.hpp"
#include "can_hardware/drivers/socketcan_driver.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "puma_motor_driver/driver.hpp"
// #include "puma_motor_driver/diagnostic_updater.hpp"

namespace FeedbackBit
{
enum
{
  DutyCycle,
  Current,
  Position,
  Speed,
  Setpoint,
  Count,
};
}

namespace StatusBit
{
enum
{
  BusVoltage,
  OutVoltage,
  AnalogInput,
  Temperature,
  Mode,
  Fault,
  Count,
};
}

class MultiPumaNode
  : public rclcpp::Node
{
public:
  MultiPumaNode(const std::string node_name);

  /**
   * Receives desired motor speeds in sensor_msgs::JointState format and
   * sends commands to each motor over CAN.
  */
  void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * Checks if feedback fields have been received from each motor driver.
   * If feedback is avaiable, creates the feedback message and returns
   * true. Otherwise, returns false.
  */
  bool getFeedback();

  /**
   * Checks if status fields have been received from each motor driver.
   * If status data is available, creates the status message and returns
   * true. Otherwise, returns false.
  */
  bool getStatus();

  /**
   * If feedback message was created, publishes feedback message.
  */
  void publishFeedback();

  /**
   * If status message was created, publishes status message.
  */
  void publishStatus();

  /**
   * Checks that all motor drivers have been configured and are active.
  */
  bool areAllActive();

  /**
   * Checks if socket connection is active. If not, attempts to establish
   * a connection.
  */
  bool connectIfNotConnected();

  /**
   * @brief Callback function for processing incoming CAN frames.
   * 
   * @param frame The received CAN frame.
   */
  void frameCallback(const can_hardware::Frame& frame);

  /**
   * Main control loop that checks and maintains the socket gateway, resets
   * and reconfigures drivers that have disconnected, verifies parameters
   * are set appropriately, receives motor data, and publishes the feedback
   * and status messages.
  */
  void run();

private:
  using DiagnosticStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
  using PumaStatus = clearpath_motor_msgs::msg::PumaStatus;

  // std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface_;
  std::shared_ptr<can_hardware::drivers::SocketCanDriver> interface_;
  std::vector<puma_motor_driver::Driver> drivers_;

  bool active_;
  double gear_ratio_;
  int encoder_cpr_;
  int freq_;
  uint8_t status_count_;
  uint8_t desired_mode_;
  std::string canbus_dev_;
  std::vector<std::string> joint_names_;
  std::vector<int64_t> joint_can_ids_;
  std::vector<int64_t> joint_directions_;

  std::queue<can_hardware::Frame> recv_msg_queue_;
  std::mutex recv_msg_mutex_;

  clearpath_motor_msgs::msg::PumaMultiStatus status_msg_;
  clearpath_motor_msgs::msg::PumaMultiFeedback feedback_msg_;

  double gain_p_;
  double gain_i_;
  double gain_d_;

  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::Publisher<clearpath_motor_msgs::msg::PumaMultiStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<clearpath_motor_msgs::msg::PumaMultiFeedback>::SharedPtr feedback_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr run_timer_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  // Diagnostic labels
  const std::map<uint8_t, std::string> MODE_FLAG_LABELS_ = {
    {PumaStatus::MODE_VOLTAGE, "Voltage"},
    {PumaStatus::MODE_CURRENT, "Current"},
    {PumaStatus::MODE_SPEED, "Speed"},
    {PumaStatus::MODE_POSITION, "Position"},
    {PumaStatus::MODE_VCOMP, "V-Comp"},
  };
  const std::map<uint8_t, std::string> FAULT_FLAG_LABELS_ = {
    {PumaStatus::FAULT_CURRENT, "Current Fault"},
    {PumaStatus::FAULT_TEMPERATURE, "Temperature Fault"},
    {PumaStatus::FAULT_BUS_VOLTAGE, "Bus Voltage Fault"},
    {PumaStatus::FAULT_BRIDGE_DRIVER, "Bridge Driver Fault"},
  };
  const std::map<std::string, std::string> PUMA_MOTOR_LABELS_ = {
    {"front_left_wheel_joint", "Front Left"},
    {"front_right_wheel_joint", "Front Right"},
    {"rear_left_wheel_joint", "Rear Left"},
    {"rear_right_wheel_joint", "Rear Right"},
  };


  // Diagnostic Tasks
  void driverDiagnostic(DiagnosticStatusWrapper & stat, int i);
  void protectionDiagnostic(DiagnosticStatusWrapper & stat);
};

#endif // PUMA_MOTOR_DRIVER_PUMA_NODE_H
