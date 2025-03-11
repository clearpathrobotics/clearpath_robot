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
#ifndef LYNX_MOTOR_DRIVER__LYNX_MOTOR_DRIVER_H
#define LYNX_MOTOR_DRIVER__LYNX_MOTOR_DRIVER_H

#include <stdint.h>
#include <string>
#include <mutex>

#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"
#include "diagnostic_updater/update_functions.hpp"

#include "lynx_motor_driver/message.hpp"

#include "clearpath_motor_msgs/msg/lynx_debug.hpp"
#include "clearpath_motor_msgs/msg/lynx_feedback.hpp"
#include "clearpath_motor_msgs/msg/lynx_status.hpp"
#include "clearpath_motor_msgs/msg/lynx_system_protection.hpp"
#include <memory>

// Used to smooth out voltage/current/velocity data for diagnostics since it is infrequently updated
#define DIAGNOSTICS_LOW_PASS 0.9

namespace lynx_motor_driver
{

namespace Feedback
{
  typedef enum
  {
    Current,
    Voltage,
    Velocity,
    Count
  } Fields;
}

namespace Status
{
  typedef enum
  {
    Version,
    MotorTemperature,
    McuTemperature,
    PcbTemperature,
    FlagsStatus,
    FlagsWarning,
    FlagsError,
    Count
  } Fields;
}

namespace Debug
{
  typedef enum
  {
    FaultFrequency,
    FlagsAccumulator,
    AccumulatorAboveRated,
    Accumulator1_1,
    Accumulator1_5,
    Accumulator2_0,
    Accumulator2_5,
    Accumulator3_0,
    CurrentIDFB,
    CurrentIDFF,
    CurrentIDREF,
    CurrentIQFB,
    CurrentIQFF,
    CurrentIQREF,
    Count
  } Fields;
}

namespace Action
{
  typedef enum
  {
    CalibrationIteration,
    CalibrationOffset,
    UpdateAlive,
    UpdateAck,
    Count
  } Fields;
}

class LynxMotorDriver
{
public:
  // Constructor
  explicit LynxMotorDriver(
    const int64_t& can_id,
    const std::string& joint_name,
    const int64_t direction,
    std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> can_interface);

  // Process CAN message
  bool processMessage(const Message& received_msg);

  // Getters
  std::string getJointName() const { return joint_name_; }
  int64_t getCanID() const { return can_id_; }
  bool getDebug() const { return debug_; }
  uint8_t getProtectionState();

  // Message getters
  bool debugMessageReady();
  bool feedbackMessageReady();
  bool statusMessageReady();

  clearpath_motor_msgs::msg::LynxDebug getDebugMessage();
  clearpath_motor_msgs::msg::LynxFeedback getFeedbackMessage();
  clearpath_motor_msgs::msg::LynxStatus getStatusMessage();

  // Commands
  void sendVelocity(double velocity);
  void sendProtectionState(uint8_t state);
  void sendCalibrationRequest();
  void sendCalibrationCancel();
  void sendBootRequest();
  void sendBootAliveCheck();

  // Action getters
  bool tryGetIteration(uint16_t & iteration);
  bool tryGetOffset(float & offset);
  bool tryGetUpdateAck(uint16_t & can_count);
  bool tryGetUpdateAlive();
  void getUpdateAck(uint16_t & can_count);
  void getUpdateAlive();

  // Update functions
  void updateReset();
  void copyApplication(const std::queue<uint8_t> app);
  float updateApp();

  // Helpers to generate data for CAN messages.
  template <typename DataT>
  void send(const uint32_t id, const DataT value);
  void send(const uint32_t id);
  void send(const uint32_t id, uint8_t * data, uint8_t length);

  // Diagnostics
  void runFreqStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  // Driver variables
  int64_t can_id_;
  std::string joint_name_;
  int64_t direction_;
  uint8_t protection_state_;
  bool debug_;
  float offset_;
  uint16_t iteration_;
  float current_filtered, voltage_filtered, velocity_filtered;

  // CAN interface
  std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> can_interface_;

  // Messages
  clearpath_motor_msgs::msg::LynxDebug debug_msg_;
  clearpath_motor_msgs::msg::LynxFeedback feedback_msg_;
  clearpath_motor_msgs::msg::LynxStatus status_msg_;

  // Mutexes
  std::array<std::mutex *, lynx_motor_driver::Debug::Fields::Count> debug_mutexes_;
  std::array<std::mutex *, lynx_motor_driver::Feedback::Fields::Count> feedback_mutexes_;
  std::array<std::mutex *, lynx_motor_driver::Status::Fields::Count> status_mutexes_;
  std::array<std::mutex *, lynx_motor_driver::Action::Fields::Count> action_mutexes_;

  // Update application
  std::queue<uint8_t> update_app_queue_;
  uint32_t update_app_size_;
  uint32_t app_count_, can_count_;

  // Get COBID from message ID
  uint32_t getCOBID(const uint32_t id) const;

  // Frequency Status for diagnostics
  std::shared_ptr<double> can_feedback_rate_; // Shared ptr prevents copy errors of FrequencyStatus
  std::shared_ptr<diagnostic_updater::FrequencyStatus> can_feedback_freq_status_;
};

}  // namespace lynx_motor_driver

#endif  // LYNX_MOTOR_DRIVER__LYNX_MOTOR_DRIVER_H
