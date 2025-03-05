/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

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
#include "clearpath_motor_msgs/msg/puma_status.hpp"
#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

#include "puma_motor_driver/driver.hpp"

#include <string>
#include <cstring>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

namespace puma_motor_driver
{

namespace ConfigurationStates
{
enum ConfigurationState
{
  Unknown = -1,
  Initializing,
  PowerFlag,
  EncoderPosRef,
  EncoderSpdRef,
  EncoderCounts,
  ClosedLoop,
  ControlMode,
  PGain,
  IGain,
  DGain,
  VerifiedParameters,
  Configured
};
}  // namespace ConfigurationStates
typedef ConfigurationStates::ConfigurationState ConfigurationState;

Driver::Driver(
  const std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface,
  std::shared_ptr<rclcpp::Node> nh,
  const uint8_t & device_number,
  const std::string & device_name)
: interface_(interface),
  nh_(nh),
  device_number_(device_number),
  device_name_(device_name),
  configured_(false),
  state_(ConfigurationState::Initializing),
  control_mode_(clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED),
  gain_p_(1),
  gain_i_(0),
  gain_d_(0),
  encoder_cpr_(1),
  gear_ratio_(1)
{
}

void Driver::processMessage(const can_msgs::msg::Frame::SharedPtr received_msg)
{
  // If it's not our message, jump out.
  if (getDeviceNumber(*received_msg) != device_number_) {
    return;
  }

  // If there's no data then this is a request message, jump out.
  if (received_msg->dlc == 0) {
    return;
  }

  Field * field = nullptr;
  uint32_t received_api = getApi(*received_msg);
  if ((received_api & CAN_MSGID_API_M & CAN_API_MC_CFG) == CAN_API_MC_CFG) {
    field = cfgFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_STATUS) == CAN_API_MC_STATUS) {
    field = statusFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_ICTRL) == CAN_API_MC_ICTRL) {
    field = ictrlFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_POS) == CAN_API_MC_POS) {
    field = posFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_VCOMP) == CAN_API_MC_VCOMP) {
    field = vcompFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_SPD) == CAN_API_MC_SPD) {
    field = spdFieldForMessage(received_api);
  } else if ((received_api & CAN_MSGID_API_M & CAN_API_MC_VOLTAGE) == CAN_API_MC_VOLTAGE) {
    field = voltageFieldForMessage(received_api);
  }

  if (!field) {
    return;
  }

  // Copy the received data and mark that field as received.
  std::copy_n(std::begin(received_msg->data), Field::FIELD_STRUCT_DATA_SIZE, std::begin(field->data));
  field->received = true;
}

double Driver::radPerSecToRpm() const
{
  return (60 * gear_ratio_) / (2 * M_PI);
}

void Driver::sendId(const uint32_t id)
{
  can_msgs::msg::Frame msg = getMsg(id);
  interface_->queue(msg);
}

void Driver::sendUint8(const uint32_t id, const uint8_t value)
{
  can_msgs::msg::Frame msg = getMsg(id);
  msg.dlc = sizeof(uint8_t);
  uint8_t data[8] = {0};
  std::memcpy(data, &value, sizeof(uint8_t));
  std::copy(std::begin(data), std::end(data), std::begin(msg.data));

  interface_->queue(msg);
}

void Driver::sendUint16(const uint32_t id, const uint16_t value)
{
  can_msgs::msg::Frame msg = getMsg(id);
  msg.dlc = sizeof(uint16_t);
  uint8_t data[8] = {0};
  std::memcpy(data, &value, sizeof(uint16_t));
  std::copy(std::begin(data), std::end(data), std::begin(msg.data));

  interface_->queue(msg);
}

void Driver::sendFixed8x8(const uint32_t id, const float value)
{
  can_msgs::msg::Frame msg = getMsg(id);
  msg.dlc = sizeof(int16_t);
  int16_t output_value = static_cast<int16_t>(static_cast<float>(1 << 8) * value);

  uint8_t data[8] = {0};
  std::memcpy(data, &output_value, sizeof(int16_t));
  std::copy(std::begin(data), std::end(data), std::begin(msg.data));

  interface_->queue(msg);
}

void Driver::sendFixed16x16(const uint32_t id, const double value)
{
  can_msgs::msg::Frame msg = getMsg(id);
  msg.dlc = sizeof(int32_t);
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1 << 16) * value));

  uint8_t data[8] = {0};
  std::memcpy(data, &output_value, sizeof(int32_t));
  std::copy(std::begin(data), std::end(data), std::begin(msg.data));

  interface_->queue(msg);
}

can_msgs::msg::Frame Driver::getMsg(const uint32_t id)
{
  can_msgs::msg::Frame msg;
  msg.id = id;
  msg.dlc = 0;
  msg.is_extended = true;
  msg.header.stamp = nh_->get_clock()->now();
  msg.header.frame_id = "base_link";
  return msg;
}

uint32_t Driver::getApi(const can_msgs::msg::Frame msg)
{
  return msg.id & (CAN_MSGID_FULL_M ^ CAN_MSGID_DEVNO_M);
}

uint32_t Driver::getDeviceNumber(const can_msgs::msg::Frame msg)
{
  return msg.id & CAN_MSGID_DEVNO_M;
}

bool Driver::verifyRaw16x16(const uint8_t * received, const double expected)
{
  uint8_t data[4];
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1 << 16) * expected));
  std::memcpy(data, &output_value, 4);
  for (uint8_t i = 0; i < 4; i++) {
    if (*received != data[i]) {
      return false;
    }
    received++;
  }
  return true;
}

bool Driver::verifyRaw8x8(const uint8_t * received, const float expected)
{
  uint8_t data[2];
  int32_t output_value = static_cast<int32_t>(static_cast<float>((1 << 8) * expected));
  std::memcpy(data, &output_value, 2);
  for (uint8_t i = 0; i < 2; i++) {
    if (*received != data[i]) {
      return false;
    }
    received++;
  }
  return true;
}

void Driver::setEncoderCPR(const uint16_t encoder_cpr)
{
  encoder_cpr_ = encoder_cpr;
}

void Driver::setGearRatio(const float gear_ratio)
{
  gear_ratio_ = gear_ratio;
}

void Driver::commandDutyCycle(const float cmd)
{
  sendFixed8x8((LM_API_VOLT_SET | device_number_), cmd);
}

void Driver::commandSpeed(const double cmd)
{
  // Converting from rad/s to RPM through the gearbox.
  sendFixed16x16((LM_API_SPD_SET | device_number_), (cmd * radPerSecToRpm()));
}

void Driver::verifyParams()
{
  switch (state_) {
    case ConfigurationState::Initializing:
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Puma Motor Controller on %s (%i): starting to verify parameters.",
        device_name_.c_str(), device_number_);
      state_ = ConfigurationState::PowerFlag;
      break;
    case ConfigurationState::PowerFlag:
      if (lastPower() == 0) {
        state_ = ConfigurationState::EncoderPosRef;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): cleared power flag.",
          device_name_.c_str(), device_number_);
      } else {
        sendId(LM_API_STATUS_POWER | device_number_);
      }
      break;
    case ConfigurationState::EncoderPosRef:
      if (posEncoderRef() == LM_REF_ENCODER) {
        state_ = ConfigurationState::EncoderSpdRef;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set position encoder reference.",
          device_name_.c_str(), device_number_);
      } else {
        sendId(LM_API_POS_REF | device_number_);
      }
      break;
    case ConfigurationState::EncoderSpdRef:
      if (spdEncoderRef() == LM_REF_QUAD_ENCODER) {
        state_ = ConfigurationState::EncoderCounts;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set speed encoder reference.",
          device_name_.c_str(), device_number_);
      } else {
        sendId(LM_API_SPD_REF | device_number_);
      }
      break;
    case ConfigurationState::EncoderCounts:
      if (encoderCounts() == encoder_cpr_) {
        state_ = ConfigurationState::ClosedLoop;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set encoder counts to %i.",
          device_name_.c_str(), device_number_, encoder_cpr_);
      } else {
        sendId(LM_API_CFG_ENC_LINES | device_number_);
      }
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      if (lastMode() == clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED) {
        state_ = ConfigurationState::ControlMode;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): entered a close-loop control mode.",
          device_name_.c_str(), device_number_);
      } else {
        sendId(LM_API_STATUS_CMODE | device_number_);
      }
      break;
    case ConfigurationState::ControlMode:
      if (lastMode() == control_mode_) {
        if (control_mode_ != clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE) {
          state_ = ConfigurationState::PGain;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
            "Puma Motor Controller on %s (%i): was set to a close loop control mode.",
            device_name_.c_str(), device_number_);
        } else {
          state_ = ConfigurationState::VerifiedParameters;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
            "Puma Motor Controller on %s (%i): was set to voltage control mode.",
            device_name_.c_str(), device_number_);
        }
      }
      break;
    case ConfigurationState::PGain:
      if (verifyRaw16x16(getRawP(), gain_p_)) {
        state_ = ConfigurationState::IGain;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): P gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getP(), gain_p_);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): P gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getP(), gain_p_);
        switch (control_mode_) {
          case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
            sendId(LM_API_ICTRL_PC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            sendId(LM_API_POS_PC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            sendId(LM_API_SPD_PC | device_number_);
            break;
        }
      }
      break;
    case ConfigurationState::IGain:
      if (verifyRaw16x16(getRawI(), gain_i_)) {
        state_ = ConfigurationState::DGain;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): I gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getI(), gain_i_);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): I gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getI(), gain_i_);
        switch (control_mode_) {
          case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
            sendId(LM_API_ICTRL_IC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            sendId(LM_API_POS_IC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            sendId(LM_API_SPD_IC | device_number_);
            break;
        }
      }
      break;
    case ConfigurationState::DGain:
      if (verifyRaw16x16(getRawD(), gain_d_)) {
        state_ = ConfigurationState::VerifiedParameters;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): D gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getD(), gain_d_);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): D gain constant was set to %f and %f was requested.",
          device_name_.c_str(), device_number_, getD(), gain_d_);
        switch (control_mode_) {
          case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
            sendId(LM_API_ICTRL_DC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            sendId(LM_API_POS_DC | device_number_);
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            sendId(LM_API_SPD_DC | device_number_);
            break;
        }
      }
      break;
  }
  if (state_ == ConfigurationState::VerifiedParameters) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "Puma Motor Controller on %s (%i): all parameters verified.",
      device_name_.c_str(), device_number_);
    configured_ = true;
    state_ = ConfigurationState::Configured;
  }
}

void Driver::configureParams()
{
  switch (state_) {
    case ConfigurationState::PowerFlag:
      sendUint8((LM_API_STATUS_POWER | device_number_), 1);
      break;
    case ConfigurationState::EncoderPosRef:
      sendUint8((LM_API_POS_REF | device_number_), LM_REF_ENCODER);
      break;
    case ConfigurationState::EncoderSpdRef:
      sendUint8((LM_API_SPD_REF | device_number_), LM_REF_QUAD_ENCODER);
      break;
    case ConfigurationState::EncoderCounts:
      // Set encoder CPR
      sendUint16((LM_API_CFG_ENC_LINES | device_number_), encoder_cpr_);
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      sendId(LM_API_SPD_EN | device_number_);
      break;
    case ConfigurationState::ControlMode:
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
          sendId(LM_API_VOLT_EN | device_number_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          sendId(LM_API_ICTRL_EN | device_number_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          sendId(LM_API_POS_EN | device_number_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          sendId(LM_API_SPD_EN | device_number_);
          break;
      }
      break;
    case ConfigurationState::PGain:
      // Set P
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_PC | device_number_), gain_p_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          sendFixed16x16((LM_API_POS_PC | device_number_), gain_p_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_PC | device_number_), gain_p_);
          break;
      }
      break;
    case ConfigurationState::IGain:
      // Set I
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_IC | device_number_), gain_i_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          sendFixed16x16((LM_API_POS_IC | device_number_), gain_i_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_IC | device_number_), gain_i_);
          break;
      }
      break;
    case ConfigurationState::DGain:
      // Set D
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_DC | device_number_), gain_d_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          sendFixed16x16((LM_API_POS_DC | device_number_), gain_d_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_DC | device_number_), gain_d_);
          break;
      }
      break;
  }
}

bool Driver::isConfigured() const
{
  return configured_;
}

void Driver::setGains(const double p, const double i, const double d)
{
  gain_p_ = p;
  gain_i_ = i;
  gain_d_ = d;

  if (configured_) {
    updateGains();
  }
}

void Driver::setMode(const uint8_t mode)
{
  if (mode == clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE) {
    control_mode_ = mode;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "Puma Motor Controller on %s (%i): mode set to voltage control.",
      device_name_.c_str(), device_number_);
    if (configured_) {
      resetConfiguration();
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Puma Motor Controller on %s (%i): Close loop modes need PID gains.",
      device_name_.c_str(), device_number_);
  }
}

void Driver::setMode(const uint8_t mode, const double p, const double i, const double d)
{
  if (mode == clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE) {
    control_mode_ = mode;
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "Puma Motor Controller on %s (%i): mode set to voltage control but PID gains are not needed.",
      device_name_.c_str(), device_number_);
    if (configured_) {
      resetConfiguration();
    }
  } else {
    control_mode_ = mode;
    if (configured_) {
      resetConfiguration();
    }
    setGains(p, i, d);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Puma Motor Controller on %s (%i): mode set to a closed-loop control with PID gains of P:%f, I:%f and D:%f.",
        device_name_.c_str(), device_number_, gain_p_, gain_i_, gain_d_);
  }
}

void Driver::clearMsgCache()
{
  // Set it all to zero, which will in part clear
  // the boolean flags to be false.
  memset(voltage_fields_, 0, sizeof(voltage_fields_));
  memset(spd_fields_, 0, sizeof(spd_fields_));
  memset(vcomp_fields_, 0, sizeof(vcomp_fields_));
  memset(pos_fields_, 0, sizeof(pos_fields_));
  memset(ictrl_fields_, 0, sizeof(ictrl_fields_));
  memset(status_fields_, 0, sizeof(status_fields_));
  memset(cfg_fields_, 0, sizeof(cfg_fields_));
}

void Driver::requestStatusMessages()
{
  sendId(LM_API_STATUS_POWER | device_number_);
}

void Driver::requestFeedbackMessages()
{
  sendId(LM_API_STATUS_VOLTOUT | device_number_);
  sendId(LM_API_STATUS_CURRENT | device_number_);
  sendId(LM_API_STATUS_POS | device_number_);
  sendId(LM_API_STATUS_SPD | device_number_);
  sendId(LM_API_SPD_SET | device_number_);
}
void Driver::requestFeedbackDutyCycle()
{
  sendId(LM_API_STATUS_VOLTOUT | device_number_);
}

void Driver::requestFeedbackCurrent()
{
  sendId(LM_API_STATUS_CURRENT | device_number_);
}

void Driver::requestFeedbackPosition()
{
  sendId(LM_API_STATUS_POS | device_number_);
}

void Driver::requestFeedbackSpeed()
{
  sendId(LM_API_STATUS_SPD | device_number_);
}

void Driver::requestFeedbackPowerState()
{
  sendId(LM_API_STATUS_POWER | device_number_);
}

void Driver::requestFeedbackSetpoint()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      sendId(LM_API_ICTRL_SET | device_number_);
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      sendId(LM_API_POS_SET | device_number_);
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      sendId(LM_API_SPD_SET | device_number_);
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
      sendId(LM_API_VOLT_SET | device_number_);
      break;
  }
}

void Driver::resetConfiguration()
{
  configured_ = false;
  state_ = ConfigurationState::Initializing;
}

void Driver::updateGains()
{
  configured_ = false;
  state_ = ConfigurationState::PGain;
}

bool Driver::receivedDutyCycle()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOLTOUT)));
  return field->received;
}

bool Driver::receivedBusVoltage()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOLTBUS)));
  return field->received;
}

bool Driver::receivedCurrent()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_CURRENT)));
  return field->received;
}

bool Driver::receivedPosition()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_POS)));
  return field->received;
}

bool Driver::receivedSpeed()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_SPD)));
  return field->received;
}

bool Driver::receivedFault()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_FAULT)));
  return field->received;
}

bool Driver::receivedPower()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_POWER)));
  return field->received;
}

bool Driver::receivedMode()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_CMODE)));
  return field->received;
}

bool Driver::receivedOutVoltage()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOUT)));
  return field->received;
}

bool Driver::receivedTemperature()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_TEMP)));
  return field->received;
}

bool Driver::receivedAnalogInput()
{
  Field * field = statusFieldForMessage(getApi(getMsg(CPR_API_STATUS_ANALOG)));
  return field->received;
}

bool Driver::receivedSetpoint()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      return receivedCurrentSetpoint();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      return receivedPositionSetpoint();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      return receivedSpeedSetpoint();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
      return receivedDutyCycleSetpoint();
      break;
    default:
      return 0;
      break;
  }
}

bool Driver::receivedSpeedSetpoint()
{
  Field * field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_SET)));
  return field->received;
}

bool Driver::receivedDutyCycleSetpoint()
{
  Field * field = voltageFieldForMessage(getApi(getMsg(LM_API_VOLT_SET)));
  return field->received;
}

bool Driver::receivedCurrentSetpoint()
{
  Field * field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_SET)));
  return field->received;
}

bool Driver::receivedPositionSetpoint()
{
  Field * field = posFieldForMessage(getApi(getMsg(LM_API_POS_SET)));
  return field->received;
}

float Driver::lastDutyCycle()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOLTOUT)));
  field->received = false;
  return field->interpretFixed8x8() / 128.0;
}

float Driver::lastBusVoltage()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOLTBUS)));
  field->received = false;
  return field->interpretFixed8x8();
}

float Driver::lastCurrent()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_CURRENT)));
  field->received = false;
  return field->interpretFixed8x8();
}

double Driver::lastPosition()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_POS)));
  field->received = false;
  return field->interpretFixed16x16() * ((2 * M_PI) / gear_ratio_);  // Convert rev to rad
}

double Driver::lastSpeed()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_SPD)));
  field->received = false;
  return field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60));  // Convert RPM to rad/s
}

uint8_t Driver::lastFault()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_FAULT)));
  field->received = false;
  return field->data[0];
}

uint8_t Driver::lastPower()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_POWER)));
  field->received = false;
  return field->data[0];
}

uint8_t Driver::lastMode()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_CMODE)));
  field->received = false;
  return field->data[0];
}

float Driver::lastOutVoltage()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_VOUT)));
  field->received = false;
  return field->interpretFixed8x8();
}

float Driver::lastTemperature()
{
  Field * field = statusFieldForMessage(getApi(getMsg(LM_API_STATUS_TEMP)));
  field->received = false;
  return field->interpretFixed8x8();
}

float Driver::lastAnalogInput()
{
  Field * field = statusFieldForMessage(getApi(getMsg(CPR_API_STATUS_ANALOG)));
  field->received = false;
  return field->interpretFixed8x8();
}

double Driver::lastSetpoint()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      return statusCurrentGet();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      return statusPositionGet();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      return statusSpeedGet();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
      return statusDutyCycleGet();
      break;
    default:
      return 0;
      break;
  }
}
double Driver::statusSpeedGet()
{
  Field * field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_SET)));
  field->received = false;
  return field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60));  // Convert RPM to rad/s
}

float Driver::statusDutyCycleGet()
{
  Field * field = voltageFieldForMessage(getApi(getMsg(LM_API_VOLT_SET)));
  field->received = false;
  return field->interpretFixed8x8() / 128.0;
}

float Driver::statusCurrentGet()
{
  Field * field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_SET)));
  field->received = false;
  return field->interpretFixed8x8();
}

double Driver::statusPositionGet()
{
  Field * field = posFieldForMessage(getApi(getMsg(LM_API_POS_SET)));
  field->received = false;
  return field->interpretFixed16x16() * (( 2 * M_PI) / gear_ratio_);  // Convert rev to rad
}

uint8_t Driver::posEncoderRef()
{
  Field * field = posFieldForMessage(getApi(getMsg(LM_API_POS_REF)));
  return field->data[0];
}

uint8_t Driver::spdEncoderRef()
{
  Field * field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_REF)));
  return field->data[0];
}

uint16_t Driver::encoderCounts()
{
  Field * field = cfgFieldForMessage(getApi(getMsg(LM_API_CFG_ENC_LINES)));
  return static_cast<uint16_t>(field->data[0]) | static_cast<uint16_t>(field->data[1] << 8);
}

double Driver::getP()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_PC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_PC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_PC)));
      break;
  }
  return field->interpretFixed16x16();
}

double Driver::getI()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_IC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_IC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_IC)));
      break;
  }
  return field->interpretFixed16x16();
}

double Driver::getD()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_DC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_DC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_DC)));
      break;
  }
  return field->interpretFixed16x16();
}

uint8_t * Driver::getRawP()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_PC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_PC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_PC)));
      break;
  }
  return field->data;
}

uint8_t * Driver::getRawI()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_IC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_IC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_IC)));
      break;
  }
  return field->data;
}

uint8_t * Driver::getRawD()
{
  Field * field;
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      field = ictrlFieldForMessage(getApi(getMsg(LM_API_ICTRL_DC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      field = posFieldForMessage(getApi(getMsg(LM_API_POS_DC)));
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      field = spdFieldForMessage(getApi(getMsg(LM_API_SPD_DC)));
      break;
  }
  return field->data;
}

Driver::Field * Driver::voltageFieldForMessage(uint32_t api)
{
  uint32_t voltage_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &voltage_fields_[voltage_field_index];
}

Driver::Field * Driver::spdFieldForMessage(uint32_t api)
{
  uint32_t spd_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &spd_fields_[spd_field_index];
}

Driver::Field * Driver::vcompFieldForMessage(uint32_t api)
{
  uint32_t vcomp_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &vcomp_fields_[vcomp_field_index];
}

Driver::Field * Driver::posFieldForMessage(uint32_t api)
{
  uint32_t pos_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &pos_fields_[pos_field_index];
}

Driver::Field * Driver::ictrlFieldForMessage(uint32_t api)
{
  uint32_t ictrl_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &ictrl_fields_[ictrl_field_index];
}

Driver::Field * Driver::statusFieldForMessage(uint32_t api)
{
  uint32_t status_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &status_fields_[status_field_index];
}

Driver::Field * Driver::cfgFieldForMessage(uint32_t api)
{
  uint32_t cfg_field_index = (api & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &cfg_fields_[cfg_field_index];
}

}  // namespace puma_motor_driver
