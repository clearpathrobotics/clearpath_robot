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

#include "puma_motor_driver/driver.hpp"

#include <string>
#include <cstring>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

// must match firmware
#define CAN_FEEDBACK_RATE 40.0

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
  CanConnection* conn,
  std::shared_ptr<rclcpp::Node> nh,
  const uint8_t & device_number,
  const std::string & device_name)
: can_hardware::adaptor::CanAdaptor(device_name, conn),
  nh_(nh),
  device_number_(device_number),
  device_name_(device_name),
  configured_(false),
  state_(ConfigurationState::Initializing),
  last_power_clear_ts_(0.0),
  control_mode_(clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED),
  gain_p_(1),
  gain_i_(0),
  gain_d_(0),
  encoder_cpr_(1),
  gear_ratio_(1)
{

  // Register Signals

  // Voltage Control Signals (voltage_fields_[4]):
  voltage_enable_   = registerSignal<uint8_t>("voltage_enable",   LM_API_VOLT_EN,       device_number_);
  voltage_disable_  = registerSignal<uint8_t>("voltage_disable", LM_API_VOLT_DIS,      device_number_);
  voltage_set_      = registerSignal<float>("voltage_set",      LM_API_VOLT_SET,      device_number_);
  voltage_set_ramp_ = registerSignal<float>("voltage_set_ramp", LM_API_VOLT_SET_RAMP, device_number_);

  // Speed Control Signals (spd_fields_[7]):
  speed_enable_  = registerSignal<uint8_t>("speed_enable",    LM_API_SPD_EN,  device_number_);
  speed_disable_ = registerSignal<uint8_t>("speed_disable",   LM_API_SPD_DIS, device_number_);
  speed_set_     = registerSignal<double>("speed_set",        LM_API_SPD_SET, device_number_);
  speed_pc_      = registerSignal<double>("speed_pc",         LM_API_SPD_PC,  device_number_);
  speed_ic_      = registerSignal<double>("speed_ic",         LM_API_SPD_IC,  device_number_);
  speed_dc_      = registerSignal<double>("speed_dc",         LM_API_SPD_DC,  device_number_);
  speed_ref_     = registerSignal<uint8_t>("speed_ref",       LM_API_SPD_REF, device_number_);
  // Voltage Compensation Signals (vcomp_fields_[5]):
  (void)registerSignal<uint32_t>("vcomp_enable",   LM_API_VCOMP_EN, device_number_);
  (void)registerSignal<uint32_t>("vcomp_disable",  LM_API_VCOMP_DIS, device_number_);
  (void)registerSignal<int32_t>("vcomp_set",       LM_API_VCOMP_SET, device_number_);
  (void)registerSignal<int32_t>("vcomp_in_ramp",   LM_API_VCOMP_IN_RAMP, device_number_);
  (void)registerSignal<int32_t>("vcomp_comp_ramp", LM_API_VCOMP_COMP_RAMP, device_number_);

  // Position Control Signals (pos_fields_[7]):
  position_enable_  = registerSignal<uint32_t>("position_enable",  LM_API_POS_EN, device_number_);
  position_disable_ = registerSignal<uint32_t>("position_disable", LM_API_POS_DIS, device_number_);
  position_set_     = registerSignal<double>("position_set",      LM_API_POS_SET, device_number_);
  position_pc_      = registerSignal<double>("position_pc",        LM_API_POS_PC, device_number_);
  position_ic_      = registerSignal<double>("position_ic",        LM_API_POS_IC, device_number_);
  position_dc_      = registerSignal<double>("position_dc",        LM_API_POS_DC, device_number_);
  position_ref_     = registerSignal<uint8_t>("position_ref",      LM_API_POS_REF, device_number_);

  // Current Control Signals (ictrl_fields_[6]):
  ictrl_enable_  = registerSignal<uint32_t>("ictrl_enable",  LM_API_ICTRL_EN, device_number_);
  ictrl_disable_ = registerSignal<uint32_t>("ictrl_disable", LM_API_ICTRL_DIS, device_number_);
  ictrl_set_     = registerSignal<float>("ictrl_set",      LM_API_ICTRL_SET, device_number_);
  ictrl_pc_      = registerSignal<double>("ictrl_pc",        LM_API_ICTRL_PC, device_number_);
  ictrl_ic_      = registerSignal<double>("ictrl_ic",        LM_API_ICTRL_IC, device_number_);
  ictrl_dc_      = registerSignal<double>("ictrl_dc",        LM_API_ICTRL_DC, device_number_);

  // Status Signals (status_fields_[15]):
  status_voltage_out_  = registerSignal<float>("status_voltage_out",     LM_API_STATUS_VOLTOUT, device_number_);
  status_voltage_bus_  = registerSignal<float>("status_voltage_bus",   LM_API_STATUS_VOLTBUS, device_number_);
  status_current_      = registerSignal<float>("status_current",       LM_API_STATUS_CURRENT, device_number_);
  status_temperature_  = registerSignal<float>("status_temperature",   LM_API_STATUS_TEMP, device_number_);
  status_position_     = registerSignal<double>("status_position",      LM_API_STATUS_POS, device_number_);
  status_speed_        = registerSignal<double>("status_speed",         LM_API_STATUS_SPD, device_number_);
  status_limit_        = registerSignal<uint32_t>("status_limit",        LM_API_STATUS_LIMIT, device_number_);
  status_fault_        = registerSignal<uint8_t>("status_fault",        LM_API_STATUS_FAULT, device_number_);
  status_power_        = registerSignal<uint8_t>("status_power",         LM_API_STATUS_POWER, device_number_);
  status_control_mode_ = registerSignal<uint8_t>("status_control_mode",  LM_API_STATUS_CMODE, device_number_);
  status_vout_         = registerSignal<float>("status_vout",          LM_API_STATUS_VOUT, device_number_);
  status_sticky_fault_ = registerSignal<uint32_t>("status_sticky_fault", LM_API_STATUS_STKY_FLT, device_number_);
  status_fault_count_  = registerSignal<uint32_t>("status_fault_count",  LM_API_STATUS_FLT_COUNT, device_number_);
  status_analog_       = registerSignal<float>("status_analog",        CPR_API_STATUS_ANALOG, device_number_);

  // Configuration Signals (cfg_fields_[15]):
  (void)registerSignal<uint32_t>("cfg_num_brushes",          LM_API_CFG_NUM_BRUSHES, device_number_);
  cfg_enc_lines_ = registerSignal<uint16_t>("cfg_enc_lines", LM_API_CFG_ENC_LINES, device_number_);
  (void)registerSignal<uint32_t>("cfg_pot_turns",            LM_API_CFG_POT_TURNS, device_number_);
  (void)registerSignal<uint32_t>("cfg_brake_coast",          LM_API_CFG_BRAKE_COAST, device_number_);
  (void)registerSignal<uint32_t>("cfg_limit_mode",           LM_API_CFG_LIMIT_MODE, device_number_);
  (void)registerSignal<int32_t>("cfg_limit_fwd",             LM_API_CFG_LIMIT_FWD, device_number_);
  (void)registerSignal<int32_t>("cfg_limit_rev",             LM_API_CFG_LIMIT_REV, device_number_);
  (void)registerSignal<uint32_t>("cfg_max_vout",             LM_API_CFG_MAX_VOUT, device_number_);
  (void)registerSignal<uint32_t>("cfg_fault_time",           LM_API_CFG_FAULT_TIME, device_number_);
  (void)registerSignal<uint32_t>("cfg_shutdown_temp",        CPR_API_CFG_SHUTDOWN_TEMP, device_number_);
  (void)registerSignal<uint32_t>("cfg_minimum_level",        CPR_API_CFG_MINIMUM_LEVEL, device_number_);
  (void)registerSignal<uint32_t>("cfg_nominal_level",        CPR_API_CFG_NOMINAL_LEVEL, device_number_);
  (void)registerSignal<uint32_t>("cfg_shutoff_level",        CPR_API_CFG_SHUTOFF_LEVEL, device_number_);
  (void)registerSignal<uint32_t>("cfg_shutoff_time",         CPR_API_CFG_SHUTOFF_TIME, device_number_);

  RCLCPP_INFO(nh_->get_logger(), "Puma Motor Driver initialized signals for device %s with ID %d", device_name_.c_str(), device_number_);

  can_feedback_rate_ = std::make_shared<double>(CAN_FEEDBACK_RATE);
  can_feedback_freq_status_ = std::make_shared<diagnostic_updater::FrequencyStatus>(
    diagnostic_updater::FrequencyStatusParam(
      can_feedback_rate_.get(),
      can_feedback_rate_.get(),
      0.1,
      5
    )
  );
}

double Driver::radPerSecToRpm() const
{
  return (60 * gear_ratio_) / (2 * M_PI);
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
  // requestWrite(voltage_set_.name, cmd);
  voltage_set_->requestWrite(cmd);
}

void Driver::commandSpeed(const double cmd)
{
  speed_set_->requestWrite(cmd * radPerSecToRpm());
}

void Driver::verifyParams()
{
  switch (state_) {
    case ConfigurationState::Initializing:
      // Don't advance state until we receive LM_API_STATUS_POWER msg
      if (receivedPower()) {
        state_ = ConfigurationState::PowerFlag;
      } else {
        // requestRead(status_power_.name);
        status_power_->requestRead();
      }
      break;
    case ConfigurationState::PowerFlag:
      if (receivedPower() && (lastPower() == 0)) {
        // Only advance state when we have a new LM_API_STATUS_POWER msg
        // with the power flag cleared
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): cleared power flag.",
          device_name_.c_str(), device_number_);
        state_ = ConfigurationState::EncoderPosRef;
      } else {
        status_power_->requestRead();
      }
      break;
    case ConfigurationState::EncoderPosRef:
      if (posEncoderRef() == LM_REF_ENCODER) {
        state_ = ConfigurationState::EncoderSpdRef;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set position encoder reference.",
          device_name_.c_str(), device_number_);
      } else {
        position_ref_->requestRead();
      }
      break;
    case ConfigurationState::EncoderSpdRef:
      if (spdEncoderRef() == LM_REF_QUAD_ENCODER) {
        state_ = ConfigurationState::EncoderCounts;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set speed encoder reference.",
          device_name_.c_str(), device_number_);
      } else {
        // requestRead(speed_ref_.name);
        speed_ref_->requestRead();
      }
      break;
    case ConfigurationState::EncoderCounts:
      if (encoderCounts() == encoder_cpr_) {
        state_ = ConfigurationState::ClosedLoop;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): set encoder counts to %i.",
          device_name_.c_str(), device_number_, encoder_cpr_);
      } else {
        cfg_enc_lines_->requestRead();
      }
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      if (lastMode() == clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED) {
        state_ = ConfigurationState::ControlMode;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
          "Puma Motor Controller on %s (%i): entered a close-loop control mode.",
          device_name_.c_str(), device_number_);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
          "Puma Motor Controller on %s (%i): must be set to a close-loop control mode to configure parameters.",
          device_name_.c_str(), device_number_);
        status_control_mode_->requestRead();
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
      if (verify(getRawP(), gain_p_)) {
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
            ictrl_pc_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            position_pc_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            speed_pc_->requestRead();
            break;
        }
      }
      break;
    case ConfigurationState::IGain:
      if (verify(getRawI(), gain_i_)) {
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
            ictrl_ic_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            position_ic_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            speed_ic_->requestRead();
            break;
        }
      }
      break;
    case ConfigurationState::DGain:
      if (verify(getRawD(), gain_d_)) {
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
            ictrl_dc_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
            position_dc_->requestRead();
            break;
          case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
            speed_dc_->requestRead();
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
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "Puma Motor Controller on %s (%i): configuring parameters.",
    device_name_.c_str(), device_number_);

  const double now = nh_->get_clock()->now().seconds();
  switch (state_) {
    case ConfigurationState::Initializing:
      break;
    case ConfigurationState::PowerFlag:
      // Continue to check last power flag until it has been cleared
      if (lastPower() == 1) {
        // Send request every second
        if ((now - last_power_clear_ts_) > 1.0) {
          status_power_->requestWrite(static_cast<uint8_t>(1));
          last_power_clear_ts_ = now;
        }
      }
      break;
    case ConfigurationState::EncoderPosRef:
      position_ref_->requestWrite(static_cast<uint8_t>(LM_REF_ENCODER));
      break;
    case ConfigurationState::EncoderSpdRef:
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "Puma Motor Controller on %s (%i): setting speed encoder reference to %i.",
        device_name_.c_str(), device_number_, LM_REF_QUAD_ENCODER);
      speed_ref_->requestWrite(static_cast<uint8_t>(LM_REF_QUAD_ENCODER));
      break;
    case ConfigurationState::EncoderCounts:
      // Set encoder CPR
      cfg_enc_lines_->requestWrite(encoder_cpr_);
      break;
    case ConfigurationState::ClosedLoop:  // Need to enter a close loop mode to record encoder data.
      speed_enable_->requestRead();
      break;
    case ConfigurationState::ControlMode:
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
          voltage_enable_->requestRead();
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          ictrl_enable_->requestRead();
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          position_enable_->requestRead();
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          speed_enable_->requestRead();
          break;
      }
      break;
    case ConfigurationState::PGain:
      // Set P
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          ictrl_pc_->requestWrite(gain_p_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          position_pc_->requestWrite(gain_p_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          speed_pc_->requestWrite(gain_p_);
          break;
      }
      break;
    case ConfigurationState::IGain:
      // Set I
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          ictrl_ic_->requestWrite(gain_i_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          position_ic_->requestWrite(gain_i_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          speed_ic_->requestWrite(gain_i_);
          break;
      }
      break;
    case ConfigurationState::DGain:
      // Set D
      switch (control_mode_) {
        case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
          ictrl_dc_->requestWrite(gain_d_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
          position_dc_->requestWrite(gain_d_);
          break;
        case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
          speed_dc_->requestWrite(gain_d_);
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

void Driver::requestStatusMessages()
{
  status_power_->requestRead();
}

void Driver::requestFeedbackMessages()
{
  status_voltage_out_->requestRead();
  status_current_->requestRead();
  status_position_->requestRead();
  status_speed_->requestRead();
  speed_set_->requestRead();
}
void Driver::requestFeedbackDutyCycle()
{
  status_voltage_out_->requestRead();
}

void Driver::requestFeedbackCurrent()
{
  status_current_->requestRead();
}

void Driver::requestFeedbackPosition()
{
  status_position_->requestRead();
}

void Driver::requestFeedbackSpeed()
{
  status_speed_->requestRead();
}

void Driver::requestFeedbackPowerState()
{
  status_power_->requestRead();
}

void Driver::requestFeedbackSetpoint()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      ictrl_set_->requestRead();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      position_set_->requestRead();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      speed_set_->requestRead();
      break;
    case clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE:
      voltage_set_->requestRead();
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
  return status_voltage_out_->received();
}

bool Driver::receivedBusVoltage()
{
  return status_voltage_bus_->received();
}

bool Driver::receivedCurrent()
{
  return status_current_->received();
}

bool Driver::receivedPosition()
{
  return status_position_->received();
}

bool Driver::receivedSpeed()
{
  return status_speed_->received();
}

bool Driver::receivedFault()
{
  return status_fault_->received();
}

bool Driver::receivedPower()
{
  return status_power_->received();
}

bool Driver::receivedMode()
{
  return status_control_mode_->received();
}

bool Driver::receivedOutVoltage()
{
  return status_vout_->received();
}

bool Driver::receivedTemperature()
{
  return status_temperature_->received();
}

bool Driver::receivedAnalogInput()
{
  return status_analog_->received();
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
  return speed_set_->received();
}

bool Driver::receivedDutyCycleSetpoint()
{
  return voltage_set_->received();
}

bool Driver::receivedCurrentSetpoint()
{
  return ictrl_set_->received();
}

bool Driver::receivedPositionSetpoint()
{
  return position_set_->received();
}

float Driver::lastDutyCycle()
{
  return status_voltage_out_->value() / 128.0;
}

float Driver::lastBusVoltage()
{
  return status_voltage_bus_->value();
}

float Driver::lastCurrent()
{
  return status_current_->value();
}

double Driver::lastPosition()
{
  return status_position_->value() * ((2 * M_PI) / gear_ratio_);  // Convert rev to rad
}

double Driver::lastSpeed()
{
  return status_speed_->value() * ((2 * M_PI) / (gear_ratio_ * 60));  // Convert RPM to rad/s
}

uint8_t Driver::lastFault()
{
  return status_fault_->value();
}

uint8_t Driver::lastPower()
{
  return status_power_->value();
}

uint8_t Driver::lastMode()
{
  return status_control_mode_->value();
}

float Driver::lastOutVoltage()
{
  return status_vout_->value();
}

float Driver::lastTemperature()
{
  return status_temperature_->value();
}

float Driver::lastAnalogInput()
{
  return status_analog_->value();
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
  return status_speed_->value() * ((2 * M_PI) / (gear_ratio_ * 60));  // Convert RPM to rad/s
}

float Driver::statusDutyCycleGet()
{
  return voltage_set_->value() / 128.0;
}

float Driver::statusCurrentGet()
{
  return ictrl_set_->value();
}

double Driver::statusPositionGet()
{
  return position_set_->value() * (( 2 * M_PI) / gear_ratio_);  // Convert rev to rad
}

uint8_t Driver::posEncoderRef()
{
  return position_ref_->value();
}

uint8_t Driver::spdEncoderRef()
{
  return speed_ref_->value();
}

uint16_t Driver::encoderCounts()
{
  return cfg_enc_lines_->value();
}

double Driver::getP()
{
  return getRawP();
}

double Driver::getI()
{
  return getRawI();
}

double Driver::getD()
{
  return getRawD();
}

double Driver::getRawP()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      return ictrl_pc_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      return position_pc_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      return speed_pc_->value();
    default:
      return 0.0;
  }
}

double Driver::getRawI()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      return ictrl_ic_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      return position_ic_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      return speed_ic_->value();
    default:
      return 0.0;
  }
}

double Driver::getRawD()
{
  switch (control_mode_) {
    case clearpath_motor_msgs::msg::PumaStatus::MODE_CURRENT:
      return ictrl_dc_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_POSITION:
      return position_dc_->value();
    case clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED:
      return speed_dc_->value();
    default:
      return 0.0;
  }
}

/**
 * @brief Runs the frequency diagnostic update to populate the status message
 */
void Driver::runFreqStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  can_feedback_freq_status_->run(stat);

  stat.add("Duty cycle", lastDutyCycle());
  stat.add("Current (A)", lastCurrent());
  stat.add("Speed (rad/s)", lastSpeed());
  stat.add("Position", lastPosition());
  stat.add("Setpoint", lastSetpoint());
}

}  // namespace puma_motor_driver
