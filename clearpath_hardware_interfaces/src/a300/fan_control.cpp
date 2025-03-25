/**
Software License Agreement (BSD)
\file      fan_control.cpp
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
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

#include "clearpath_hardware_interfaces/a300/fan_control.hpp"

std::string a300_cooling::thermalStatusToString(a300_cooling::ThermalStatus status)
{
  switch (status)
  {
    case a300_cooling::ThermalStatus::Normal:
      return "Normal";
    case a300_cooling::ThermalStatus::LowError:
      return "Low Error";
    case a300_cooling::ThermalStatus::LowWarning:
      return "Low Warning";
    case a300_cooling::ThermalStatus::HighWarning:
      return "High Warning";
    case a300_cooling::ThermalStatus::HighError:
      return "High Error";
    default:
      return "Unknown";
  }
}

a300_cooling::ThermalSensor::ThermalSensor(float low_error, float low_warning, float high_warning, float high_error)
  : reading_(INITIAL_READING), low_error_(low_error), low_warning_(low_warning), high_warning_(high_warning), high_error_(high_error) {}

a300_cooling::ThermalSensor::ThermalSensor()
  : reading_(INITIAL_READING), low_error_(0.0), low_warning_(0.0), high_warning_(0.0), high_error_(0.0) {}

void a300_cooling::ThermalSensor::setValue(float value) { this->reading_ = value; }

float a300_cooling::ThermalSensor::getValue() const { return this->reading_; }

a300_cooling::ThermalStatus a300_cooling::ThermalSensor::getStatus() const
{
  if (this->reading_ < this->low_error_) return a300_cooling::ThermalStatus::LowError;
  if (this->reading_ < this->low_warning_) return a300_cooling::ThermalStatus::LowWarning;
  if (this->reading_ > this->high_error_) return a300_cooling::ThermalStatus::HighError;
  if (this->reading_ > this->high_warning_) return a300_cooling::ThermalStatus::HighWarning;
  return a300_cooling::ThermalStatus::Normal;
}

void a300_cooling::ThermalSensor::getThresholds(float & low_error, float & low_warning, float & high_warning, float & high_error) const
{
  low_error = low_error_;
  low_warning = low_warning_;
  high_warning = high_warning_;
  high_error = high_error_;
}

a300_cooling::ThermalSensors::ThermalSensors()
{
  sensors =
  {
    {"mcu",          ThermalSensor{-20, 0, 60, 80}},
    {"fan1",         ThermalSensor{-20, 0, 60, 80}},
    {"fan2",         ThermalSensor{-20, 0, 60, 80}},
    {"fan3",         ThermalSensor{-20, 0, 60, 80}},
    {"fan4",         ThermalSensor{-20, 0, 60, 80}},
    {"5v_inductor",  ThermalSensor{-20, 0, 60, 80}},
    {"main_gnd_lug", ThermalSensor{-20, 0, 60, 80}},
    {"dcdc_24v",     ThermalSensor{-20, 0, 60, 80}},
    {"dcdc_12v",     ThermalSensor{-20, 0, 60, 80}},
    {"battery",      ThermalSensor{-20, 0, 55, 60}},
    {"pcb_motor1",   ThermalSensor{-20, 0, 60, 80}},
    {"mcu_motor1",   ThermalSensor{-20, 0, 60, 80}},
    {"pcb_motor2",   ThermalSensor{-20, 0, 60, 80}},
    {"mcu_motor2",   ThermalSensor{-20, 0, 60, 80}},
    {"pcb_motor3",   ThermalSensor{-20, 0, 60, 80}},
    {"mcu_motor3",   ThermalSensor{-20, 0, 60, 80}},
    {"pcb_motor4",   ThermalSensor{-20, 0, 60, 80}},
    {"mcu_motor4",   ThermalSensor{-20, 0, 60, 80}},
  };
}

const std::map<std::string, a300_cooling::ThermalSensor> & a300_cooling::ThermalSensors::getSensors() const
{
  return sensors;
}

void a300_cooling::ThermalSensors::setSensorValue(const std::string &name, float value)
{
  if (sensors.find(name) != sensors.end())
  {
    sensors[name].setValue(value);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("a300_fan_controller"), "Could not find thermal sensor %s", name.c_str());
  }
}

a300_cooling::ThermalStatus a300_cooling::ThermalSensors::getHighestStatus() const
{
  a300_cooling::ThermalStatus highest_status = a300_cooling::ThermalStatus::Normal;
  for (const auto &sensor : sensors) 
  {
    if (sensor.second.getStatus() > a300_cooling::ThermalStatus::Normal)
    {
      RCLCPP_WARN(rclcpp::get_logger("a300_fan_controller"), "Thermal sensor %s has %s (Measured: %.1f C)",
                  sensor.first.c_str(), thermalStatusToString(sensor.second.getStatus()).c_str(),
                  sensor.second.getValue());
    }
    if (sensor.second.getStatus() > highest_status)
    {
      highest_status = sensor.second.getStatus();
    }
  }
  return highest_status;
}

a300_cooling::FanController::FanController() : Node("a300_fan_controller"),
  updater_(this)
{
  fan_publisher_ = create_publisher<clearpath_platform_msgs::msg::Fans>(FAN_CONTROL_TOPIC_NAME, rclcpp::SensorDataQoS());

  temp_subscription_ = create_subscription<clearpath_platform_msgs::msg::Temperature>(
    MCU_TEMPERATURE_TOPIC,
    rclcpp::SensorDataQoS(),
    [this](const clearpath_platform_msgs::msg::Temperature::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    this->thermal_sensors_.setSensorValue("mcu", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_MCU]);
    this->thermal_sensors_.setSensorValue("fan1", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_FAN1]);
    this->thermal_sensors_.setSensorValue("fan2", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_FAN2]);
    this->thermal_sensors_.setSensorValue("fan3", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_FAN3]);
    this->thermal_sensors_.setSensorValue("fan4", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_FAN4]);
    this->thermal_sensors_.setSensorValue("5v_inductor", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_5V_INDUCTOR]);
    this->thermal_sensors_.setSensorValue("main_gnd_lug", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_MAIN_GND_LUG]);
    this->thermal_sensors_.setSensorValue("dcdc_24v", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_24V_DCDC]);
    this->thermal_sensors_.setSensorValue("dcdc_12v", msg->temperatures[clearpath_platform_msgs::msg::Temperature::CC01_12V_DCDC]);
    this->temperature_stale_ = 0;
  });

  motor_subscription_ = create_subscription<clearpath_motor_msgs::msg::LynxMultiStatus>(
    MOTOR_TEMPERATURE_TOPIC,
    rclcpp::SensorDataQoS(),
    [this](const clearpath_motor_msgs::msg::LynxMultiStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    try
    {
      this->thermal_sensors_.setSensorValue("pcb_motor1",  msg->drivers.at(0).pcb_temperature);
      this->thermal_sensors_.setSensorValue("mcu_motor1",  msg->drivers.at(0).mcu_temperature);
      this->thermal_sensors_.setSensorValue("pcb_motor2",  msg->drivers.at(1).pcb_temperature);
      this->thermal_sensors_.setSensorValue("mcu_motor2",  msg->drivers.at(1).mcu_temperature);
      this->thermal_sensors_.setSensorValue("pcb_motor3",  msg->drivers.at(2).pcb_temperature);
      this->thermal_sensors_.setSensorValue("mcu_motor3",  msg->drivers.at(2).mcu_temperature);
      this->thermal_sensors_.setSensorValue("pcb_motor4",  msg->drivers.at(3).pcb_temperature);
      this->thermal_sensors_.setSensorValue("mcu_motor4",  msg->drivers.at(3).mcu_temperature);
    }
    catch (const std::out_of_range & e) 
    {
      RCLCPP_ERROR(this->get_logger(),
                   "%s topic does not contain 4 drivers: %s", MOTOR_TEMPERATURE_TOPIC.c_str(), e.what());
    }
    this->lynx_status_stale_ = 0;
  });

  battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
    BATTERY_TEMPERATURE_TOPIC,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    this->thermal_sensors_.setSensorValue("battery", msg->temperature);
    this->battery_stale_ = 0;
  });

  user_fan_cmd_subscription_ = create_subscription<clearpath_platform_msgs::msg::Fans>(
    USER_FAN_CONTROL_TOPIC_NAME,
    rclcpp::SensorDataQoS(),
    [this](const clearpath_platform_msgs::msg::Fans::SharedPtr msg)
  {
    if (msg->fans.size() != 8)
    {
      RCLCPP_ERROR(this->get_logger(),
          "Received invalid fan command message, expected 8 values, received %ld", msg->fans.size());
      return;
    }
    this->user_fans_cmd_msg_ = *msg;
    this->user_fan_cmd_timeout_ = 0;
  });


  this->temperature_stale_ = 0;
  this->lynx_status_stale_ = 0;
  this->battery_stale_ = 0;
  this->user_fan_cmd_timeout_ = 0;
  this->active_control_status_timeout_ = 0;

  this->user_control_active_ = false;

  this->control_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FanController::timerCallback, this));

  updater_.setHardwareID("a300");
  updater_.add("Fan Controller", this, &FanController::fanDiagnostic);
}

void a300_cooling::FanController::timerCallback()
{
  std::lock_guard<std::mutex> lock(update_mutex_);
  static auto current_status = a300_cooling::ThermalStatus::Normal;
  static auto last_status = a300_cooling::ThermalStatus::Normal;
  static uint8_t fan_command = computeFanValue(a300_cooling::ThermalStatus::Normal);

  // Get the updated current status
  current_status = thermal_sensors_.getHighestStatus();

  // Under user control, send out the message received from the user
  if ((current_status == a300_cooling::ThermalStatus::Normal)
      && this->user_control_active_)
  {
    this->fans_cmd_msg_ = user_fans_cmd_msg_;
    RCLCPP_DEBUG(get_logger(), "User control active, setting fans to user values");
  }
  // This when the user is not controlling the fans
  else
  {
    // If the controller is active, send out the expected command
    if (current_status != a300_cooling::ThermalStatus::Normal)
    {
      fan_command = computeFanValue(current_status);
      RCLCPP_DEBUG(get_logger(), "Set all fans to: %d", fan_command);
      last_status = current_status;
      this->active_control_status_timeout_ = 0;
    }
    // Trying to do hysteresis only when the change is coming from a non-normal state to normal
    else
    {
      // Is normal and was normal so send out the normal command
      if (last_status == a300_cooling::ThermalStatus::Normal)
      {
        fan_command = computeFanValue(current_status);
        RCLCPP_DEBUG(get_logger(), "Set all fans to: %d", fan_command);
      }
      else
      {
        fan_command = computeFanValue(last_status);
        RCLCPP_DEBUG(get_logger(), "Set all fans to: %d", fan_command);
        if (this->active_control_status_timeout_ >= ACTIVE_CONTROL_STATUS_TIMEOUT)
        {
          last_status = current_status;
        }
      }
    }
    this->fans_cmd_msg_.fans = {fan_command, fan_command, fan_command, fan_command, 0, 0, 0, 0};
  }
  
  fan_publisher_->publish(fans_cmd_msg_);

  if (user_fan_cmd_timeout_ >= USER_CMD_TIMEOUT)
  {
    this->user_control_active_ = false;
    RCLCPP_INFO(get_logger(), "User fan control inactive, no user command received in %d seconds", USER_CMD_TIMEOUT);
  }
  else
  {
    this->user_control_active_ = true;
    RCLCPP_INFO(get_logger(), "User fan control active");
  }

  temperature_stale_++;
  lynx_status_stale_++;
  battery_stale_++;

  this->user_fan_cmd_timeout_++;
  this->active_control_status_timeout_++;
}

uint8_t a300_cooling::FanController::computeFanValue(a300_cooling::ThermalStatus status)
{
  static float fan_fraction;

  switch (status)
  {
    case a300_cooling::ThermalStatus::HighError:
      fan_fraction = a300_cooling::FAN_CMD_HIGH_ERROR;
      break;
    case a300_cooling::ThermalStatus::HighWarning:
      fan_fraction = a300_cooling::FAN_CMD_HIGH_WARNING;
      break;
    case a300_cooling::ThermalStatus::LowError:
      fan_fraction = a300_cooling::FAN_CMD_LOW_ERROR;
      break;
    case a300_cooling::ThermalStatus::LowWarning:
      fan_fraction = a300_cooling::FAN_CMD_LOW_WARNING;
      break;
    default:
      fan_fraction = a300_cooling::FAN_CMD_NORMAL;
      break;
  }
  return static_cast<uint8_t>(fan_fraction * 255.0f);
}

void a300_cooling::FanController::fanDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "OK"); // Is overwritten in mergeSummary by warn or error

  // Set summary error if any topic is stale
  if (temperature_stale_ >= STALE_SECS)
  {
    stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::ERROR,
                  "No temperature data from %s", MCU_TEMPERATURE_TOPIC.c_str());
  }
  if (lynx_status_stale_ >= STALE_SECS)
  {
    stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::ERROR,
                       "No temperature data from %s", MOTOR_TEMPERATURE_TOPIC.c_str());
  }
  if (battery_stale_ >= STALE_SECS)
  {
    stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::ERROR,
                       "No temperature data from %s", BATTERY_TEMPERATURE_TOPIC.c_str());
  }

  // Report on fan control values
  for (uint8_t i = 0; i < fans_cmd_msg_.fans.size(); i++)
  {
    stat.addf("Fan " + std::to_string(i + 1) + " Control (%)", "%.0f", fans_cmd_msg_.fans [i] * 100 / 255.0);
  }

  float low_error_thresh = 0, low_warning_thresh = 0, high_warning_thresh = 0, high_error_thresh = 0;

  // Report all sensor temperatures and thresholds
  for (const auto &sensor : thermal_sensors_.getSensors())
  {
    stat.addf(sensor.first + " Temperature (C)", "%.1f", sensor.second.getValue());

    sensor.second.getThresholds(low_error_thresh, low_warning_thresh, high_warning_thresh, high_error_thresh);
    stat.addf(sensor.first + " Thresholds (C)",
              "Low Error: %.1f, Low Warning: %.1f, High Warning: %.1f, High Error: %.1f",
              low_error_thresh, low_warning_thresh, high_warning_thresh, high_error_thresh);

    auto status = sensor.second.getStatus();
    if (status == a300_cooling::ThermalStatus::HighWarning || status == a300_cooling::ThermalStatus::LowWarning)
    {
      stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::WARN, "%s on %s thermal sensor",
      thermalStatusToString(status).c_str(), sensor.first.c_str());
    }
    else if (status == a300_cooling::ThermalStatus::HighError || status == a300_cooling::ThermalStatus::LowError)
    {
      stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "%s on %s thermal sensor",
      thermalStatusToString(status).c_str(), sensor.first.c_str());
    }
  }

  stat.add("User Control Allowed", ((thermal_sensors_.getHighestStatus() == a300_cooling::ThermalStatus::Normal) ? "True" : "False"));
  stat.add("User Control Active", (user_control_active_ ? "True" : "False"));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a300_cooling::FanController>());
  rclcpp::shutdown();
  return 0;
}
