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
      return "LowError";
    case a300_cooling::ThermalStatus::LowWarning:
      return "LowWarning";
    case a300_cooling::ThermalStatus::HighWarning:
      return "HighWarning";
    case a300_cooling::ThermalStatus::HighError:
      return "HighError";
    default:
      return "Unknown";
  }
}

a300_cooling::ThermalSensor::ThermalSensor(float low_error, float low_warning, float high_warning, float high_error)
  : reading_(INITIAL_READING), low_error_(low_error), low_warning_(low_warning), high_warning_(high_warning), high_error_(high_error) {}

a300_cooling::ThermalSensor::ThermalSensor()
  : reading_(INITIAL_READING), low_error_(0.0), low_warning_(0.0), high_warning_(0.0), high_error_(0.0) {}

void a300_cooling::ThermalSensor::setValue(float value) { this->reading_ = value; }

a300_cooling::ThermalStatus a300_cooling::ThermalSensor::getStatus() const
{
  if (this->reading_ < this->low_error_) return a300_cooling::ThermalStatus::LowError;
  if (this->reading_ < this->low_warning_) return a300_cooling::ThermalStatus::LowWarning;
  if (this->reading_ > this->high_error_) return a300_cooling::ThermalStatus::HighError;
  if (this->reading_ > this->high_warning_) return a300_cooling::ThermalStatus::HighWarning;
  return a300_cooling::ThermalStatus::Normal;
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
  for (const auto &sensor : sensors) {
    if (sensor.second.getStatus() > highest_status)
    {
      if (sensor.second.getStatus() >= a300_cooling::ThermalStatus::Normal)
      {
        RCLCPP_WARN(rclcpp::get_logger("a300_fan_controller"), "Thermal sensor %s has %s",
        sensor.first.c_str(), thermalStatusToString(sensor.second.getStatus()).c_str());
      }
      highest_status = sensor.second.getStatus();
    }
  }
  return highest_status;
}

a300_cooling::FanController::FanController() : Node("a300_fan_controller")
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
  });

  motor_subscription_ = create_subscription<clearpath_motor_msgs::msg::LynxMultiStatus>(
    MOTOR_TEMPERATURE_TOPIC,
    rclcpp::SensorDataQoS(),
    [this](const clearpath_motor_msgs::msg::LynxMultiStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    this->thermal_sensors_.setSensorValue("pcb_motor1",  msg->drivers[0].pcb_temperature);
    this->thermal_sensors_.setSensorValue("mcu_motor1",  msg->drivers[0].mcu_temperature);
    this->thermal_sensors_.setSensorValue("pcb_motor2",  msg->drivers[1].pcb_temperature);
    this->thermal_sensors_.setSensorValue("mcu_motor2",  msg->drivers[1].mcu_temperature);
    this->thermal_sensors_.setSensorValue("pcb_motor3",  msg->drivers[2].pcb_temperature);
    this->thermal_sensors_.setSensorValue("mcu_motor3",  msg->drivers[2].mcu_temperature);
    this->thermal_sensors_.setSensorValue("pcb_motor4",  msg->drivers[3].pcb_temperature);
    this->thermal_sensors_.setSensorValue("mcu_motor4",  msg->drivers[3].mcu_temperature);
  });

  battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
    BATTERY_TEMPERATURE_TOPIC,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    this->thermal_sensors_.setSensorValue("battery", msg->temperature);
  });

  control_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FanController::timerCallback, this));
}

void a300_cooling::FanController::timerCallback()
{
  std::lock_guard<std::mutex> lock(update_mutex_);
  uint8_t fan_value = computeFanValue(thermal_sensors_.getHighestStatus());
  clearpath_platform_msgs::msg::Fans fans_msg;
  fans_msg.fans = {fan_value, fan_value, fan_value, fan_value, 0, 0, 0, 0};
  fan_publisher_->publish(fans_msg);
  RCLCPP_DEBUG(get_logger(), "Set all fans to: %d", fan_value);
}

uint8_t a300_cooling::FanController::computeFanValue(a300_cooling::ThermalStatus status)
{
  float fan_fraction;

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a300_cooling::FanController>());
  rclcpp::shutdown();
  return 0;
}
