/**
Software License Agreement (BSD)
\file      fan_control.hpp
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

#ifndef CLEARPATH_HARDWARE_INTERFACES_A300_FAN_CONTROL_HPP
#define CLEARPATH_HARDWARE_INTERFACES_A300_FAN_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <clearpath_platform_msgs/msg/temperature.hpp>
#include <clearpath_platform_msgs/msg/fans.hpp>
#include <clearpath_motor_msgs/msg/lynx_multi_status.hpp>
#include <mutex>
#include <map>

namespace a300_cooling
{
/// Fan control topic name
const std::string FAN_CONTROL_TOPIC_NAME = "platform/mcu/cmd_fans";

/// MCU temperature topic name
const std::string MCU_TEMPERATURE_TOPIC = "platform/mcu/status/temperature";

/// Motor temperature topic name
const std::string MOTOR_TEMPERATURE_TOPIC = "platform/motors/status";

/// Battery temperature topic name
const std::string BATTERY_TEMPERATURE_TOPIC = "platform/bms/battery_state";

/// Fan command for high error state, percent of maximum speed
constexpr float FAN_CMD_HIGH_ERROR = 1.0f;

/// Fan command for high warning state, percent of maximum speed
constexpr float FAN_CMD_HIGH_WARNING = 0.75f;

/// Fan command for low error state, percent of maximum speed
constexpr float FAN_CMD_LOW_ERROR = 0.0f;

/// Fan command for low warning state, percent of maximum speed
constexpr float FAN_CMD_LOW_WARNING = 0.25f;

/// Fan command for normal state, percent of maximum speed
constexpr float FAN_CMD_NORMAL = 0.5f;

/// Initial temperature reading when no data is available, degrees Celsius
constexpr float INITIAL_READING = 30.0f;

/// Enumeration to represent thermal status
enum class ThermalStatus
{
  Normal,      ///< Thermal state is within normal range
  LowError,    ///< Low error thermal state
  LowWarning,  ///< Low warning thermal state
  HighWarning, ///< High warning thermal state
  HighError    ///< High error thermal state
};

/**
 * @brief Convert thermal status to string
 * 
 * @param status The thermal status
 * @return A string representing the thermal status
 */
std::string thermalStatusToString(ThermalStatus status);

/**
 * @brief Represents a thermal sensor with associated status thresholds
 */
class ThermalSensor
{
public:
  /**
   * @brief Construct a new ThermalSensor object with specific threshold values
   * 
   * @param low_error Temperature below this value is an error
   * @param low_warning Temperature below this value is a warning
   * @param high_warning Temperature above this value is a warning
   * @param high_error Temperature above this value is an error
   */
  ThermalSensor(float low_error, float low_warning, float high_warning, float high_error);

  /**
   * @brief Construct a new ThermalSensor object with default values
   */
  ThermalSensor();

  /**
   * @brief Set the temperature reading of the sensor
   * 
   * @param value Temperature value to be set
   */
  void setValue(float value);

  /**
   * @brief Get the current thermal status of the sensor
   * 
   * @return The current thermal status
   */
  ThermalStatus getStatus() const;

private:
  float reading_;     ///< Current temperature reading
  float low_error_;   ///< Threshold for low error
  float low_warning_; ///< Threshold for low warning
  float high_warning_;///< Threshold for high warning
  float high_error_;  ///< Threshold for high error
  ThermalStatus thermal_status_; ///< Current thermal status
};

/**
 * @brief Collection of thermal sensors in the system
 */
class ThermalSensors
{
public:
  /**
   * @brief Default constructor that initializes the sensor list with default values
   */
  ThermalSensors();

  /**
   * @brief Set the value of a specific sensor
   * 
   * @param name The sensor's name
   * @param value The temperature value to set
   */
  void setSensorValue(const std::string &name, float value);

  /**
   * @brief Get the highest thermal status from all sensors
   * 
   * @return The highest thermal status
   */
  ThermalStatus getHighestStatus() const;

private:
  std::map<std::string, ThermalSensor> sensors; ///< Map of sensor names to sensor objects
};

/**
 * @brief Main class for managing fan control based on thermal sensor readings
 */
class FanController : public rclcpp::Node 
{
public:
  /**
   * @brief Construct a new FanController object
   */
  FanController();

private:
  /**
   * @brief Timer callback to periodically check and control the fans
   */
  void timerCallback();

  /**
   * @brief Compute the fan value based on the thermal status
   * 
   * @param status The thermal status of the highest priority sensor
   * @return Computed fan value as a uint8_t (0 to 255)
   */
  uint8_t computeFanValue(ThermalStatus status);

  std::mutex update_mutex_;    ///< Mutex for updating thermal sensors
  ThermalSensors thermal_sensors_; ///< Thermal sensor data
  rclcpp::Publisher<clearpath_platform_msgs::msg::Fans>::SharedPtr fan_publisher_; ///< Fan control publisher
  rclcpp::Subscription<clearpath_platform_msgs::msg::Temperature>::SharedPtr temp_subscription_; ///< Temperature subscription
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_; ///< Battery state subscription
  rclcpp::Subscription<clearpath_motor_msgs::msg::LynxMultiStatus>::SharedPtr motor_subscription_; ///< Motor status subscription
  rclcpp::TimerBase::SharedPtr control_timer_; ///< Timer for controlling the fan
};

}  // namespace a300_cooling

#endif  // CLEARPATH_HARDWARE_INTERFACES_A300_FAN_CONTROL_HPP
