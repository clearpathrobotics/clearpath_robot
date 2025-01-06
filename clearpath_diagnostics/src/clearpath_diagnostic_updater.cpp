/**
Software License Agreement (BSD)

\file      clearpath_diagnostic_updater.cpp
\authors   Hilary Luo <hluo@clearpathrobotics.com>
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

#include "clearpath_diagnostics/clearpath_diagnostic_updater.hpp"

namespace clearpath
{

ClearpathDiagnosticUpdater::ClearpathDiagnosticUpdater()
: Node(
    "clearpath_diagnostic_updater",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(true)),
  updater_(this)  // Create the diagnostic updater object
{
  serial_number_ = this->get_mandatory_param("serial_number");
  platform_model_ = this->get_mandatory_param("platform_model");

  // Set Hardware ID as serial number in diagnostics
  updater_.setHardwareID(serial_number_);
  updater_.add("MCU Status", this, &ClearpathDiagnosticUpdater::mcu_status_diagnostic);

  // subscribe to MCU status
  sub_mcu_status_ =
    this->create_subscription<clearpath_platform_msgs::msg::Status>(
      "platform/mcu/status",
      rclcpp::SensorDataQoS(),
      std::bind(&ClearpathDiagnosticUpdater::mcu_callback, this, std::placeholders::_1));

  setup_topic_rate_diagnostics();
}

std::string ClearpathDiagnosticUpdater::get_mandatory_param(std::string param_name)
{
  try {
    return this->get_parameter(param_name).as_string();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Could not retrieve parameter %s: %s",
                                      param_name.c_str(), e.what());
    return "unknown";
  }
}

void ClearpathDiagnosticUpdater::mcu_callback(const clearpath_platform_msgs::msg::Status & msg)
{
  firmware_version_ = msg.firmware_version;
  mcu_hardware_id_ = msg.hardware_id;
  mcu_uptime_ = msg.mcu_uptime.sec;
  connection_uptime_ = msg.connection_uptime.sec;
  mcu_temperature_ = msg.mcu_temperature;
  pcb_temperature_ = msg.pcb_temperature;
}

void ClearpathDiagnosticUpdater::mcu_status_diagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // add to append key-value pairs to the diagnostic
  stat.add("Firmware Version", firmware_version_);
  stat.add("Hardware ID", mcu_hardware_id_);
  stat.add("MCU Uptime", mcu_uptime_);
  stat.add("Connection Uptime", connection_uptime_);
  stat.add("MCU Temperature", mcu_temperature_);
  stat.add("PCB Temperature", pcb_temperature_);
}

void ClearpathDiagnosticUpdater::setup_topic_rate_diagnostics()
{
  std::map<std::string, rclcpp::Parameter> topic_map_raw;

  // Get all parameters under the "topics" key in the yaml and store it in map format
  if (!this->get_parameters("topics", topic_map_raw)) {
    RCLCPP_WARN(this->get_logger(), "No topics found to monitor.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Retrieved %zu topic parameters.", topic_map_raw.size());

  // Parse the raw topic map into a map of parameters per topic, stored in another map with
  // the topic as the key
  for (const auto & entry : topic_map_raw) {

    // Identify the topic name and parameter name in the entry key
    auto key = entry.first;  // Contains topic_name.param_name
    auto pos = key.find(".");
    auto topic_name = key.substr(0, pos);
    auto param_name = key.substr(pos + 1);

    // RCLCPP_INFO(this->get_logger(),
    //             "Diagnostics config: Topic is %s, parameter %s = %s.",
    //             topic_name.c_str(), param_name.c_str(), entry.second.value_to_string().c_str());

    // Add the parameter to the topic specific map
    if (auto it = topic_map_.find(topic_name); it != topic_map_.end()) {
      // Topic already exists as a key
      it->second[param_name] = entry.second;
    } else {
      // Topic needs to be added as a new key
      std::map<std::string, rclcpp::Parameter> map;
      map[param_name] = entry.second;
      topic_map_[topic_name] = map;
    }
  }

  // For each topic, create a subscription that monitors the publishing frequency
  for (const auto & topic : topic_map_) {
    auto topic_name = topic.first;
    RCLCPP_INFO(this->get_logger(), "Diagnostics config: Topic is %s", topic_name.c_str());
    auto params_map = topic.second;

    for(const auto & param : params_map) {
      RCLCPP_INFO(this->get_logger(), "  Param: %s = %s",
                  param.first.c_str(), param.second.value_to_string().c_str());
    }

    // Get the message type
    std::string type;
    try {
      type = params_map.at("type").as_string();
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(), "No type provided for %s. This topic will not be monitored",
                   topic_name.c_str());
      continue;
    } catch(const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Type provided for %s was not a valid string. Value is %s. \
                   This topic will not be monitored",
                   topic_name.c_str(), params_map.at("type").value_to_string().c_str());
      continue;
    }

    // Get the expected publishing rate
    double rate = 0.0;
    try {
      rate = params_map.at("rate").as_double();
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(this->get_logger(), "No rate provided for %s. This topic will not be monitored",
                   topic_name.c_str());
      continue;
    } catch(const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Rate provided for %s was not a valid double. Value is %s. \
                   This topic will not be monitored",
                   topic_name.c_str(), params_map.at("rate").value_to_string().c_str());
      continue;
    }

    /*
    The section below does not use a generic subscription because the generic subscription was
    observed to have significantly hgiher CPU usage seemingly related to too short of callbacks
    and allocating / releasing the memory too quickly with Fast DDS. Standard subscriptions
    perform more reliably.
    */

    // Create a subscription using the topic name and message type info from the yaml
    if (type == "sensor_msgs/msg/CompressedImage") {
      add_rate_diagnostic<sensor_msgs::msg::CompressedImage>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/Image") {
      add_rate_diagnostic<sensor_msgs::msg::Image>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/Imu") {
      add_rate_diagnostic<sensor_msgs::msg::Imu>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/LaserScan") {
      add_rate_diagnostic<sensor_msgs::msg::LaserScan>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/MagneticField") {
      add_rate_diagnostic<sensor_msgs::msg::MagneticField>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/NavSatFix") {
      add_rate_diagnostic<sensor_msgs::msg::NavSatFix>(topic_name, rate);
    } else if (type == "sensor_msgs/msg/PointCloud2") {
      add_rate_diagnostic<sensor_msgs::msg::PointCloud2>(topic_name, rate);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Type \"%s\" of topic \"%s\" is not supported",
                   type.c_str(),
                   topic_name.c_str());
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Created subscription for %s", topic_name.c_str());
  }
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<clearpath::ClearpathDiagnosticUpdater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
