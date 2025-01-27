/**
Software License Agreement (BSD)

\file      clearpath_diagnostic_updater.hpp
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

#ifndef CLEARPATH_DIAGNOSTIC_UPDATER_HPP
#define CLEARPATH_DIAGNOSTIC_UPDATER_HPP

#include <map>
#include <list>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "clearpath_platform_msgs/msg/status.hpp"

namespace clearpath
{

class ClearpathDiagnosticUpdater : public rclcpp::Node
{
public:
  ClearpathDiagnosticUpdater();

private:
  std::string serial_number_;
  std::string platform_model_;
  std::string namespace_;

  // MCU Status Info
  std::string ros_distro_;  // Specifically the ros distro used for the firmware apt package check
  std::string latest_apt_firmware_version_;
  std::string installed_apt_firmware_version_;
  std::string mcu_firmware_version_;
  std::string mcu_platform_model_;
  double mcu_status_rate_;
  int mcu_temperature_;
  int pcb_temperature_;
  long connection_uptime_;
  long mcu_uptime_;
  std::shared_ptr<diagnostic_updater::FrequencyStatus> mcu_freq_status_;
  rclcpp::Subscription<clearpath_platform_msgs::msg::Status>::SharedPtr sub_mcu_status_;

  diagnostic_updater::Updater updater_;
  std::map<std::string, std::map<std::string, rclcpp::Parameter>> topic_map_;

  // Lists to ensure all variables relating to the rate monitoring persist until spin
  std::list<double> rates_;
  std::list<std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>> topic_diagnostics_;
  std::list<std::shared_ptr<void>> subscriptions_;

  void mcu_callback(const clearpath_platform_msgs::msg::Status & msg);
  void check_firmware_version(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void mcu_status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::string get_mandatory_param(std::string param_name);
  void setup_topic_rate_diagnostics();

  template<class MsgType> void add_rate_diagnostic(const std::string topic_name, const double rate)
  {
    // Store the rate so that it can be accessed via a pointer and is not deleted
    rates_.push_back(rate);

    // Create the diagnostic task object that handles calculating and publishing rate statistics
    auto topic_diagnostic =
      std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
        topic_name,
        updater_,
        diagnostic_updater::FrequencyStatusParam(&rates_.back(), &rates_.back(), 0.1, 5));

    // Store the diagnostic task object so that it can be accessed via a pointer and is not deleted
    topic_diagnostics_.push_back(topic_diagnostic);

    auto sub = this->create_subscription<MsgType>(
      topic_name,
      rclcpp::SensorDataQoS(),
      [this, topic_diagnostic]
      ([[maybe_unused]] const MsgType & msg) {
        topic_diagnostic->tick();
      });
    subscriptions_.push_back(std::static_pointer_cast<void>(sub));
  }
};

}

#endif  // CLEARPATH_DIAGNOSTIC_UPDATER_HPP
