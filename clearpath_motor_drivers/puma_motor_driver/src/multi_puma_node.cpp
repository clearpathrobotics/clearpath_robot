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
#include "puma_motor_driver/multi_puma_node.hpp"

MultiPumaNode::MultiPumaNode(const std::string node_name)
:Node(node_name),
  active_(false),
  status_count_(0),
  desired_mode_(clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED)
{
  // Parameters
  this->declare_parameter("canbus_dev", "vcan0");
  this->declare_parameter("encoder_cpr", 1024);
  this->declare_parameter("frequency", 20);
  this->declare_parameter("gain.p", 0.1);
  this->declare_parameter("gain.i", 0.01);
  this->declare_parameter("gain.d", 0.0);
  this->declare_parameter("gear_ratio", 24.0);
  this->declare_parameter("joint_can_ids", std::vector<int64_t>());
  this->declare_parameter("joint_directions", std::vector<int64_t>());
  this->declare_parameter("joint_names", std::vector<std::string>());

  this->get_parameter("canbus_dev", canbus_dev_);
  this->get_parameter("encoder_cpr", encoder_cpr_);
  this->get_parameter("frequency", freq_);
  this->get_parameter("gain.p", gain_p_);
  this->get_parameter("gain.i", gain_i_);
  this->get_parameter("gain.d", gain_d_);
  this->get_parameter("gear_ratio", gear_ratio_);
  joint_can_ids_ = this->get_parameter("joint_can_ids").as_integer_array();
  joint_directions_ = this->get_parameter("joint_directions").as_integer_array();
  joint_names_ = this->get_parameter("joint_names").as_string_array();

  RCLCPP_INFO(
    this->get_logger(),
    "Gear Ratio %f\nEncoder CPR %d\nFrequency %d\nGain PID %f,%f,%f\nCANBus Device %s",
    gear_ratio_,
    encoder_cpr_,
    freq_,
    gain_p_,
    gain_i_,
    gain_d_,
    canbus_dev_.c_str()
  );

  // Validate Parameters
  if (joint_names_.size() != joint_can_ids_.size() ||
    joint_names_.size() != joint_directions_.size())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Length of joint_name list must match length of joint_can_ids list and joint_directions list.");
    return;
  }

  // Subsciber
  cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "platform/motors/cmd",
    rclcpp::SensorDataQoS(),
    std::bind(&MultiPumaNode::cmdCallback, this, std::placeholders::_1));

  // Publishers
  feedback_pub_ = this->create_publisher<clearpath_motor_msgs::msg::PumaMultiFeedback>(
    "platform/motors/feedback",
    rclcpp::SensorDataQoS());
  status_pub_ = this->create_publisher<clearpath_motor_msgs::msg::PumaMultiStatus>(
    "platform/motors/status",
    rclcpp::SensorDataQoS());

  node_handle_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

  // Socket
  interface_.reset(new clearpath_ros2_socketcan_interface::SocketCANInterface(
    canbus_dev_, node_handle_));

  for (uint8_t i = 0; i < joint_names_.size(); i++) {
    drivers_.push_back(puma_motor_driver::Driver(
      interface_,
      node_handle_,
      joint_can_ids_[i],
      joint_names_[i]
    ));
  }

  recv_msg_.reset(new can_msgs::msg::Frame());
  feedback_msg_.drivers_feedback.resize(drivers_.size());
  status_msg_.drivers.resize(drivers_.size());

  uint8_t i = 0;
  for (auto & driver : drivers_) {
    driver.clearMsgCache();
    driver.setEncoderCPR(encoder_cpr_);
    driver.setGearRatio(gear_ratio_ * joint_directions_[i]);
    driver.setMode(desired_mode_, gain_p_, gain_i_, gain_d_);
    i++;
  }

  run_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / freq_), std::bind(&MultiPumaNode::run, this));
}

bool MultiPumaNode::getFeedback()
{
  // Check All Fields Received
  uint8_t received = 0;
  for (auto & driver : drivers_) {
    received |= driver.receivedDutyCycle() << FeedbackBit::DutyCycle;
    received |= driver.receivedCurrent() << FeedbackBit::Current;
    received |= driver.receivedPosition() << FeedbackBit::Position;
    received |= driver.receivedSpeed() << FeedbackBit::Speed;
    received |= driver.receivedSetpoint() << FeedbackBit::Setpoint;
  }

  if (received != (1 << FeedbackBit::Count) - 1) {
    return false;
  }

  uint8_t feedback_index = 0;
  for (auto & driver : drivers_) {
    clearpath_motor_msgs::msg::PumaFeedback * f = &feedback_msg_.drivers_feedback[feedback_index];
    f->device_number = driver.deviceNumber();
    f->device_name = driver.deviceName();
    f->duty_cycle = driver.lastDutyCycle();
    f->current = driver.lastCurrent();
    f->travel = driver.lastPosition();
    f->speed = driver.lastSpeed();
    f->setpoint = driver.lastSetpoint();

    feedback_index++;
  }
  feedback_msg_.header.stamp = this->get_clock()->now();
  return true;
}

bool MultiPumaNode::getStatus()
{
  // Check All Fields Received
  uint8_t status_index = 0;
  uint8_t received_fields = 0;
  uint8_t received_status = 0;
  for (auto & driver : drivers_) {
    received_fields |= driver.receivedBusVoltage() << StatusBit::BusVoltage;
    received_fields |= driver.receivedOutVoltage() << StatusBit::OutVoltage;
    received_fields |= driver.receivedAnalogInput() << StatusBit::AnalogInput;
    received_fields |= 1 << StatusBit::AnalogInput;
    received_fields |= driver.receivedTemperature() << StatusBit::Temperature;
    received_fields |= driver.receivedMode() << StatusBit::Mode;
    received_fields |= driver.receivedFault() << StatusBit::Fault;
    if (received_fields != (1 << StatusBit::Count) - 1) {
      RCLCPP_DEBUG(this->get_logger(), "Received Status Fields %x", received_fields);
    } else {
      received_status |= 1 << status_index;
    }
    status_index++;
  }

  if (received_status != (1 << status_index) - 1) {
    RCLCPP_DEBUG(this->get_logger(), "Received Status %x", received_status);
    return false;
  }

  // Prepare output status message to ROS.
  status_index = 0;
  for (auto & driver : drivers_) {
    clearpath_motor_msgs::msg::PumaStatus * s = &status_msg_.drivers[status_index];
    s->device_number = driver.deviceNumber();
    s->device_name = driver.deviceName();
    s->bus_voltage = driver.lastBusVoltage();
    s->output_voltage = driver.lastOutVoltage();
    s->analog_input = driver.lastAnalogInput();
    s->temperature = driver.lastTemperature();
    s->mode = driver.lastMode();
    s->fault = driver.lastFault();

    status_index++;
  }
  status_msg_.header.stamp = this->get_clock()->now();
  return true;
}

void MultiPumaNode::publishFeedback()
{
  if (getFeedback()) {
    feedback_pub_->publish(feedback_msg_);
  }
}

void MultiPumaNode::publishStatus()
{
  if (getStatus()) {
    status_pub_->publish(status_msg_);
  }
}

void MultiPumaNode::cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (active_) {
    for (auto & driver : drivers_) {
      for (int i = 0; i < static_cast<int>(msg->name.size()); i++) {
        if (driver.deviceName() == msg->name[i]) {
          if (desired_mode_ == clearpath_motor_msgs::msg::PumaStatus::MODE_VOLTAGE) {
            driver.commandDutyCycle(msg->velocity[i]);
          } else if (desired_mode_ == clearpath_motor_msgs::msg::PumaStatus::MODE_SPEED) {
            driver.commandSpeed(msg->velocity[i]);
          }
        }
      }
    }
  }
}

bool MultiPumaNode::areAllActive()
{
  for (auto & driver : drivers_) {
    if (!driver.isConfigured()) {
      return false;
    }
  }
  return true;
}

void MultiPumaNode::run()
{
  if (active_) {
    // Checks to see if power flag has been reset for each driver
    for (auto & driver : drivers_) {
      if (driver.lastPower() != 0) {
        active_ = false;
        RCLCPP_WARN(this->get_logger(),
          "Power reset detected on device ID %d, will reconfigure all drivers.",
          driver.deviceNumber());
        for (auto & driver : drivers_) {
          driver.resetConfiguration();
        }
      }
    }
    // Queue data requests for the drivers in order to assemble an amalgamated status message.
    for (auto & driver : drivers_) {
      driver.requestStatusMessages();
      driver.requestFeedbackSetpoint();
    }
  } else {
    // Set parameters for each driver.
    for (auto & driver : drivers_) {
      driver.configureParams();
    }
  }

  // Process all received messages through the connected driver instances.
  while (interface_->recv(recv_msg_)) {
    for (auto & driver : drivers_) {
      driver.processMessage(recv_msg_);
    }
  }

  // Check parameters of each driver instance.
  if (!active_) {
    for (auto & driver : drivers_) {
      driver.verifyParams();
    }
  }

  // Verify that the all drivers are configured.
  if (areAllActive() == true && active_ == false) {
    active_ = true;
    RCLCPP_INFO(this->get_logger(), "All controllers active.");
  }
  // Broadcast feedback and status messages
  if (active_) {
    publishFeedback();
    publishStatus();
  }
  status_count_++;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<MultiPumaNode> multi_puma_node =
    std::make_shared<MultiPumaNode>("multi_puma_node");

  exe.add_node(multi_puma_node);
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
