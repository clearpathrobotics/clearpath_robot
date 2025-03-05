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
#ifndef PUMA_MOTOR_DRIVER_DRIVER_H
#define PUMA_MOTOR_DRIVER_DRIVER_H

#include <stdint.h>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

#include "clearpath_motor_msgs/msg/puma_status.hpp"

#include "puma_motor_driver/can_proto.hpp"

namespace puma_motor_driver
{

class Driver
{
public:
  Driver(
    const std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface,
    std::shared_ptr<rclcpp::Node> nh,
    const uint8_t & device_number,
    const std::string & device_name);

  void processMessage(const can_msgs::msg::Frame::SharedPtr received_msg);

  double radPerSecToRpm() const;

  /**
   * Sends messages to the motor controller requesting all missing elements to
   * populate the cache of status data. Returns true if any messages were sent,
   * false if the cache is already complete.
   */
  void requestStatusMessages();

  /**
   * Sends messages to the motor controller requesting all missing elements to
   * populate the cache of feedback data. Returns true if any messages were sent,
   * false if the cache is already complete.
   */
  void requestFeedbackMessages();
  /**
   * Sends a message to the motor controller requesting the instantaneous duty cycle to
   * populate the cache of feedback data.
   */
  void requestFeedbackDutyCycle();
  /**
   * Sends a message to the motor controller requesting the instantaneous current consumption to
   * populate the cache of feedback data.
   */
  void requestFeedbackCurrent();
  /**
   * Sends a message to the motor controller requesting the instantaneous angular distance to
   * populate the cache of feedback data.
   */
  void requestFeedbackPosition();
  /**
   * Sends a message to the motor controller requesting the instantaneous angular speed to
   * populate the cache of feedback data.
   */
  void requestFeedbackSpeed();
  /**
   * Sends a message to the motor controller requesting the state of the power flag.
   */
  void requestFeedbackPowerState();
  /**
   * Sends a message to the motor controller requesting the instantaneous set point of the
   * current control mode to populate the cache of feedback data.
   */
  void requestFeedbackSetpoint();
  /**
   * Clear the received flags from the status cache, in preparation for the next
   * request batch to go out.
   */
  void clearMsgCache();
  /**
   * Command the supplied value in open-loop voltage control.
   *
   * @param[in] cmd Value to command, ranging from -1.0 to 1.0, where zero is neutral.
   */
  void commandDutyCycle(const float cmd);
  /**
   * Command the desired speed set-point in close-loop speed control.
   *
   * @param[in] cmd Value to command in rad/s.
   */
  void commandSpeed(const double cmd);
  // void currentSet(float cmd);
  // void positionSet(float cmd);
  // void neutralSet();

  /**
   * Set the encoders resolution in counts per rev.
   *
   * @param[in] encoder_cpr Value to set.
   */
  void setEncoderCPR(const uint16_t encoder_cpr);
  /**
   * Set the gear ratio of the motors.
   *
   * @param[in] gear_ratio Value to set.
   */
  void setGearRatio(const float gear_ratio);
  /**
   * Set the control mode of the motor drivers.
   *
   * @param[in] mode Value to set.
   */
  void setMode(const uint8_t mode);
  /**
   * Set the control mode of the motor drivers
   * with PID gains for close loop control.
   *
   * @param[in] mode Value to set.
   * @param[in] p Value to set.
   * @param[in] i Value to set.
   * @param[in] d Value to set.
   */
  void setMode(const uint8_t mode, const double p, const double i, const double d);
  /**
   * Set the control mode's PID gains for close loop control.
   *
   * @param[in] p Value to set.
   * @param[in] i Value to set.
   * @param[in] d Value to set.
   */
  void setGains(const double p, const double i, const double d);

  /**
   * Check fault response field was received.
   *
   * @return received flag
  */
  bool receivedFault();
  /**
   * Check power field was received.
   *
   * @return received flag
  */
  bool receivedPower();
  /**
   * Check mode field was received.
   *
   * @return received flag
  */
  bool receivedMode();
  /**
   * Check duty cycle field was received.
   *
   * @return received flag
  */
  bool receivedDutyCycle();
  /**
   * Check bus voltage field was received.
   *
   * @return received flag
  */
  bool receivedBusVoltage();
  /**
   * Check current field was received.
   *
   * @return received flag
  */
  bool receivedCurrent();
  /**
   * Check out voltage field was received.
   *
   * @return received flag
  */
  bool receivedOutVoltage();
  /**
   * Check teperature field was received.
   *
   * @return received flag
  */
  bool receivedTemperature();
  /**
   * Check analog input field was received.
   *
   * @return received flag
  */
  bool receivedAnalogInput();
  /**
   * Check position field was received.
   *
   * @return received flag
  */
  bool receivedPosition();
  /**
   * Check speed field was received.
   *
   * @return received flag
  */
  bool receivedSpeed();
  /**
   * Check setpoint field was received.
   *
   * @return received flag
  */
  bool receivedSetpoint();
  /**
   * Check the set-point response in voltage
   * open-loop control was received.
   *
   * @return received flag
   */
  bool receivedDutyCycleSetpoint();
  /**
   * Check the set-point response in speed
   * closed-loop control was received.
   *
   * @return received flag
   */
  bool receivedSpeedSetpoint();
  /**
   * Check the set-point response in currrent
   * closed-loop control was received.
   *
   * @return received flag
   */
  bool receivedCurrentSetpoint();
  /**
   * Check the set-point response in position
   * closed-loop control was received.
   *
   * @return received flag
   */
  bool receivedPositionSetpoint();
  /**
   * Process the last received fault response.
   *
   * @return state of fault status.
   */
  uint8_t lastFault();
  /**
   * Process the last received power response.
   *
   * @return state of power status.
   */
  uint8_t lastPower();
  /**
   * Process the last received mode response.
   *
   * @return current mode of motor driver.
   */
  uint8_t lastMode();
  /**
   * Process the last received duty cycle response.
   *
   * @return value of the instantaneous duty cycle.
   */
  float lastDutyCycle();
  /**
   * Process the last received bus voltage response.
   *
   * @return value of the instantaneous bus voltage.
   */
  float lastBusVoltage();
  /**
   * Process the last received current response.
   *
   * @return value of the instantaneous current.
   */
  float lastCurrent();
  /**
   * Process the last received out voltage response.
   *
   * @return value of the instantaneous out voltage.
   */
  float lastOutVoltage();
  /**
   * Process the last received temperature response.
   *
   * @return value of the instantaneous temperature.
   */
  float lastTemperature();
  /**
   * Process the last received analog_input response.
   *
   * @return value of the instantaneous analog_input.
   */
  float lastAnalogInput();
  /**
   * Process the last received travel response.
   *
   * @return value of the instantaneous angular position.
   */
  double lastPosition();
  /**
   * Process the last received speed response.
   *
   * @return value of the instantaneous angular speed.
   */
  double lastSpeed();
  /**
   * Process the last received set-point response
   * for the current control mode.
   *
   * @return value of the set-point response.
   */
  double lastSetpoint();

  /**
   * The requesting part of the state machine that sends a message to the
   * motor controller requesting a parameter be set.
   */
  void configureParams();
  /**
   * The verifying part of the state machine that checks the response of
   * the motor controller to ensure the value was set.
   */
  void verifyParams();
  /**
   * Gets if the driver has been configured.
   *
   * @return bool if driver is configured.
   */
  bool isConfigured() const;
  void resetConfiguration();
  /**
   * Reset the configured flag to restart the verification process.
   */
  void updateGains();
  /**
   * Updates the PID gains.
   */

  /**
   * Process the last received position encoder reference response
   *
   * @return value of the reference response.
   */
  uint8_t posEncoderRef();
  /**
   * Process the last received speed encoder reference response
   *
   * @return value of the reference response.
   */
  uint8_t spdEncoderRef();
  /**
   * Process the last received encoder counts response
   *
   * @return value of the encoder counts.
   */
  uint16_t encoderCounts();

  /**
   * Process the last received P gain
   * for the current control mode.
   *
   * @return value of the P gain response.
   */
  double getP();
  /**
   * Process the last received I gain
   * for the current control mode.
   *
   * @return value of the I gain response.
   */
  double getI();
  /**
   * Process the last received D gain
   * for the current control mode.
   *
   * @return value of the D gain response.
   */
  double getD();
  /**
   * Process the last received P gain
   * for the current control mode.
   *
   * @return pointer to raw 4 bytes of the P gain response.
   */
  uint8_t * getRawP();
  /**
   * Process the last received I gain
   * for the current control mode.
   *
   * @return pointer to raw 4 bytes of the I gain response.
   */
  uint8_t * getRawI();
  /**
   * Process the last received I gain
   * for the current control mode.
   *
   * @return pointer to raw 4 bytes of the I gain response.
   */
  uint8_t * getRawD();
  /**
   * Process the last received set-point response
   * in voltage open-loop control.
   *
   * @return value of the set-point response.
   */
  float statusDutyCycleGet();
  /**
   * Process the last received set-point response
   * in speed closed-loop control.
   *
   * @return value of the set-point response.
   */
  double statusSpeedGet();
  /**
   * Process the last received set-point response
   * in currrent closed-loop control.
   *
   * @return value of the set-point response.
   */
  float statusCurrentGet();
  /**
   * Process the last received set-point response
   * in position closed-loop control.
   *
   * @return value of the set-point response.
   */
  double statusPositionGet();

  std::string deviceName() const {return device_name_;}

  uint8_t deviceNumber() const {return device_number_;}

  struct Field
  {
    static constexpr int FIELD_STRUCT_DATA_SIZE = 4;
    uint8_t data[FIELD_STRUCT_DATA_SIZE];
    bool received;

    float interpretFixed8x8()
    {
      return *(reinterpret_cast<int16_t *>(data)) / static_cast<float>(1 << 8);
    }

    double interpretFixed16x16()
    {
      return *(reinterpret_cast<int32_t *>(data)) / static_cast<double>(1 << 16);
    }
  };

private:
  std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> interface_;
  std::shared_ptr<rclcpp::Node> nh_;
  uint8_t device_number_;
  std::string device_name_;

  bool configured_;
  uint8_t state_;

  uint8_t control_mode_;
  double gain_p_;
  double gain_i_;
  double gain_d_;
  uint16_t encoder_cpr_;
  float gear_ratio_;

  /**
   * Helpers to generate data for CAN messages.
   */
  can_msgs::msg::Frame::SharedPtr can_msg_;
  void sendId(const uint32_t id);
  void sendUint8(const uint32_t id, const uint8_t value);
  void sendUint16(const uint32_t id, const uint16_t value);
  void sendFixed8x8(const uint32_t id, const float value);
  void sendFixed16x16(const uint32_t id, const double value);
  can_msgs::msg::Frame getMsg(const uint32_t id);
  uint32_t getApi(const can_msgs::msg::Frame msg);
  uint32_t getDeviceNumber(const can_msgs::msg::Frame msg);

  /**
   * Comparing the raw bytes of the 16x16 fixed-point numbers
    * to avoid comparing the floating point values.
   *
   * @return boolean if received is equal to expected.
   */
  bool verifyRaw16x16(const uint8_t * received, const double expected);

  /**
   * Comparing the raw bytes of the 8x8 fixed-point numbers
    * to avoid comparing the floating point values.
   *
   * @return boolean if received is equal to expected.
   */
  bool verifyRaw8x8(const uint8_t * received, const float expected);

  Field voltage_fields_[4];
  Field spd_fields_[7];
  Field vcomp_fields_[5];
  Field pos_fields_[7];
  Field ictrl_fields_[6];
  Field status_fields_[15];
  Field cfg_fields_[15];

  Field * voltageFieldForMessage(uint32_t api);
  Field * spdFieldForMessage(uint32_t api);
  Field * vcompFieldForMessage(uint32_t api);
  Field * posFieldForMessage(uint32_t api);
  Field * ictrlFieldForMessage(uint32_t api);
  Field * statusFieldForMessage(uint32_t api);
  Field * cfgFieldForMessage(uint32_t api);
};

}  // namespace puma_motor_driver

#endif  // PUMA_MOTOR_DRIVER_DRIVER_H
