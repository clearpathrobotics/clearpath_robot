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
#include <queue>
#include <mutex>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include "can_hardware/common/types.hpp"
#include "can_hardware/adaptors/can/can_adaptor.hpp"


#include "clearpath_motor_msgs/msg/puma_status.hpp"

#include "diagnostic_updater/update_functions.hpp"

#include "puma_motor_driver/can_proto.hpp"
#include "puma_motor_driver/can_interface.hpp"

namespace puma_motor_driver
{

class Driver : public can_hardware::adaptor::CanAdaptor
{
public:
  Driver(
    CanConnection* conn,
    std::shared_ptr<rclcpp::Node> nh,
    const uint8_t & device_number,
    const std::string & device_name);

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
   * @return Value of the P gain response.
   */
  double getRawP();
  /**
   * Process the last received I gain
   * for the current control mode.
   *
   * @return Value of the I gain response.
   */
  double getRawI();
  /**
   * Process the last received I gain
   * for the current control mode.
   *
   * @return Value of the I gain response.
   */
  double getRawD();
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

  // Diagnostics
  void runFreqStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void driverUpdateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat, bool updating);

  can_hardware::Frame createRequestReadFrame(const can_hardware::CanId id) override
  {
    // Override the CAN adaptor's default read request frame creation that uses RTR frames.

    can_hardware::Frame frame;
    frame.id = id;
    frame.is_extended = true;
    frame.dlc = 0;
    return frame;
  }

private:
  std::shared_ptr<rclcpp::Node> nh_;
  uint8_t device_number_;
  std::string device_name_;

  bool configured_;
  uint8_t state_;
  double last_power_clear_ts_;

  uint8_t control_mode_;
  double gain_p_;
  double gain_i_;
  double gain_d_;
  uint16_t encoder_cpr_;
  float gear_ratio_;

  std::queue<can_hardware::Frame> message_queue_;
  std::mutex message_queue_mutex_;
  std::set<std::string> signals_;

  template<typename T>
  class Signal
  {
  public:
    using Ptr = std::unique_ptr<Signal<T>>;

    Signal() = default;

    Signal(const std::string& name, SignalAdaptor& adaptor)
      : name(name), adaptor(adaptor)
    {
    }
    ~Signal() = default;

    void requestRead()
    {
      adaptor.get().requestRead(name);
    }

    void requestWrite(const T& value)
    {
      adaptor.get().requestWrite(name, value);
    }

    T value() const
    {
      return adaptor.get().template getSignalAs<T>(name);
    }

    bool received() const
    {
      return !hal::is_none(adaptor.get().getSignal(name));
    }

    bool isNone() const
    {
      return hal::is_none(adaptor.get().getSignal(name));
    }

  public:
    //! The name of the signal
    std::string name;
    
  private:
    //! Reference to the signal adaptor
    std::reference_wrapper<SignalAdaptor> adaptor;
  };

  template<typename T>
  Signal<T>::Ptr registerSignal(const std::string& name, const can_hardware::CanId id, uint8_t device_number)
  {
    constexpr uint32_t bit_offset = 0;
    constexpr uint32_t byte_length = 4;
    constexpr uint32_t bit_length = byte_length * 8;

    if constexpr (std::is_integral<T>::value && std::is_unsigned<T>::value)
    {
      addSignal<T>(name, bit_offset, bit_length, false, 1.0, 0, std::endian::little);
    }
    else if constexpr (std::is_integral<T>::value && std::is_signed<T>::value)
    {
      addSignal<T>(name, bit_offset, bit_length, true, 1.0, 0, std::endian::little);
    }
    else if constexpr (std::is_same<T, double>::value)
    {
      // Fixed-point 16x16 representation
      addSignal<T>(name, bit_offset, bit_length, true, 1.0 / static_cast<double>(1 << 16), 0, std::endian::little);
    }
    else if constexpr (std::is_same<T, float>::value)
    {
      // Fixed-point 8x8 representation
      addSignal<T>(name, bit_offset, bit_length, true, 1.0 / static_cast<float>(1 << 8), 0, std::endian::little);
    }
    else
    {
      // Trigger a compile-time error for unsupported types
      static_assert(sizeof(T) == 0, "Unsupported signal type");
    }

    const can_hardware::CanId full_id = id | (device_number & CAN_MSGID_DEVNO_M);
    // Register the signal to a CAN frame with it associated CAN ID
    addRxSignals(full_id, {name});
    // Register the signal for on request transmission (rate=0.0)
    addTxSignals(full_id, byte_length, true, false, {name}, 0.0);

    signals_.insert(name);

    return std::make_unique<Signal<T>>(name, *this);
  }


  bool verify(const double recv, const double expected) const
  {
    return std::abs(recv - expected) < 1e-4;  // Allow for fixed-point quantization
  }

  // Voltage control Signals
  Signal<uint8_t>::Ptr voltage_enable_;
  Signal<uint8_t>::Ptr voltage_disable_;
  Signal<float>::Ptr voltage_set_;
  Signal<float>::Ptr voltage_set_ramp_;

  // Status Signals
  Signal<float>::Ptr status_voltage_out_;
  Signal<float>::Ptr status_voltage_bus_;
  Signal<float>::Ptr status_current_;
  Signal<double>::Ptr status_speed_;
  Signal<uint32_t>::Ptr status_limit_;
  Signal<float>::Ptr status_temperature_;
  Signal<double>::Ptr status_position_;
  Signal<uint8_t>::Ptr status_power_;
  Signal<uint8_t>::Ptr status_control_mode_;
  Signal<float>::Ptr status_vout_;
  Signal<float>::Ptr status_analog_;
  Signal<uint8_t>::Ptr status_fault_;
  Signal<uint32_t>::Ptr status_sticky_fault_;
  Signal<uint32_t>::Ptr status_fault_count_;

  // Position Control Signals
  Signal<uint32_t>::Ptr position_enable_;
  Signal<uint32_t>::Ptr position_disable_;
  Signal<double>::Ptr position_set_;
  Signal<uint8_t>::Ptr position_ref_;
  Signal<double>::Ptr position_pc_;
  Signal<double>::Ptr position_ic_;
  Signal<double>::Ptr position_dc_;
  // Speed Control Signals
  Signal<uint8_t>::Ptr speed_enable_;
  Signal<uint8_t>::Ptr speed_disable_;
  Signal<double>::Ptr speed_set_;
  Signal<double>::Ptr speed_pc_;
  Signal<double>::Ptr speed_ic_;
  Signal<double>::Ptr speed_dc_;
  Signal<uint8_t>::Ptr speed_ref_;
  // Current Control Signals
  Signal<uint32_t>::Ptr ictrl_enable_;
  Signal<uint32_t>::Ptr ictrl_disable_;
  Signal<float>::Ptr ictrl_set_;
  Signal<double>::Ptr ictrl_pc_;
  Signal<double>::Ptr ictrl_ic_;
  Signal<double>::Ptr ictrl_dc_;
  // Configuration Signals
  Signal<uint16_t>::Ptr cfg_enc_lines_;

  // Frequency Status for diagnostics
  std::shared_ptr<double> can_feedback_rate_; // Shared ptr prevents copy errors of FrequencyStatus
  std::shared_ptr<diagnostic_updater::FrequencyStatus> can_feedback_freq_status_;
};

}  // namespace puma_motor_driver

#endif  // PUMA_MOTOR_DRIVER_DRIVER_H
