/**
Software License Agreement (BSD)

\authors   Roni Kreinin <rkreinin@clearpathrobotics.com>
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

#include "lynx_motor_driver/lynx_motor_driver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <cstring>
#include <math.h>


namespace lynx_motor_driver
{

/**
 * @brief Construct a new Lynx Motor Driver object
 * 
 * @param can_id CAN ID
 * @param joint_name Joint name
 * @param direction Direction of rotation
 * @param can_interface SocketCANInterface shared pointer
 */
LynxMotorDriver::LynxMotorDriver(const int64_t& can_id,
               const std::string& joint_name,
               const int64_t direction,
               std::shared_ptr<clearpath_ros2_socketcan_interface::SocketCANInterface> can_interface)
  : can_id_(can_id), joint_name_(joint_name), direction_(direction),
    protection_state_(clearpath_motor_msgs::msg::LynxSystemProtection::NORMAL), debug_(false),
    can_interface_(can_interface)
{
  for (auto & m : debug_mutexes_)
  {
    m = new std::mutex();
    m->lock();
  }

  for (auto & m : feedback_mutexes_)
  {
    m = new std::mutex();
    m->lock();
  }

  for (auto & m : status_mutexes_)
  {
    m = new std::mutex();
    m->lock();
  }

  for (auto & m : action_mutexes_)
  {
    m = new std::mutex();
    m->lock();
  }

  debug_msg_.can_id = getCanID();
  debug_msg_.joint_name = getJointName();
  debug_msg_.accumulators.resize(6);
  debug_msg_.currents.resize(6);

  feedback_msg_.can_id = getCanID();
  feedback_msg_.joint_name = getJointName();

  status_msg_.can_id = getCanID();
  status_msg_.joint_name = getJointName();
}

/**
 * @brief Process a received CAN message
 * 
 * @param received_msg Message reference
 * @return true if processed by this driver.
 * @return false otherwise.
 */
bool LynxMotorDriver::processMessage(const Message& received_msg)
{
  // If it's not our message, jump out.
  if (received_msg.getID() != can_id_) return false;

  // If there's no data then this is a request message, jump out.
  if (received_msg.getLength() == 0) return false;

  // Handle message based on API
  switch(received_msg.getApi())
  {
    case CAN_MSGID_FEEDBACK:
    {
      auto message_pair = received_msg.getDataAsIndexedFloat();
      uint8_t index = std::get<uint8_t>(message_pair);
      float data = std::get<float>(message_pair);

      switch(index)
      {
        case Feedback::Fields::Current:
        {
          feedback_msg_.current = data;
          break;
        }

        case Feedback::Fields::Voltage:
        {
          feedback_msg_.voltage = data;
          break;
        }

        case Feedback::Fields::Velocity:
        {
          feedback_msg_.velocity = data * direction_;
          break;
        }
      }

      feedback_mutexes_[index]->unlock();
      break;
    }

    case CAN_MSGID_STATUS:
    {
      uint8_t index = received_msg.getDataAsUint8();

      switch(index)
      {
        case Status::Fields::Version:
        {
          uint32_t version = std::get<uint32_t>(received_msg.getDataAsIndexedUint32());
          bool debug = (version & 0xFF);

          status_msg_.firmware_version = 
            std::to_string((version >> 24) & 0xFF) +
            "." +
            std::to_string((version >> 16) & 0xFF) +
            "." +
            std::to_string((version >> 8) & 0xFF);
          
          if (debug)
          {
            status_msg_.firmware_version += " Debug";
          }
          else
          {
            status_msg_.firmware_version += " Release";
          }

          debug_ = debug;
          break;
        }

        case Status::Fields::FlagsStatus:
        {
          status_msg_.status_flags = std::get<uint32_t>(received_msg.getDataAsIndexedUint32());
          status_msg_.stopped = status_msg_.status_flags & (1 << clearpath_motor_msgs::msg::LynxStatus::STATUS_FLAG_ESTOPPED);
          break;
        }

        case Status::Fields::FlagsError:
        {
          status_msg_.error_flags = std::get<uint32_t>(received_msg.getDataAsIndexedUint32());
          break;
        }

        case Status::Fields::MotorTemperature:
        {
          status_msg_.motor_temperature = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Status::Fields::McuTemperature:
        {
          status_msg_.mcu_temperature = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Status::Fields::PcbTemperature:
        {
          status_msg_.pcb_temperature = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }
      }

      status_mutexes_[index]->unlock();
      break;
    }

    case CAN_MSGID_DEBUG:
    {
      uint8_t index = received_msg.getDataAsUint8();

      switch(index)
      {
        case Debug::Fields::FaultFrequency:
        {
          debug_msg_.fault_frequency = std::get<uint32_t>(received_msg.getDataAsIndexedUint32());
          break;
        }

        case Debug::Fields::FlagsAccumulator:
        {
          debug_msg_.accumulator_flags = std::get<uint32_t>(received_msg.getDataAsIndexedUint32());
          break;
        }

        case Debug::Fields::AccumulatorAboveRated:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_ABOVE_RATED] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::Accumulator1_1:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_CONTINUOUS_1_1] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::Accumulator1_5:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_CONTINUOUS_1_5] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::Accumulator2_0:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_CONTINUOUS_2_0] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::Accumulator2_5:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_CONTINUOUS_2_5] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::Accumulator3_0:
        {
          debug_msg_.accumulators[clearpath_motor_msgs::msg::LynxDebug::ACCUMULATOR_CONTINUOUS_3_0] = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIDFB:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_ID_FB) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIDFF:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_ID_FF) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIDREF:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_ID_REF) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIQFB:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_IQ_FB) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIQFF:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_IQ_FF) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }

        case Debug::Fields::CurrentIQREF:
        {
          debug_msg_.currents.at(clearpath_motor_msgs::msg::LynxDebug::CURRENT_IQ_REF) = std::get<float>(received_msg.getDataAsIndexedFloat());
          break;
        }
      }

      debug_mutexes_[index]->unlock();
      break;
    }

    case CAN_MSGID_PROTECTION_MOTOR_STATE:
    {
      protection_state_ = received_msg.getDataAsUint8();
      break;
    }

    case CAN_MSGID_CALIBRATION_ITERATION:
    {
      iteration_ = received_msg.getDataAsUint16();
      action_mutexes_[Action::Fields::CalibrationIteration]->unlock();
      break;
    }

    case CAN_MSGID_CALIBRATION_OFFSET:
    {
      offset_ = received_msg.getDataAsFloat(0);
      action_mutexes_[Action::Fields::CalibrationOffset]->unlock();
      break;
    }

    case CAN_MSGID_BOOT_RESP:
    {
      uint16_t resp = received_msg.getDataAsUint16();
      if (resp == 0)
      {
        action_mutexes_[Action::Fields::UpdateAlive]->unlock();
      }
      else
      {
        can_count_ = resp;
        action_mutexes_[Action::Fields::UpdateAck]->unlock();
      }
      break;
    }

    default:
    {
      break;
    }
  }

  return true;
}

/**
 * @brief Check if debug message has been filled
 * 
 * @return true if message is ready.
 * @return false otherwise.
 */
bool LynxMotorDriver::debugMessageReady()
{
  for (auto & m : debug_mutexes_)
  {
    if (!m->try_lock())
    {
      return false;
    }
    else
    {
      m->unlock();
    }
  }

  return true;
}

/**
 * @brief Check if feedback message has been filled
 * 
 * @return true if message is ready.
 * @return false otherwise.
 */
bool LynxMotorDriver::feedbackMessageReady()
{
  for (auto & m : feedback_mutexes_)
  {
    if (!m->try_lock())
    {
      return false;
    }
    else
    {
      m->unlock();
    }
  }

  return true;
}

/**
 * @brief Check if status message has been filled
 * 
 * @return true if message is ready.
 * @return false otherwise.
 */
bool LynxMotorDriver::statusMessageReady()
{
  for (auto & m : status_mutexes_)
  {
    if (!m->try_lock())
    {
      return false;
    }
    else
    {
      m->unlock();
    }
  }

  return true;
}

/**
 * @brief Get the debug message
 * 
 * @return clearpath_motor_msgs::msg::LynxDebug 
 */
clearpath_motor_msgs::msg::LynxDebug LynxMotorDriver::getDebugMessage()
{
  for (auto & m : debug_mutexes_)
  {
    (void)m->try_lock();
  }

  return debug_msg_;
}

/**
 * @brief Get the feedback message
 * 
 * @return clearpath_motor_msgs::msg::LynxFeedback 
 */
clearpath_motor_msgs::msg::LynxFeedback LynxMotorDriver::getFeedbackMessage()
{
  for (auto & m : feedback_mutexes_)
  {
    (void)m->try_lock();
  }

  return feedback_msg_;
}

/**
 * @brief Get the status message
 * 
 * @return clearpath_motor_msgs::msg::LynxStatus 
 */
clearpath_motor_msgs::msg::LynxStatus LynxMotorDriver::getStatusMessage()
{
  for (auto & m : status_mutexes_)
  {
    (void)m->try_lock();
  }

  return status_msg_;
}

/**
 * @brief Send a velocity to the driver
 * 
 * @param velocity Velocity in rad/s
 */
void LynxMotorDriver::sendVelocity(double velocity)
{
  send(CAN_MSGID_VELOCITY_DEMAND, static_cast<float>(velocity * direction_));
}

/**
 * @brief Send the system protection state to the driver
 * 
 * @param state System protection state
 */
void LynxMotorDriver::sendProtectionState(uint8_t state)
{
  send(CAN_MSGID_PROTECTION_SYSTEM_STATE, state);
}

/**
 * @brief Get the driver protection state
 * 
 * @return uint8_t representing the protection state
 */
uint8_t LynxMotorDriver::getProtectionState()
{
  return protection_state_;
}

/**
 * @brief Send a calibration request to the driver
 * 
 */
void LynxMotorDriver::sendCalibrationRequest()
{
  send(CAN_MSGID_CALIBRATION_REQUEST);
}

/**
 * @brief Cancel calibration on the driver
 * 
 */
void LynxMotorDriver::sendCalibrationCancel()
{
  send(CAN_MSGID_CALIBRATION_CANCEL);
}

/**
 * @brief Get the COBID from a message ID and the driver's CAN ID
 * 
 * @param msg_id message ID
 * @return uint32_t representing the COBID
 */
uint32_t LynxMotorDriver::getCOBID(const uint32_t msg_id) const
{
  return (((msg_id << CAN_MSGID_API_SHIFT) & CAN_MSGID_API_MASK) | ((can_id_ << CAN_MSGID_CANID_SHIFT) & CAN_MSGID_CANID_MASK)) & CAN_MSGID_FULL_MASK;
}

/**
 * @brief Send a message to the driver. Converts from DataT to an 8 byte CAN data array.
 * 
 * @tparam DataT Data type (Up to 8 bytes long)
 * @param id Message ID
 * @param value Data value
 */
template <typename DataT>
void LynxMotorDriver::send(const uint32_t id, const DataT value)
{
  if (sizeof(DataT) > 8)
  {
    std::cerr << "Invalid Data type " << std::endl;
    return;
  }

  // Create frame message
  can_msgs::msg::Frame frame;
  frame.id = getCOBID(id);
  frame.dlc = sizeof(DataT);
  frame.is_extended = true;

  // Copy data into 8 byte array
  uint8_t data[8] = {0};
  std::memcpy(data, &value, sizeof(DataT));
  std::copy(std::begin(data), std::end(data), std::begin(frame.data));
  
  // Send frame to CAN interface
  can_interface_->queue(frame);
}

/**
 * @brief Send a message to the driver with no data.
 * 
 * @param id Message ID
 */
void LynxMotorDriver::send(const uint32_t id)
{
  // Create frame message
  can_msgs::msg::Frame frame;
  frame.id = getCOBID(id);
  frame.dlc = 0;
  frame.is_extended = true;
  
  // Send frame to CAN interface
  can_interface_->queue(frame);
}

/**
 * @brief Send a message to the driver with data from a buffer.
 * 
 * @param id Message ID
 * @param data Data buffer
 * @param length Data buffer length
 */
void LynxMotorDriver::send(const uint32_t id, uint8_t * data, uint8_t length)
{
  // Max 8 bytes
  if (length > 8)
  {
    std::cerr << "Invalid Data type " << std::endl;
    return;
  }

  // Create frame message
  can_msgs::msg::Frame frame;
  frame.id = getCOBID(id);
  frame.dlc = length;
  frame.is_extended = true;

  // Copy data into 8 byte array
  uint8_t buffer[8] = {0};
  std::memcpy(buffer, data, length);
  std::copy(std::begin(buffer), std::end(buffer), std::begin(frame.data));
  
  // Send frame to CAN interface
  can_interface_->queue(frame);
}

/**
 * @brief Send a boot request to the driver.
 * 
 */
void LynxMotorDriver::sendBootRequest()
{
  send(CAN_MSGID_BOOT_REQ);
}

/**
 * @brief Send a boot alive check to the driver.
 * 
 */
void LynxMotorDriver::sendBootAliveCheck()
{
  send(CAN_MSGID_BOOT_ALIVE);
}

void LynxMotorDriver::updateReset()
{
  uint16_t temp;
  (void)tryGetUpdateAck(temp);
  (void)tryGetUpdateAlive();
  can_count_ = 0;
  app_count_ = 0;
}

/**
 * @brief Copy the application to the driver.
 * 
 * @param app Queue of bytes representing the application
 */
void LynxMotorDriver::copyApplication(const std::queue<uint8_t> app)
{
  update_app_queue_ = app;
  update_app_size_ = update_app_queue_.size();
}

/**
 * @brief Run an iteration of a CAN bootloader update.
 * 
 * @return float representing update progress as a percentage.
 */
float LynxMotorDriver::updateApp()
{
  uint16_t can_count;
  uint8_t frame_data[8];

  if (update_app_queue_.size() >= 8)
  {
    // Send next 8 bytes
    for (int i = 0; i < 8; i++)
    {
      frame_data[i] = update_app_queue_.front();
      update_app_queue_.pop();
    }

    app_count_++;

    do
    {
      send(CAN_MSGID_BOOT_DATA, frame_data, 8);
      // Wait for ACK
      getUpdateAck(can_count);
    } while (can_count < app_count_);
  }
  else // Send remaining bytes
  {
    int last_size = update_app_queue_.size();
    for (int i = 0; i < 8; i++)
    {
      if (i < last_size)
      {
        frame_data[i] = update_app_queue_.front();
        update_app_queue_.pop();
      }
      else // Fill remaining data with 0's
      {
        frame_data[i] = 0x00;
      }
    }
  
    app_count_++;

    do
    {
      send(CAN_MSGID_BOOT_DATA, frame_data, 8);
      // Wait for ACK
      getUpdateAck(can_count);
    } while (can_count < app_count_);
  }

  // Calculate progress
  float progress = 1.0f - static_cast<float>(update_app_queue_.size()) / static_cast<float>(update_app_size_);

  return progress;
}

/**
 * @brief Attempt to acquire the iteration mutex.
 * If acquired, assign the iteration value to the iteration parameter.
 * 
 * @param iteration Reference to iteration variable.
 * @return true on success.
 * @return false otherwise.
 */
bool LynxMotorDriver::tryGetIteration(uint16_t & iteration)
{
  if (action_mutexes_[Action::Fields::CalibrationIteration]->try_lock())
  {
    iteration = iteration_;
    return true;
  }

  return false;
}

/**
 * @brief Attempt to acquire the offset mutex.
 * If acquired, assign the offset value to the offset parameter.
 * 
 * @param offset Reference to offset variable.
 * @return true on success.
 * @return false otherwise.
 */
bool LynxMotorDriver::tryGetOffset(float & offset)
{
  if (action_mutexes_[Action::Fields::CalibrationOffset]->try_lock())
  {
    offset = offset_;
    return true;
  }

  return false;
}

/**
 * @brief Attempt to acquire the update ack mutex.
 * 
 * @return true if acquired.
 * @return false otherwise.
 */
bool LynxMotorDriver::tryGetUpdateAck(uint16_t & can_count)
{
  if (action_mutexes_[Action::Fields::UpdateAck]->try_lock())
  {
    can_count = can_count_;
    return true;
  }

  return false;
}

/**
 * @brief Attempt to acquire the update alive mutex.
 * 
 * @return true if acquired.
 * @return false otherwise.
 */
bool LynxMotorDriver::tryGetUpdateAlive()
{
  return action_mutexes_[Action::Fields::UpdateAlive]->try_lock();
}

/**
 * @brief Acquire the update ack mutex.
 * Blocks until acquired.
 */
void LynxMotorDriver::getUpdateAck(uint16_t & can_count)
{
  action_mutexes_[Action::Fields::UpdateAck]->lock();
  can_count = can_count_;
}

/**
 * @brief Acquire the update alive mutex.
 * Blocks until acquired.
 */
void LynxMotorDriver::getUpdateAlive()
{
  action_mutexes_[Action::Fields::UpdateAlive]->lock();
}

}  // namespace lynx_motor_driver
