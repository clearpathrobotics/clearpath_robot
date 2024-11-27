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

#ifndef LYNX_MOTOR_DRIVER__MESSAGE_H
#define LYNX_MOTOR_DRIVER__MESSAGE_H

#include "lynx_motor_driver/can_api.hpp"
#include "can_msgs/msg/frame.hpp"

#include <stdint.h>
#include <cstring>
#include <cmath>

namespace lynx_motor_driver
{

/**
 * @brief Wrap a can_msgs Frame to add Lynx API functions
 * 
 */
struct Message
{
  can_msgs::msg::Frame frame_;

  Message(can_msgs::msg::Frame::SharedPtr frame) : frame_(*frame)
  {}

  Message(can_msgs::msg::Frame frame) : frame_(frame)
  {}

  Message()
  {}

  /**
   * @brief Get the Frame object
   * 
   * @return can_msgs::msg::Frame 
   */
  can_msgs::msg::Frame getFrame() const
  {
    return frame_;
  }

  /**
   * @brief Get the Lynx CAN ID from the frame
   * 
   * @return uint32_t representing the CAN ID
   */
  uint32_t getID() const
  {
    return frame_.id & CAN_MSGID_CANID_MASK;
  }

  /**
   * @brief Get the Lynx CAN API from the frame
   * 
   * @return uint32_t representing the MSGID from the API
   */
  uint32_t getApi() const
  {
    return (frame_.id & (CAN_MSGID_FULL_MASK ^ CAN_MSGID_CANID_MASK)) >> CAN_MSGID_API_SHIFT;
  }

  /**
   * @brief Get the length of the frame data
   * 
   * @return uint8_t representing the length
   */
  uint8_t getLength() const
  {
    return frame_.dlc;
  }

  /**
   * @brief Get the first byte of the frame data
   * 
   * @return uint8_t byte
   */
  uint8_t getDataAsUint8() const
  {
    return frame_.data[0];
  }
  
  /**
   * @brief Get the first two bytes of data as a uint16_t object
   * 
   * @return uint16_t 
   */
  uint16_t getDataAsUint16() const
  {
    return *reinterpret_cast<const uint16_t *>(&frame_.data[0]);
  }

  /**
   * @brief Get a float from the frame data starting from start_byte
   * 
   * @param start_byte index of first byte
   * @return float 
   */
  float getDataAsFloat(uint8_t start_byte) const
  {
    float dataf = std::nanf("0");

    if (start_byte <= 4)
    {
      dataf = *reinterpret_cast<const float *>(&frame_.data[start_byte]);
    }

    return dataf;
  }

  /**
   * @brief Get a pair representing the data index and float value
   * 
   * @return std::pair<uint8_t, float> 
   */
  std::pair<uint8_t, float> getDataAsIndexedFloat() const
  {
    uint8_t index = std::numeric_limits<uint8_t>::max();
    float dataf = std::nanf("0");

    if (frame_.dlc == 5)
    {
      index = frame_.data[0];
      dataf = *reinterpret_cast<const float *>(&frame_.data[1]);
    }

    return std::pair<uint8_t, float>(index, dataf);
  }

  /**
   * @brief Get a pair representing the data index and uint32_t value
   * 
   * @return std::pair<uint8_t, uint32_t> 
   */
  std::pair<uint8_t, uint32_t> getDataAsIndexedUint32() const
  {
    uint8_t index = std::numeric_limits<uint8_t>::max();
    uint32_t data = 0;

    if (frame_.dlc == 5)
    {
      index = frame_.data[0];
      data = *reinterpret_cast<const uint32_t *>(&frame_.data[1]);
    }

    return std::pair<uint8_t, uint32_t>(index, data);
  }
};

}  // namespace lynx_motor_driver

#endif  // LYNX_MOTOR_DRIVER__MESSAGE_H
