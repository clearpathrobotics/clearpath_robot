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

#include "lynx_motor_driver/lynx_motor_node.hpp"

using namespace lynx_motor_driver;


/**
 * @brief Update system level proctection state
 * 
 */
void LynxMotorNode::updateSystemState()
{
  uint8_t system_state = clearpath_motor_msgs::msg::LynxSystemProtection::NORMAL;

  // Set system level protection state to max of all driver protection states
  for (auto & driver : drivers_)
  {
    if (driver.getProtectionState() > system_state)
    {
      system_state = driver.getProtectionState();
    }
  }

  // Update system protection state
  if (system_state != system_protection_msg_.system_state)
  {
    system_protection_msg_.system_state = system_state;
    sendSystemState();
  }
}

/**
 * @brief Send system protection state to each driver and publish to topic
 * 
 */
void LynxMotorNode::sendSystemState()
{
  int i = 0;
  for (auto & driver : drivers_)
  {
    system_protection_msg_.motor_states.at(i++) = driver.getProtectionState();
    if (!updating_)
    {
      driver.sendProtectionState(system_protection_msg_.system_state);
    }
  }

  publishSystemState();
}

/**
 * @brief Stamp and publish system protection state
 * 
 */
void LynxMotorNode::publishSystemState()
{
  system_protection_msg_.header.stamp = this->get_clock()->now();
  system_protection_pub_->publish(system_protection_msg_);
}
