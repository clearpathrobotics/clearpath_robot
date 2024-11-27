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

#ifndef LYNX_MOTOR_DRIVER__CAN_API_H
#define LYNX_MOTOR_DRIVER__CAN_API_H


// Masks and bit shifts
#define CAN_MSGID_FULL_MASK        0x1FFFFFFF
#define CAN_MSGID_CANID_MASK       0x0000007F
#define CAN_MSGID_API_MASK         (CAN_MSGID_FULL_MASK ^ CAN_MSGID_CANID_MASK)
#define CAN_MSGID_CANID_SHIFT       0
#define CAN_MSGID_API_SHIFT         7

// Lynx CAN message IDs
#define CAN_MSGID_VELOCITY_DEMAND             0x606B
#define CAN_MSGID_FEEDBACK                    0x7000
#define CAN_MSGID_STATUS                      0x7001
#define CAN_MSGID_DEBUG                       0x7002
#define CAN_MSGID_FLUX_GAINS                  0x9000
#define CAN_MSGID_TORQUE_GAINS                0x9001
#define CAN_MSGID_SPEED_GAINS                 0x9002
#define CAN_MSGID_FLUX_LIMITS                 0xA000
#define CAN_MSGID_TORQUE_LIMITS               0xA001
#define CAN_MSGID_SPEED_LIMITS                0xA002
#define CAN_MSGID_CALIBRATION_REQUEST         0xB000
#define CAN_MSGID_CALIBRATION_ITERATION       0xB001
#define CAN_MSGID_CALIBRATION_OFFSET          0xB002
#define CAN_MSGID_CALIBRATION_CANCEL          0xB003
#define CAN_MSGID_PROTECTION_SYSTEM_STATE     0xC000
#define CAN_MSGID_PROTECTION_MOTOR_STATE      0xC001
#define CAN_MSGID_PROTECTION_ACCUMULATORS     0xC002
#define CAN_MSGID_BOOT_ALIVE                  0xF000
#define CAN_MSGID_BOOT_DATA                   0xF001
#define CAN_MSGID_BOOT_RESP                   0xF002
#define CAN_MSGID_BOOT_REQ                    0xF003

#endif // LYNX_MOTOR_DRIVER__CAN_API_H
