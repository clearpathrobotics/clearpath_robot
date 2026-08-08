/**
Software License Agreement (proprietary)

\file      can_interface.hpp
\authors   Natesh Narain <nnarain@clearpath.ai>
\copyright Copyright (c) 2025, Rockwell Automation Technologies, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Rockwell Automation Technologies.
*/

#ifndef PUMA_MOTOR_DRIVER_CAN_INTERFACE_H
#define PUMA_MOTOR_DRIVER_CAN_INTERFACE_H

#include <can_hardware/drivers/socketcan_driver.hpp>
#include <can_hardware/adaptors/can/connection.hpp>

using CanConnection = can_hardware::adaptor::Connection<can_hardware::drivers::SocketCanDriver>;

#endif // PUMA_MOTOR_DRIVER_CAN_INTERFACE_H
