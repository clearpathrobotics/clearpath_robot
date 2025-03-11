^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lynx_motor_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2025-03-11)
------------------
* Lynx Warning flags (`#151 <https://github.com/clearpathrobotics/clearpath_robot/issues/151>`_)
  * Added warning flags
  * Updated diagnostics for warning flags
  * Updated binary
* Added Lynx motor driver diagnostics (`#149 <https://github.com/clearpathrobotics/clearpath_robot/issues/149>`_)
  * Removed trailing spaces
  * Added Lynx motor diagnostics
* Contributors: Hilary Luo, Roni Kreinin

2.1.2 (2025-02-28)
------------------

2.1.1 (2025-02-06)
------------------

2.1.0 (2025-01-31)
------------------
* Lynx updates (`#139 <https://github.com/clearpathrobotics/clearpath_robot/issues/139>`_)
  * Wildcard for binary file name
  * Don't queue CAN messages
* Contributors: Roni Kreinin

2.0.4 (2025-01-22)
------------------
* Multiply feedback velocity by wheel direction (`#132 <https://github.com/clearpathrobotics/clearpath_robot/issues/132>`_)
* Contributors: Roni Kreinin

2.0.3 (2025-01-17)
------------------

2.0.2 (2025-01-17)
------------------
* [clearpath_motor_drivers/lynx_motor_driver] Added rclcpp_action as dep.
* Contributors: Tony Baltovski

2.0.1 (2025-01-17)
------------------
* [clearpath_motor_drivers/lynx_motor_driver] Fixed required version of clearpath_motor_msgs.
* Contributors: Tony Baltovski

2.0.0 (2025-01-17)
------------------
* CAN bootloader fixes (`#112 <https://github.com/clearpathrobotics/clearpath_robot/issues/112>`_)
  * Send boot request to all drivers on action start
  * Removed first ack check
  * Added Ack counter
  * Resend data if counters dont match
  * Make app counter a private variable of driver
  * Re-attempt entering bootloader and alive check 5 times before skipping
  * Fix for skipping unresponsive Lynx's
  * Removed whitespace
* A300 (`#106 <https://github.com/clearpathrobotics/clearpath_robot/issues/106>`_)
  * Added lynx hardware interface
  * Lynx motor driver
  Rename clearpath_platform namespace to clearpath_hardware_interfaces
  * Added A300 and Inventus battery to generator
  * A300 lighting
  * Dependencies and README
  * Rename platform to hardware_interfaces in hardware.xml
  * Fix append of bms in generator
  * Removed wheel_joints\_ map
  ---------
  Co-authored-by: Luis Camero <lcamero@clearpathrobotics.com>
* Contributors: Roni Kreinin

* CAN bootloader fixes (`#112 <https://github.com/clearpathrobotics/clearpath_robot/issues/112>`_)
  * Send boot request to all drivers on action start
  * Removed first ack check
  * Added Ack counter
  * Resend data if counters dont match
  * Make app counter a private variable of driver
  * Re-attempt entering bootloader and alive check 5 times before skipping
  * Fix for skipping unresponsive Lynx's
  * Removed whitespace
* A300 (`#106 <https://github.com/clearpathrobotics/clearpath_robot/issues/106>`_)
  * Added lynx hardware interface
  * Lynx motor driver
  Rename clearpath_platform namespace to clearpath_hardware_interfaces
  * Added A300 and Inventus battery to generator
  * A300 lighting
  * Dependencies and README
  * Rename platform to hardware_interfaces in hardware.xml
  * Fix append of bms in generator
  * Removed wheel_joints\_ map
  ---------
  Co-authored-by: Luis Camero <lcamero@clearpathrobotics.com>
* Contributors: Roni Kreinin

1.0.1 (2024-11-28)
------------------

1.0.0 (2024-11-26)
------------------

0.3.2 (2024-10-04)
------------------

0.2.15 (2024-08-12)
-------------------

0.2.14 (2024-08-08)
-------------------

0.2.13 (2024-07-30)
-------------------

0.2.12 (2024-07-22)
-------------------

0.2.11 (2024-05-28)
-------------------

0.2.10 (2024-05-16 17:09)
-------------------------

0.2.9 (2024-05-16 12:19)
------------------------

0.2.8 (2024-05-14)
------------------

0.2.7 (2024-04-10)
------------------

0.2.6 (2024-04-08)
------------------

0.2.5 (2024-03-07)
------------------

0.2.4 (2024-01-19)
------------------

0.2.3 (2024-01-18)
------------------

0.2.2 (2024-01-10)
------------------

0.2.1 (2023-12-18)
------------------

0.2.0 (2023-12-13)
------------------

0.1.3 (2023-10-04)
------------------

0.1.2 (2023-09-27)
------------------

0.1.1 (2023-09-11)
------------------

0.1.0 (2023-08-31)
------------------

0.0.3 (2023-08-15)
------------------

0.0.2 (2023-07-25)
------------------

0.0.1 (2023-07-20)
------------------
