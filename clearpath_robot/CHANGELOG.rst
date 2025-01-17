^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-01-15)
------------------
* Add dependency for ewellix_driver (`#125 <https://github.com/clearpathrobotics/clearpath_robot/issues/125>`_)
  * Add dependency for ewellix_driver
  * Alphabetical dependencies
* Contributors: luis-camero

2.0.0 (2025-01-17)
------------------
* [clearpath_robot] Fixed comment.
* [clearpath_robot] Added check for binary install path and fallback to check from a workspace in generate. (`#127 <https://github.com/clearpathrobotics/clearpath_robot/issues/127>`_)
* 1.1.0
* Changes.
* Add dependency for ewellix_driver (`#125 <https://github.com/clearpathrobotics/clearpath_robot/issues/125>`_)
  * Add dependency for ewellix_driver
  * Alphabetical dependencies
* Add zenoh service files & generators (`#116 <https://github.com/clearpathrobotics/clearpath_robot/issues/116>`_)
  * Add zenoh service files & generators
* Remove udev rules for joy controllers (`#113 <https://github.com/clearpathrobotics/clearpath_robot/issues/113>`_)
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
* Make robot service always restart vcan
* Add ur_robot_driver dependency
* Add vcan to robot service wants
* Change vcan service to use generated script
* 0.3.2
* Changes.
* [clearpath_robot] Added script to grab diagnostic logs for troublesho… (`#84 <https://github.com/clearpathrobotics/clearpath_robot/issues/84>`_)
  * [clearpath_robot] Added script to grab diagnostic logs for troubleshooting.
  * Make grab-diagnostics script executable and installed
  ---------
  Co-authored-by: Luis Camero <lcamero@clearpathrobotics.com>
* Remove missing jazzy dependencies (for now)
* Socket CAN Bridges (`#93 <https://github.com/clearpathrobotics/clearpath_robot/issues/93>`_)
  * Generate can bridges
  * Generate script source robot workspace
  * Remove extra line
  ---------
  Co-authored-by: Roni Kreinin <rkreinin@clearpathrobotics.com>
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

1.0.1 (2024-11-28)
------------------
* Added missing dependencies (`#108 <https://github.com/clearpathrobotics/clearpath_robot/issues/108>`_)
* Contributors: Roni Kreinin

1.0.0 (2024-11-26)
------------------
* Added minimum version.
* Make robot service always restart vcan
* Add ur_robot_driver dependency
* Add vcan to robot service wants
* Change vcan service to use generated script
* Contributors: Luis Camero, Tony Baltovski

0.3.2 (2024-10-04)
------------------
* [clearpath_robot] Added script to grab diagnostic logs for troubleshooting.
* Contributors: Luis Camero, Tony Baltovski

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Enable vcan service when installed
* Add dependency socat
* Headers to bash scripts
* Add R100 to Puma enabled
* Use root as user
* Add vcan service
* Added vcan script
* Added SRDF generation to robot service
* Removed incorrect dependency
* Added manipulators dependencies and service
* Contributors: Luis Camero, luis-camero

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

0.2.10 (2024-05-16)
-------------------
* Block microstrain in J100 MCU udev rule
* Contributors: Hilary Luo

0.2.9 (2024-05-16)
------------------

0.2.8 (2024-05-14)
------------------
* Ensure that the network interfaces are active before clearpath_robot service starts - required for FastDDS
* Contributors: Hilary Luo

0.2.7 (2024-04-10)
------------------

0.2.6 (2024-04-08)
------------------
* Removed the argument to source
* Added platform and sensor service to robot service wants
* Added discovery server service
* Contributors: Hilary Luo, Luis Camero

0.2.5 (2024-03-07)
------------------

0.2.4 (2024-01-19)
------------------
* [clearpath_robot] Added udev rule for STM32 ROM bootloader.
* Contributors: Tony Baltovski

0.2.3 (2024-01-18)
------------------

0.2.2 (2024-01-10)
------------------

0.2.1 (2023-12-18)
------------------

0.2.0 (2023-12-13)
------------------
* Run platform and sensor services as user
* [clearpath_robot] Added udev rule to automatically bring-up CANBUS PCIe card for W200.
* [clearpath_robot] Added can-utils as dep.
* Contributors: Roni Kreinin, Tony Baltovski

0.1.3 (2023-10-04)
------------------
* Run platform and sensor services as user
* Contributors: Roni Kreinin

0.1.2 (2023-09-27)
------------------

0.1.1 (2023-09-11)
------------------

0.1.0 (2023-08-31)
------------------
* Create dummy launch files if they do not exist
* Fixed sensors launch file name
* Contributors: Luis Camero, Roni Kreinin

0.0.3 (2023-08-15)
------------------
* Linter
* Move author in all package.xml to pass xml linter.
* Contributors: Roni Kreinin, Tony Baltovski

0.0.2 (2023-07-25)
------------------
* Config update
* Contributors: Roni Kreinin

0.0.1 (2023-07-20)
------------------
* [clearpath_platform] Added J100 MCU, FTDI and Logitech joy udev rules.
* Moved clearpath_platform to clearpath_common
  Added clearpath_generator_robot
  Created clearpath_robot metapackage
  Moved scripts and services into clearpath_robot
* Contributors: Roni Kreinin, Tony Baltovski
