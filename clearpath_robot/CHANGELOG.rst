^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
