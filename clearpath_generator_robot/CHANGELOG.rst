^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.2 (2024-10-04)
------------------

0.3.1 (2024-09-23)
------------------

0.3.0 (2024-09-19)
------------------
* Only add manipulator.launch if manipulator added
* Add Ridgeback to generator
* Added dependency to puma_motor_driver
* Added puma node to generated platform launch
* Added manipulators to launch generator
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

0.2.9 (2024-05-16)
------------------
* Fix tf_static for realsense and microstrain
* Contributors: Hilary Luo

0.2.8 (2024-05-14)
------------------
* Lint error in test
* Workspace install paths
* Ignore error from deleting clearpath temp folder
* More linting changes
* Fixed linting errors
* Added pytest to check config
* Fixed linter errors
* Contributors: Luis Camero

0.2.7 (2024-04-10)
------------------
* Check the correct launch file
* Contributors: Luis Camero

0.2.6 (2024-04-08)
------------------
* Add extra launch file to sensor service
* Contributors: Luis Camero

0.2.5 (2024-03-07)
------------------
* Add valence dependency
* Launch Valence BMS when relevant
* Rename node based on input
* Added rectify and resize
* Float hz parameter
* Contributors: Hilary Luo, Luis Camero, Roni Kreinin

0.2.4 (2024-01-19)
------------------

0.2.3 (2024-01-18)
------------------
* Removed namespaced tf_static
* Contributors: Luis Camero

0.2.2 (2024-01-10)
------------------
* [clearpath_generator_robot] Re-added sevcon_traction as dependency.
* Contributors: Tony Baltovski

0.2.1 (2023-12-18)
------------------
* Added missing dependency
* Contributors: Hilary Luo

0.2.0 (2023-12-13)
------------------
* [clearpath_generator_robot] Disabled depend for now.
* Added D100 and D150 to generator and battery node
* IMU 0 filter for W200
* sevcon_traction dependency
* Generate sevcon traction node
* Generate lighting node
* Launch battery state control
* Renamed to battery_state_estimator
  Added to robot generator
* Get namespace from robot.yaml for diagnostics launch
  Added diagnostics launch to generator
* W200 uROS node
* Contributors: Roni Kreinin, Tony Baltovski

0.1.3 (2023-10-04)
------------------

0.1.2 (2023-09-27)
------------------

0.1.1 (2023-09-11)
------------------
* [clearpath_generator_robot] Re-added micro-ros-agent as exec depend.
* Contributors: Tony Baltovski

0.1.0 (2023-08-31)
------------------

0.0.3 (2023-08-15)
------------------
* Removed micros-ros-agent as dep.
* Renamed UST10 to UST
  Cleaned up generators
* Move author in all package.xml to pass xml linter.
* [clearpath_generator_robot] Added author to package.xml.
* Added UM6/7
* Added Garmin 18x, Smart6 and Smart7
* Contributors: Roni Kreinin, Tony Baltovski

0.0.2 (2023-07-25)
------------------
* Sensor namespace
* Param generator
* Launch generator cleanup
* NMEA navsat driver
* Import paths
* Contributors: Roni Kreinin

0.0.1 (2023-07-20)
------------------
* Set use_sim_time to false
* Updated namespace and domain id service call
* Updates for how launch files are written
* Namespacing support
* Moved clearpath_platform to clearpath_common
  Added clearpath_generator_robot
  Created clearpath_robot metapackage
  Moved scripts and services into clearpath_robot
* Contributors: Roni Kreinin
