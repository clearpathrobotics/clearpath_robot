^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_generator_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
