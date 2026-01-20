^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.6 (2026-01-20)
------------------

2.8.5 (2026-01-19)
------------------

2.8.4 (2025-11-13)
------------------
* [clearpath_sensors] Updated the StereoLabs Zed topic remapping to reflect changes in the driver. (`#282 <https://github.com/clearpathrobotics/clearpath_robot/issues/282>`_)
* Contributors: Tony Baltovski

2.8.3 (2025-11-10)
------------------

2.8.2 (2025-10-28)
------------------

2.8.1 (2025-10-23)
------------------
* Fix/zed2 objdet params (`#277 <https://github.com/clearpathrobotics/clearpath_robot/issues/277>`_)
  * [clearpath_sensors] Updated Zed2 object detection mode type.
  * [clearpath_sensors] Updated Zed2 object detection model name.
* Contributors: Tony Baltovski

2.8.0 (2025-10-23)
------------------
* Remap preview to separate topic (`#276 <https://github.com/clearpathrobotics/clearpath_robot/issues/276>`_)
* Contributors: luis-camero

2.7.2 (2025-09-18)
------------------

2.7.1 (2025-09-16)
------------------
* Jazzy Fix: Stereolabs Zed launch file (`#265 <https://github.com/clearpathrobotics/clearpath_robot/issues/265>`_)
  * Fix: Stereolabs Zed launch file (`#208 <https://github.com/clearpathrobotics/clearpath_robot/issues/208>`_)
  * Update node parameters
  * Switch to composable node
  * Double to single quotes in Zed parameters
  * Add 'NEURAL_LIGHT' to parameter comment
* Contributors: luis-camero

2.7.0 (2025-08-25)
------------------
* Remove exec_depend on fixposition driver (`#263 <https://github.com/clearpathrobotics/clearpath_robot/issues/263>`_)
  Update the version requirement in the commented-out block below
* Update Fixposition launch file, parameters to use 8.x driver (`#253 <https://github.com/clearpathrobotics/clearpath_robot/issues/253>`_)
  * Migration of Fixposition 8.x parameters
  * Update topic remaps for new driver version
* Contributors: Chris Iverach-Brereton

2.6.3 (2025-08-18)
------------------
* Add missing dependency (`#258 <https://github.com/clearpathrobotics/clearpath_robot/issues/258>`_)
* Fix/image remaps (`#257 <https://github.com/clearpathrobotics/clearpath_robot/issues/257>`_)
  * Add remaps for additional image transports
  * Add missing mono remap
* Correct default oakd config file (`#255 <https://github.com/clearpathrobotics/clearpath_robot/issues/255>`_)
* Contributors: Hilary Luo

2.6.2 (2025-07-15)
------------------

2.6.1 (2025-07-04)
------------------

2.6.0 (2025-07-04)
------------------
* [clearpath_sensors] Fixed missing dependency lms1xx. (`#233 <https://github.com/clearpathrobotics/clearpath_robot/issues/233>`_)
* Contributors: Tony Baltovski

2.5.1 (2025-06-17)
------------------

2.5.0 (2025-05-29)
------------------
* Fix: Wiferion Charger Dependency (`#221 <https://github.com/clearpathrobotics/clearpath_robot/issues/221>`_)
  Add wiferion_charger dependency to clearpath_sensors
* Feature: ros2_canopen Inventus driver switch  (`#216 <https://github.com/clearpathrobotics/clearpath_robot/issues/216>`_)
  * Add canopen_inventus launch to generator
  * Add canopen_inventus_bringup dependency
  * Add canopen_inventus_bringup to CI rosdep key ignore
  * Remove inventus_bmu config and launch
  * Remove unused import
* Contributors: luis-camero

2.4.1 (2025-05-20)
------------------

2.4.0 (2025-04-30)
------------------

2.3.3 (2025-04-17)
------------------

2.3.2 (2025-04-16)
------------------

2.3.1 (2025-04-14)
------------------

2.3.0 (2025-04-11)
------------------
* Feature: Wiferion Charger (`#194 <https://github.com/clearpathrobotics/clearpath_robot/issues/194>`_)
* Remap IMU topic to imu/data_raw (per RPSW-2504) (`#188 <https://github.com/clearpathrobotics/clearpath_robot/issues/188>`_)
* Add support for INS sensors + Fixposition XVN (`#176 <https://github.com/clearpathrobotics/clearpath_robot/issues/176>`_)
* Feature/inventus diagnostics (`#170 <https://github.com/clearpathrobotics/clearpath_robot/issues/170>`_)
* Contributors: Chris Iverach-Brereton, Hilary Luo, Luis Camero

2.2.4 (2025-04-07)
------------------

2.2.3 (2025-03-20)
------------------

2.2.2 (2025-03-17)
------------------

2.2.1 (2025-03-12)
------------------

2.2.0 (2025-03-11)
------------------
* Feature Jazzy Ouster (`#155 <https://github.com/clearpathrobotics/clearpath_robot/issues/155>`_)
  * Add Ouster launch files
  * Add ouster dependency
  * Lint
  * Fix config file in launch
* Apply serial, IP address, MX ID to OAK-D cameras (`#159 <https://github.com/clearpathrobotics/clearpath_robot/issues/159>`_)
* Contributors: Chris Iverach-Brereton, luis-camero

2.1.2 (2025-02-28)
------------------

2.1.1 (2025-02-06)
------------------

2.1.0 (2025-01-31)
------------------
* Jazzy Phidgets IMU Filter (`#138 <https://github.com/clearpathrobotics/clearpath_robot/issues/138>`_)
  * IMU Filter
  * Add imu_filter launch file and added madgwick entry to filter
  * Add imu filter to generator
  * Rename imu_filter_node to imu_filter_madgwick
* Feature/diagnostics (`#135 <https://github.com/clearpathrobotics/clearpath_robot/issues/135>`_)
  * Initial port of diagnostics to C++
  * Remap axis camera topics to match API
  * Monitor MCU Status message frequency
  * Added firmware version check
  * Group MCU diagnostics together
  * Improve messaging around firmware versions
  * Disable MCU diagnostics for A200
* Contributors: Hilary Luo, luis-camero

2.0.4 (2025-01-22)
------------------
* Add config and launch for inventus
* Contributors: Luis Camero

2.0.3 (2025-01-17)
------------------

2.0.2 (2025-01-17)
------------------

2.0.1 (2025-01-17)
------------------

2.0.0 (2025-01-17)
------------------
* Remap `navsatfix` to `fix` (`#128 <https://github.com/clearpathrobotics/clearpath_robot/issues/128>`_)
* 1.1.0
* Changes.
* Remove the image-conversion launch files; they're being relocated to clearpath_offboard_sensors (`#118 <https://github.com/clearpathrobotics/clearpath_robot/issues/118>`_)
* Fix the path for the pointcloud calibration path for Jazzy
* Add support for Axis cameras (`#101 <https://github.com/clearpathrobotics/clearpath_robot/issues/101>`_)
  * Add the axis camera launch file
  * Add the default PTZ dome configuration & pass those parameters by default
  * Set the node name so the parameter namespace resolves correctly
  * Add the complete PTZ config for the dome camera
  * Remap image_raw -> image, ~joint_states -> /robot_namespace/joint_states
  * Add missing / to namespaces
  * Remove the ~ from the namespaces; it's not needed
  * Move the joint_states into the platform namespace
  * Remove duplicate `ptz` entry
* Move imu_filter.yaml to clearpath_sensors
* Remappings to for Phidget Spatial
* Ffmpeg manual launch (`#105 <https://github.com/clearpathrobotics/clearpath_robot/issues/105>`_)
  * Added manual launch files for encoding/decoding ffmpeg
* Switch remove RGB from pointcloud
* Added pointcloud support to OakD
* Set Blackfly binning as 2 for optimal operation
* Update frame rate to match default in clearpath_config although this rate is always overridden, must be integer
* Restrict image transports on decoded blackfly topics to relevant ones
* Disable all extra image transports on the encoded Blackfly topic
* Add ffmpeg compression support for Blackfly
* 0.3.2
* Changes.
* Add OAKD (`#92 <https://github.com/clearpathrobotics/clearpath_robot/issues/92>`_)
  * Add OAKD
  * Remove unused parameter
  * Add UDEV for OAKD
* Add phidgets spatial (`#91 <https://github.com/clearpathrobotics/clearpath_robot/issues/91>`_)
  * Add phidget spatial config and launch files
  * Add dependency
  * Double to single quotes
* Contributors: Chris Iverach-Brereton, Hilary Luo, Luis Camero, Tony Baltovski, luis-camero

1.0.1 (2024-11-28)
------------------

1.0.0 (2024-11-26)
------------------
* Fix full velodyne remapping (`#107 <https://github.com/clearpathrobotics/clearpath_robot/issues/107>`_)
* Velodyne tf and tf_static should be remapped to the robot namespace (`#102 <https://github.com/clearpathrobotics/clearpath_robot/issues/102>`_)
  * Remap Velodyne `/tf` and `/tf_static` topics into robot namespace
* Ffmpeg manual launch (`#105 <https://github.com/clearpathrobotics/clearpath_robot/issues/105>`_)
  * Added manual launch files for encoding/decoding ffmpeg
* Add support for Axis cameras (`#101 <https://github.com/clearpathrobotics/clearpath_robot/issues/101>`_)
  * Add the axis camera launch file
  * Add the default PTZ dome configuration & pass those parameters by default
  * Set the node name so the parameter namespace resolves correctly
  * Add the complete PTZ config for the dome camera
  * Remap image_raw -> image, ~joint_states -> /robot_namespace/joint_states
  * Add missing / to namespaces
  * Remove the ~ from the namespaces; it's not needed
  * Move the joint_states into the platform namespace
  * Remove duplicate `ptz` entry
* Ffmpeg manual launch (`#105 <https://github.com/clearpathrobotics/clearpath_robot/issues/105>`_)
  * Added manual launch files for encoding/decoding ffmpeg
* Move imu_filter.yaml to clearpath_sensors
* Add support for Axis cameras (`#101 <https://github.com/clearpathrobotics/clearpath_robot/issues/101>`_)
  * Add the axis camera launch file
  * Add the default PTZ dome configuration & pass those parameters by default
  * Set the node name so the parameter namespace resolves correctly
  * Add the complete PTZ config for the dome camera
  * Remap image_raw -> image, ~joint_states -> /robot_namespace/joint_states
  * Add missing / to namespaces
  * Remove the ~ from the namespaces; it's not needed
  * Move the joint_states into the platform namespace
  * Remove duplicate `ptz` entry
* Remappings to for Phidget Spatial
* Switch remove RGB from pointcloud
* Set Blackfly binning as 2 for optimal operation
* Update frame rate to match default in clearpath_config although this rate is always overridden, must be integer
* Restrict image transports on decoded blackfly topics to relevant ones
* Disable all extra image transports on the encoded Blackfly topic
* Add ffmpeg compression support for Blackfly
* Added pointcloud support to OakD
* Contributors: Chris Iverach-Brereton, Hilary Luo, Luis Camero, Marco Ambrosio

0.3.2 (2024-10-04)
------------------
* Add OAKD camera
* Add phidget spatial config and launch files
* Contributors: Luis Camero, Tony Baltovski

0.3.1 (2024-09-23)
------------------
* Line too long
* Contributors: Luis Camero

0.3.0 (2024-09-19)
------------------
* Disable all tools in default microstrain config
* Update remappings on image_resize republisher
* Add relay to have a camera info topic
* Contributors: Luis Camero, luis-camero

0.2.15 (2024-08-12)
-------------------
* Removed unused import
* Add blackfly to composable container
* Use intraprocess comms
* Contributors: Luis Camero

0.2.14 (2024-08-08)
-------------------
* Adds GQ7 yaml and launch file
* Contributors: robbiefish

0.2.13 (2024-07-30)
-------------------
* Fixed bug in microstrain param
* Fixed remapping to allow for compressed vizualization
* Contributors: Luis Camero

0.2.12 (2024-07-22)
-------------------
* Remove test file
* Updated Microstrain parameters
* Generate remappings
* Updated Realsense parameters
* Contributors: Luis Camero

0.2.11 (2024-05-28)
-------------------
* Added Zed launch and configuration files
* Contributors: Luis Camero

0.2.10 (2024-05-16)
-------------------

0.2.9 (2024-05-16)
------------------
* Fix tf_static for realsense and microstrain
* Contributors: Hilary Luo

0.2.8 (2024-05-14)
------------------
* Map points to API
* Fixed linting errors
* Renamed realsense node to intel_realsense
* Remapped realsense topics
* Contributors: Luis Camero

0.2.7 (2024-04-10)
------------------

0.2.6 (2024-04-08)
------------------

0.2.5 (2024-03-07)
------------------
* Added image_transport_plugins to package.xml
* Added binning parameters
* Recitfy node matches resize
* Added compressed encode and decode launch files
* Fixed theora encoding node
* Add theora encode and decode launch
* Added rectify and resize
* Contributors: Luis Camero

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
* Removed 'platform' from default namespace
* Added image proc as container
* Missing comma
* Correct debayer node and add remapping
* Added debayer node
* Removed errant bracket
* add serial number to yaml
* Initial Blackfly addition
* Contributors: Hilary Luo, Luis Camero, Tony Baltovski

0.1.3 (2023-10-04)
------------------
* Removed 'platform' from default namespace
* Added image proc as container
* Missing comma
* Correct debayer node and add remapping
* Added debayer node
* Removed errant bracket
* add serial number to yaml
* Initial Blackfly addition
* Contributors: Hilary Luo, Luis Camero

0.1.2 (2023-09-27)
------------------
* Renamed convert to transform
* Contributors: Luis Camero

0.1.1 (2023-09-11)
------------------

0.1.0 (2023-08-31)
------------------

0.0.3 (2023-08-15)
------------------
* Renamed UST10 to UST
  Cleaned up generators
* Fixed umx ports
* Move author in all package.xml to pass xml linter.
* Added UM6/7
* Updated default port for generic gps
* Added Garmin 18x, Smart6 and Smart7
* Contributors: Roni Kreinin, Tony Baltovski

0.0.2 (2023-07-25)
------------------
* Sensor namespace
* Microstrain namespacing
  LMS1xx parameters
* Contributors: Roni Kreinin

0.0.1 (2023-07-20)
------------------
* Namespacing support
* Linter fix
* IMU and VLP fix
* Bishop sensors
* Licenses
  sick launch
* Added microstrain
* Fixed namespacing
* Remove old generated files before generating again
  Pass topic namespace to nodes
  Added velodyne
* realsense
* Simplified launch generation
  Added robot launch
* Initial working launch generator
* Contributors: Roni Kreinin
