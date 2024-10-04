^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
