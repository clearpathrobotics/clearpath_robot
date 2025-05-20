^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.1 (2025-05-20)
------------------

2.4.0 (2025-04-30)
------------------
* Fix A300 fans test (`#204 <https://github.com/clearpathrobotics/clearpath_robot/issues/204>`_)
* Add additional allowed warnings/errors for specific platforms (`#202 <https://github.com/clearpathrobotics/clearpath_robot/issues/202>`_)
* Reduce strafe distance, increase strafe speed (`#203 <https://github.com/clearpathrobotics/clearpath_robot/issues/203>`_)
* Move clearpath_tests into clearpath_robot (`#201 <https://github.com/clearpathrobotics/clearpath_robot/issues/201>`_)
* Contributors: Chris Iverach-Brereton

2.3.3 (2025-04-24)
------------------
* Re-implement rotation test, add linear acceleration test, add motor-cutoff tests for dingo & jackal (`#7 <https://github.com/clearpathrobotics/clearpath_tests/issues/7>`_)
  * Re-implement the rotation test to be an IMU check instead of odometry
  * Re-enable rotation tests for all platforms with an integrated IMU. Add IMU tilt test to Dingo variants
  * Clarified log messages, use consistent accuracy vs error terminology
  * Allow wider margins on the Jackal IMU tests, as it is a component with known lower quality
  * Minor code cleanup & refactoring for consistency
  * Add motor-cutoff tests for Jackal, Dingo
* Fixed topic of fans test. (`#6 <https://github.com/clearpathrobotics/clearpath_tests/issues/6>`_)
* Contributors: Chris Iverach-Brereton, Tony Baltovski

2.3.2
-----

2.3.1 (2025-04-10)
------------------
* Fix a bug where the e-stop test could cause the test node to crash
* Add a 2s delay between clearing the e-stop and moving the wheels to allow CAN reconnections as needed
* Remove CAN device ID count for platforms that don't use CANopen
* Lower the time threshold for linear drive test to 0.75 (from 0.8)
* Change the default serial device for Jackal
* Contributors: Chris Iverach-Brereton

2.3.0 (2025-04-03)
------------------
* Allow symlinks to the device handle
* Fix serial MCU tests
* Ensure the firmware is 2.3 or higher to allow fan control; otherwise just skip the fan test
* Add support for lateral driving test for omni platforms (`#5 <https://github.com/clearpathrobotics/clearpath_tests/issues/5>`_)
* Re-enable the fan tests (`#4 <https://github.com/clearpathrobotics/clearpath_tests/issues/4>`_)
* Remove rotation test (`#3 <https://github.com/clearpathrobotics/clearpath_tests/issues/3>`_)
* Contributors: Chris Iverach-Brereton

0.2.9 (2025-03-18)
------------------
* Rewrite rotation test (`#2 <https://github.com/clearpathrobotics/clearpath_tests/issues/2>`_)
  * Overhaul the logic of the rotation to use an accumulator instead of relying on the latest odom data. Factor in the rate of the EKF + the odometry twist instead of just looking at positional data. On 00005 we're still overshooting slightly, but it's now within the margin of error
* Append the can interface to the node name to suppress duplicate node name warning
* Contributors: Chris Iverach-Brereton

0.2.8 (2025-03-18)
------------------
* Use the last 7 bits for the CAN ID, sort the IDs in the final report
* Note that the IDs are CANopen, and could be incorrect for other devices.
* Contributors: Chris Iverach-Brereton

0.2.7 (2025-03-18)
------------------
* Move the confirmation about the lights being in the normal state before we call start()
* Log the hardware ID and firmware version reported by the MCU status topic
* Move the TF listener implementation to its own file
* Add tests, refactor & reformat to address errors they caught
* Contributors: Chris Iverach-Brereton

0.2.6 (2025-03-18)
------------------
* Add an optional flag for the e-stop, remove key-switch test, add wireless e-stop as an optional component
* Contributors: Chris Iverach-Brereton

0.2.5 (2025-03-18)
------------------
* Simplify linear driving test, reduce mobility test logging (`#1 <https://github.com/clearpathrobotics/clearpath_tests/issues/1>`_)
* Contributors: Chris Iverach-Brereton

0.2.4 (2025-03-17)
------------------
* Reduce the minimum duration for a rotation. Log possible false-positives during the rotation test. Print the calculated duration error for the rotation & drive tests
* Log the version of clearpath_tests in the report
* Increase the length of expected lynx messages to 5, cast the length to an integer before comparing it
* Contributors: Chris Iverach-Brereton

0.2.3 (2025-03-14)
------------------
* Invert the angle of the lateral test
* Add a mutex to prevent issues with reading & writing the current & previous orientations asynchronously; this sometimes causes false positives or false negatives during the test
* Don't fail if we get controller_manager rate errors
* Add newline between average motor currents in report
* Increase the allowed margin of error on the IMU test to 20% (from 10%)
* Add an extra confirmation that the lights are in a controllable state before starting the test
* Contributors: Chris Iverach-Brereton

0.2.2 (2025-03-10)
------------------
* Add missing message dependencies
* Contributors: Chris Iverach-Brereton

0.2.1 (2025-03-07)
------------------
* Fix simple_term_menu_vendor dependency
* Contributors: Chris Iverach-Brereton

0.2.0 (2025-03-07)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton, Tony Baltovski
