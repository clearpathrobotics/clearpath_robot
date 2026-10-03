# clearpath_robot

ROS 2 packages for interfacing with Clearpath Platforms (real hardware).

For supported platforms, sensors and manipulators plus additional details, please see: https://docs.clearpathrobotics.com/docs/ros/

## Generator Tests

Changes to the generators in this repository (`clearpath_generator_robot`) may affect the
generated output for launch files and parameter files. The
[clearpath_generator_tests](https://github.com/clearpathrobotics/clearpath_generator_tests)
repository versions the expected output and validates it through CI.

Before merging, ensure a corresponding branch with the **same name** exists in
`clearpath_generator_tests` with regenerated samples. See the
[Development Workflow](https://github.com/clearpathrobotics/clearpath_generator_tests#development-workflow)
section of `clearpath_generator_tests` for the full process.