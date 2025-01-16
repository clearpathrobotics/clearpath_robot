# Lynx Motor Driver

C++ Driver and ROS 2 node for Clearpath's Lynx BLDC motor controller.

## Clearpath Platforms using Lynx

- Husky A300

## Usage

This driver will be automatically launched when using the [Clearpath Config](https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview) system.

### Manual launch

`ros2 launch lynx_motor_driver lynx_motor_driver.launch.py can_bus:=my_can0 namespace:=/my_namespace parameters:=/path/to/my/parameters.yaml`

### Launch arguments

- `can_bus`: CAN bus interface to use. 
  - Default: `can0`
- `namespace`: Robot namespace. 
  - Default: `/`
- `parameters`: Node parameters. 
  - Default: `lynx_motor_driver/config/single_test.yaml`

## Node

The Lynx motor node will create a driver for each specified motor controller. It will subscribe to velocity commands coming from ROS 2 controls, and publish feedback and status data it receives from the Lynx controllers. It also manages the system protection state.

Additionally, the Lynx motor node can be used to calibrate the motors, and update firmware to the Lynx motor controller.

### Subscribers

- `platform/motors/cmd`: Velocity commands to send to the motor controller. Joint name must match parameters.
  - Type: `sensor_msgs/msg/JointState`

### Publishers
- `platform/motors/system_protection`: System and individual motor controller protection states.
  - Type: `clearpath_motor_msgs/msg/LynxSystemProtection`

- `platform/motors/feedback`: Motor feedback such as current, voltage, and velocity.
  - Type: `clearpath_motor_msgs/msg/LynxMultiFeedback`

- `platform/motors/status`: Motor statuses such as temperature, and flags.
  - Type: `clearpath_motor_msgs/msg/LynxMultiStatus`

### Actions

- `platform/motors/calibrate`: Run the calibration sequence on each motor controller.
  - Type: `clearpath_motor_msgs/action/LynxCalibrate`


  - Usage:
`ros2 action send_goal /platform/motors/calibrate clearpath_motor_msgs/action/LynxCalibrate {} --feedback`

:warning:
The robot must be placed on a box and off the ground before running this action. The wheels will begin to turn when it is called.


- `platform/motors/update`: Update each motor controller with a binary file.
  - Type: `clearpath_motor_msgs/action/LynxUpdate`


  - Usage:
`ros2 action send_goal /platform/motors/update clearpath_motor_msgs/action/LynxUpdate "file: ''" --feedback`

:warning:
Only use Clearpath verified binary files. Attempting to flash custom files can render the Lynx motor controller non-functional. Leave the file field empty to use the default binary.
