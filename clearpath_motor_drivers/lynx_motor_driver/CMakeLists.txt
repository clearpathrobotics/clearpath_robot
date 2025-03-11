################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(lynx_motor_driver)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif ()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(clearpath_motor_msgs REQUIRED)
find_package(clearpath_ros2_socketcan_interface REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Build
################################################################################

include_directories(
  include
)

ament_export_include_directories(include)

set(DEPENDENCIES
  ament_cmake
  ament_index_cpp
  can_msgs
  clearpath_motor_msgs
  clearpath_ros2_socketcan_interface
  diagnostic_updater
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
)

add_executable(lynx_motor_driver src/main.cpp
  src/calibrate_action.cpp
  src/lynx_motor_driver.cpp
  src/lynx_motor_node.cpp
  src/system_protection.cpp
  src/update_action.cpp
)

ament_target_dependencies(lynx_motor_driver ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(TARGETS lynx_motor_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bin config launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
