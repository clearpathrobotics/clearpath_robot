cmake_minimum_required(VERSION 3.8)
project(puma_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(can_hardware REQUIRED)
find_package(clearpath_motor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(DEPENDENCIES
  ament_cmake
  can_hardware
  clearpath_motor_msgs
  diagnostic_updater
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(multi_puma_node
  src/multi_puma_node.cpp
  src/driver.cpp
)
target_include_directories(multi_puma_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(multi_puma_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(multi_puma_node ${DEPENDENCIES})
target_link_libraries(multi_puma_node)

install(TARGETS multi_puma_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
