cmake_minimum_required(VERSION 3.20)
project(clearpath_proton_adaptors)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(ament_index_cpp REQUIRED)
	find_package(pluginlib REQUIRED)
	find_package(proton_ros2_interfaces REQUIRED)
	find_package(yaml_cpp_vendor REQUIRED)
	find_package(yaml-cpp REQUIRED)
	ament_add_gtest(test_plugin_ownership test/test_plugin_ownership.cpp)
	ament_target_dependencies(test_plugin_ownership
		ament_index_cpp pluginlib proton_ros2_interfaces
	)
	target_link_libraries(test_plugin_ownership yaml-cpp)
endif()

ament_package()
