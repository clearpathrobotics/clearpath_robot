cmake_minimum_required(VERSION 3.20)
project(clearpath_proton_adaptor_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(schema_selection test/test_generate_adaptors.py)
endif()

ament_package(CONFIG_EXTRAS cmake/clearpath_proton_adaptor_support-extras.cmake)
