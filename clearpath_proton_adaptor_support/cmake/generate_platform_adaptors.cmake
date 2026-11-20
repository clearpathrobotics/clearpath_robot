set(_CLEARPATH_PROTON_ADAPTOR_SUPPORT_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

macro(clearpath_generate_platform_adaptors schema_name)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(clearpath_firmware REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(proton_vendor REQUIRED)
  find_package(proton_ros2_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(clearpath_platform_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nmea_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  get_filename_component(firmware_share "${clearpath_firmware_DIR}/.." ABSOLUTE)
  set(schema_dir "${firmware_share}/proton")
  set(schema_file "${schema_dir}/${schema_name}")
  if(NOT EXISTS "${schema_file}")
    message(FATAL_ERROR "Selected Proton schema not found: ${schema_file}")
  endif()

  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import pathlib, proton_ros2_adaptor_generator; print(pathlib.Path(proton_ros2_adaptor_generator.__file__).parent)"
    OUTPUT_VARIABLE generator_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE generator_path_result
  )
  if(NOT generator_path_result EQUAL 0)
    message(FATAL_ERROR "Unable to locate proton_ros2_adaptor_generator Python module")
  endif()
  file(GLOB generator_files CONFIGURE_DEPENDS
    "${generator_dir}/*.py"
    "${generator_dir}/resources/*.jinja"
  )
  set(driver "${_CLEARPATH_PROTON_ADAPTOR_SUPPORT_DIR}/scripts/generate_adaptors.py")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${schema_file}" ${generator_files} "${driver}"
  )

  set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${driver}"
      --schemas "${schema_dir}" --schema "${schema_name}"
      --output "${output_dir}" --package-name "${PROJECT_NAME}"
    RESULT_VARIABLE generator_result
  )
  if(NOT generator_result EQUAL 0)
    message(FATAL_ERROR "Proton adaptor generation failed (${generator_result})")
  endif()
  include("${output_dir}/adaptor_sources.cmake")
  add_library(${PROJECT_NAME} SHARED ${PROTON_ADAPTOR_SOURCES})
  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
  ament_target_dependencies(${PROJECT_NAME}
    pluginlib proton_ros2_interfaces rclcpp clearpath_platform_msgs
    geometry_msgs nmea_msgs sensor_msgs std_msgs
  )
  install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
  )
  install(FILES "${output_dir}/plugins.xml" DESTINATION share/${PROJECT_NAME})
  ament_index_register_resource("proton_ros2_interfaces__pluginlib__plugin"
    CONTENT "share/${PROJECT_NAME}/plugins.xml\n"
  )
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(
    pluginlib proton_ros2_interfaces rclcpp clearpath_platform_msgs
    geometry_msgs nmea_msgs sensor_msgs std_msgs
  )
endmacro()
