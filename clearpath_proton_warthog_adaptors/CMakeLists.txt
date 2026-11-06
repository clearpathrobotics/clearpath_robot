cmake_minimum_required(VERSION 3.20)
project(clearpath_proton_warthog_adaptors)

find_package(ament_cmake REQUIRED)
find_package(clearpath_proton_adaptor_support REQUIRED)
clearpath_generate_platform_adaptors(w200.yaml)

ament_package()
