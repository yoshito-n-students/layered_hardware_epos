cmake_minimum_required(VERSION 3.8)
project(layered_hardware_epos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(epos_command_library_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(layered_hardware REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

## Declare a C++ library
add_library(
  layered_hardware_epos_plugins SHARED
  src/layered_hardware_epos_plugins.cpp)
target_include_directories(
  layered_hardware_epos_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  layered_hardware_epos_plugins
  controller_interface
  epos_command_library_cpp
  hardware_interface
  layered_hardware
  pluginlib
  rclcpp
  yaml_cpp_vendor)

## Declare a C++ executable

pluginlib_export_plugin_description_file(layered_hardware plugins.xml)

install(
  TARGETS layered_hardware_epos_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# install(
#   DIRECTORY config launch urdf
#   DESTINATION share/layered_hardware_epos)

# TODO: declare tests here

ament_package()