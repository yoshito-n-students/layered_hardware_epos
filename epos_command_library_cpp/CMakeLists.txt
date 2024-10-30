cmake_minimum_required(VERSION 3.8)
project(epos_command_library_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(epos_command_library REQUIRED)

## Find system dependencies
find_package(
  Boost REQUIRED
  COMPONENTS program_options)

## Declare C++ executables
add_executable(
  list_nodes
  src/list_nodes.cpp)
target_include_directories(
  list_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  list_nodes
  ${Boost_LIBRARIES})
ament_target_dependencies(
  list_nodes
  epos_command_library)

## install resources
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS list_nodes
  RUNTIME DESTINATION lib/epos_command_library_cpp)

# TODO: declare tests here

ament_export_dependencies(
  epos_command_library)
ament_export_include_directories(
  include)

ament_package()