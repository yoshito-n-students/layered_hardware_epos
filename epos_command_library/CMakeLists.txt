cmake_minimum_required(VERSION 3.8)
project(epos_command_library)

# find dependencies
find_package(ament_cmake REQUIRED)

# locate library's directory based on prosessor type
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(IMPORTED_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib/intel/x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i386"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i486"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i586"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i686")
  set(IMPORTED_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib/intel/x86)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv6")
  set(IMPORTED_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib/arm/v6)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(IMPORTED_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib/arm/v7)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(IMPORTED_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib/arm/v8)
else()
  message(FATAL_ERROR "Unsupported processor type: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# import libraries
add_library(EposCmd SHARED IMPORTED)
set_target_properties(
  EposCmd PROPERTIES
  IMPORTED_LOCATION ${IMPORTED_LIBRARY_DIR}/libEposCmd.so.6.8.1.0)
add_library(ftd2xx SHARED IMPORTED)
set_target_properties(
  ftd2xx PROPERTIES
  IMPORTED_LOCATION ${IMPORTED_LIBRARY_DIR}/libftd2xx.so.1.4.8)

# install headers & libraries,
# then generates unversioned symlinks (i.e. libEposCmd.so & libftd2xx.so)
install(
  DIRECTORY include/
  DESTINATION include)
install(
  IMPORTED_RUNTIME_ARTIFACTS EposCmd ftd2xx
  DESTINATION lib)
install(
  CODE "execute_process(COMMAND ldconfig -n \"${CMAKE_INSTALL_PREFIX}/lib\")")

# export headers & libraries to packages which depend on this package
ament_export_include_directories(include)
ament_export_libraries(ftd2xx EposCmd)

# TODO: declare tests here

ament_package()