cmake_minimum_required(VERSION 3.8)
project(epos_command_library)

# find dependencies
find_package(ament_cmake REQUIRED)

# locate shared library files
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i386"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i486"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i586"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i686")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv6")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libEposCmd.so.6.8.1.0)
else()
  message(FATAL_ERROR "Unsupported processor type: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# import libraries
add_library(ftd2xx SHARED IMPORTED)
set_target_properties(
  ftd2xx PROPERTIES
  IMPORTED_LOCATION ${ftd2xx_LIBRARY})
add_library(EposCmd SHARED IMPORTED)
set_target_properties(
  EposCmd PROPERTIES
  IMPORTED_LOCATION ${EposCmd_LIBRARY})

# install headers & libraries
install(
  DIRECTORY include/
  DESTINATION include)
install(
  IMPORTED_RUNTIME_ARTIFACTS ftd2xx EposCmd
  DESTINATION lib)

# Export headers & libraries
ament_export_include_directories(include)
ament_export_libraries(ftd2xx EposCmd)

# TODO: declare tests here

ament_package()