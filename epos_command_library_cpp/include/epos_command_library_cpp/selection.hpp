#ifndef EPOS_COMMAND_LIBRARY_CPP_SELECTION_HPP
#define EPOS_COMMAND_LIBRARY_CPP_SELECTION_HPP

#include <string>
#include <vector>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/result.hpp>

namespace epos_command_library_cpp {

static inline Result< std::vector< std::string > > getDeviceNameList() {
  typedef Result< std::vector< std::string > > ResultSV;
  char buffer[256];
  int end_of_selection; // BOOL
  unsigned int error_code;
  std::vector< std::string > device_names;

  if (VCS_GetDeviceNameSelection(/* start_of_selection = */ 1, buffer, 256, &end_of_selection,
                                 &error_code) != 0) {
    device_names.push_back(buffer);
    while (end_of_selection == 0) {
      if (VCS_GetDeviceNameSelection(/* start_of_selection = */ 0, buffer, 256, &end_of_selection,
                                     &error_code) != 0) {
        device_names.push_back(buffer);
      }
    }
  }

  return ResultSV::success(device_names);
}

static inline Result< std::vector< std::string > >
getProtocolStackNameList(const std::string &device_name) {
  typedef Result< std::vector< std::string > > ResultSV;
  char buffer[256];
  int end_of_selection; // BOOL
  unsigned int error_code;
  std::vector< std::string > protocol_stack_names;

  if (VCS_GetProtocolStackNameSelection(const_cast< char * >(device_name.c_str()),
                                        /* start_of_selection = */ 1, buffer, 256,
                                        &end_of_selection, &error_code) != 0) {
    protocol_stack_names.push_back(buffer);
    while (end_of_selection == 0) {
      if (VCS_GetProtocolStackNameSelection(const_cast< char * >(device_name.c_str()),
                                            /* start_of_selection = */ 0, buffer, 256,
                                            &end_of_selection, &error_code) != 0) {
        protocol_stack_names.push_back(buffer);
      }
    }
  }

  return ResultSV::success(protocol_stack_names);
}

static inline Result< std::vector< std::string > >
getInterfaceNameList(const std::string &device_name, const std::string &protocol_stack_name) {
  typedef Result< std::vector< std::string > > ResultSV;
  char buffer[256];
  int end_of_selection; // BOOL
  unsigned int error_code;
  std::vector< std::string > interface_names;

  if (VCS_GetInterfaceNameSelection(const_cast< char * >(device_name.c_str()),
                                    const_cast< char * >(protocol_stack_name.c_str()),
                                    /* start_of_selection = */ 1, buffer, 256, &end_of_selection,
                                    &error_code) != 0) {
    interface_names.push_back(buffer);
    while (end_of_selection == 0) {
      if (VCS_GetInterfaceNameSelection(const_cast< char * >(device_name.c_str()),
                                        const_cast< char * >(protocol_stack_name.c_str()),
                                        /* start_of_selection = */ 0, buffer, 256,
                                        &end_of_selection, &error_code) != 0) {
        interface_names.push_back(buffer);
      }
    }
  }

  return ResultSV::success(interface_names);
}

static inline Result< std::vector< std::string > >
getPortNameList(const std::string &device_name, const std::string &protocol_stack_name,
                const std::string &interface_name) {
  typedef Result< std::vector< std::string > > ResultSV;
  char buffer[256];
  int end_of_selection; // BOOL
  unsigned int error_code;
  std::vector< std::string > port_names;

  if (VCS_GetPortNameSelection(const_cast< char * >(device_name.c_str()),
                               const_cast< char * >(protocol_stack_name.c_str()),
                               const_cast< char * >(interface_name.c_str()),
                               /* start_of_selection = */ 1, buffer, 256, &end_of_selection,
                               &error_code) != 0) {
    port_names.push_back(buffer);
    while (end_of_selection == 0) {
      if (VCS_GetPortNameSelection(const_cast< char * >(device_name.c_str()),
                                   const_cast< char * >(protocol_stack_name.c_str()),
                                   const_cast< char * >(interface_name.c_str()),
                                   /* start_of_selection = */ 0, buffer, 256, &end_of_selection,
                                   &error_code) != 0) {
        port_names.push_back(buffer);
      }
    }
  }

  return ResultSV::success(port_names);
}

static inline Result< std::vector< unsigned int > >
getBaudrateList(const std::string &device_name, const std::string &protocol_stack_name,
                const std::string &interface_name, const std::string &port_name) {
  typedef Result< std::vector< unsigned int > > ResultSV;
  unsigned int baudrate;
  int end_of_selection; // BOOL
  unsigned int error_code;
  std::vector< unsigned int > baudrates;

  if (VCS_GetBaudrateSelection(
          const_cast< char * >(device_name.c_str()),
          const_cast< char * >(protocol_stack_name.c_str()),
          const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()),
          /* start_of_selection = */ 1, &baudrate, &end_of_selection, &error_code) != 0) {
    baudrates.push_back(baudrate);
    while (end_of_selection == 0) {
      if (VCS_GetBaudrateSelection(
              const_cast< char * >(device_name.c_str()),
              const_cast< char * >(protocol_stack_name.c_str()),
              const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()),
              /* start_of_selection = */ 0, &baudrate, &end_of_selection, &error_code) != 0) {
        baudrates.push_back(baudrate);
      }
    }
  }

  return ResultSV::success(baudrates);
}

} // namespace epos_command_library_cpp
#endif