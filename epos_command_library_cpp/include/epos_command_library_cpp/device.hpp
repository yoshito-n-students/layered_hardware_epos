#ifndef EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP

#include <string>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/result.hpp>

#include <boost/shared_ptr.hpp>

namespace epos_command_library_cpp {

class Device {
  friend class Node; // to allow access to the device handle

public:
  virtual ~Device() {}

  // ===============
  // factory method

  static Result< Device > open(const std::string &device_name,
                               const std::string &protocol_stack_name,
                               const std::string &interface_name, const std::string &port_name) {
    typedef Result< Device > ResultD;
    Device device;
    unsigned int error_code;
    device.handle_.reset(VCS_OpenDevice(const_cast< char * >(device_name.c_str()),
                                        const_cast< char * >(protocol_stack_name.c_str()),
                                        const_cast< char * >(interface_name.c_str()),
                                        const_cast< char * >(port_name.c_str()), &error_code),
                         Device::close);
    return device.handle_ ? ResultD::success(device) : ResultD::error(error_code);
  }

  // ========================
  // protocol stack settings

  Result< void > setBaudrate(const unsigned int baudrate) {
    typedef Result< void > ResultV;

    const Result< unsigned int > timeout(getTimeout());
    if (timeout.isError()) {
      return ResultV::error(timeout.errorCode());
    }

    unsigned int error_code;
    if (VCS_SetProtocolStackSettings(handle_.get(), baudrate, *timeout, &error_code) == 0) {
      return ResultV::error(error_code);
    }

    return ResultV::success();
  }

  Result< unsigned int > getBaudrate() const {
    typedef Result< unsigned int > ResultUI;
    unsigned int baudrate, timeout, error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), &baudrate, &timeout, &error_code) != 0
               ? ResultUI::success(baudrate)
               : ResultUI::error(error_code);
  }

  Result< void > setTimeout(const unsigned int timeout_ms) {
    typedef Result< void > ResultV;

    const Result< unsigned int > baudrate(getBaudrate());
    if (baudrate.isError()) {
      return ResultV::error(baudrate.errorCode());
    }

    unsigned int error_code;
    if (VCS_SetProtocolStackSettings(handle_.get(), *baudrate, timeout_ms, &error_code) ==
        0) {
      return ResultV::error(error_code);
    }

    return ResultV::success();
  }

  Result< unsigned int > getTimeout() const {
    typedef Result< unsigned int > ResultUI;
    unsigned int baudrate, timeout, error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), &baudrate, &timeout, &error_code) != 0
               ? ResultUI::success(timeout)
               : ResultUI::error(error_code);
  }

  // ============
  // information

  Result< std::string > getDeviceName() const {
    typedef Result< std::string > ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetDeviceName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result< std::string > getProtocolStackName() const {
    typedef Result< std::string > ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetProtocolStackName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result< std::string > getInterfaceName() const {
    typedef Result< std::string > ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetInterfaceName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result< std::string > getPortName() const {
    typedef Result< std::string > ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetPortName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

private:
  Device() {}

  static void close(void *const handle) {
    unsigned int error_code;
    if (VCS_CloseDevice(handle, &error_code) == 0) {
      // deleter of shared_ptr must not throw
      // TODO: print error info
    }
  }

private:
  boost::shared_ptr< void > handle_;
};
} // namespace epos_command_library_cpp

#endif