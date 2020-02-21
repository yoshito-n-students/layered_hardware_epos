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
    typedef Result< Device > result;
    Device device;
    unsigned int error_code;
    device.handle_.reset(VCS_OpenDevice(const_cast< char * >(device_name.c_str()),
                                        const_cast< char * >(protocol_stack_name.c_str()),
                                        const_cast< char * >(interface_name.c_str()),
                                        const_cast< char * >(port_name.c_str()), &error_code),
                         Device::close);
    return device.handle_ ? result::success(device) : result::error(error_code);
  }

  // ========================
  // protocol stack settings

  Result< unsigned int > setBaudrate(const unsigned int baudrate) {
    typedef Result< unsigned int > result;

    const result result_timeout(getTimeout());
    if (result_timeout.isError()) {
      return result_timeout;
    }

    unsigned int error_code;
    if (VCS_SetProtocolStackSettings(handle_.get(), baudrate, *result_timeout, &error_code) == 0) {
      return result::error(error_code);
    }

    return result::success(baudrate);
  }

  Result< unsigned int > getBaudrate() const {
    typedef Result< unsigned int > result;
    unsigned int baudrate, timeout, error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), &baudrate, &timeout, &error_code) != 0
               ? result::success(baudrate)
               : result::error(error_code);
  }

  Result< unsigned int > setTimeout(const unsigned int timeout_ms) {
    typedef Result< unsigned int > result;

    const result result_baudrate(getBaudrate());
    if (result_baudrate.isError()) {
      return result_baudrate;
    }

    unsigned int error_code;
    if (VCS_SetProtocolStackSettings(handle_.get(), *result_baudrate, timeout_ms, &error_code) ==
        0) {
      return result::error(error_code);
    }

    return result::success(timeout_ms);
  }

  Result< unsigned int > getTimeout() const {
    typedef Result< unsigned int > result;
    unsigned int baudrate, timeout, error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), &baudrate, &timeout, &error_code) != 0
               ? result::success(timeout)
               : result::error(error_code);
  }

  // ============
  // information

  Result< std::string > getDeviceName() const {
    typedef Result< std::string > result;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetDeviceName(handle_.get(), buffer, 256, &error_code) != 0
               ? result::success(buffer)
               : result::error(error_code);
  }

  Result< std::string > getProtocolStackName() const {
    typedef Result< std::string > result;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetProtocolStackName(handle_.get(), buffer, 256, &error_code) != 0
               ? result::success(buffer)
               : result::error(error_code);
  }

  Result< std::string > getInterfaceName() const {
    typedef Result< std::string > result;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetInterfaceName(handle_.get(), buffer, 256, &error_code) != 0
               ? result::success(buffer)
               : result::error(error_code);
  }

  Result< std::string > getPortName() const {
    typedef Result< std::string > result;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetPortName(handle_.get(), buffer, 256, &error_code) != 0
               ? result::success(buffer)
               : result::error(error_code);
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