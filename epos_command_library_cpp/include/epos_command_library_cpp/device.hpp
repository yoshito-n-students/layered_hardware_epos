#ifndef EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP

#include <iostream>
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

    unsigned int error_code;
    void *const handle(VCS_OpenDevice(const_cast< char * >(device_name.c_str()),
                                      const_cast< char * >(protocol_stack_name.c_str()),
                                      const_cast< char * >(interface_name.c_str()),
                                      const_cast< char * >(port_name.c_str()), &error_code));
    if (!handle) {
      return ResultD::error(error_code);
    }

    Device device;
    device.handle_.reset(handle, Device::close);
    return ResultD::success(device);
  }

  // ========================
  // protocol stack settings

  Result< void > setProtocolStackSettings(const unsigned int baudrate,
                                          const unsigned int timeout_ms) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetProtocolStackSettings(handle_.get(), baudrate, timeout_ms, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setProtocolStackSettings(const unsigned int baudrate, const double timeout_sec) {
    return setProtocolStackSettings(baudrate, static_cast< unsigned int >(timeout_sec * 1000.));
  }

  Result< void > getProtocolStackSettings(unsigned int *const baudrate,
                                          unsigned int *const timeout_ms) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), baudrate, timeout_ms, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > getProtocolStackSettings(unsigned int *const baudrate,
                                          double *const timeout_sec) const {
    typedef Result< void > ResultV;

    unsigned int timeout_ms;
    const ResultV result_get(getProtocolStackSettings(baudrate, &timeout_ms));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *timeout_sec = timeout_ms / 1000.;
    return ResultV::success();
  }

  Result< void > setBaudrate(const unsigned int baudrate) {
    typedef Result< void > ResultV;

    const Result< unsigned int > timeout(getTimeout());
    if (timeout.isError()) {
      return ResultV::error(timeout.errorCode());
    }

    const ResultV result_set(setProtocolStackSettings(baudrate, *timeout));
    if (result_set.isError()) {
      return ResultV::error(result_set.errorCode());
    }

    return ResultV::success();
  }

  Result< unsigned int > getBaudrate() const {
    typedef Result< unsigned int > ResultUI;
    unsigned int baudrate, timeout;
    const Result< void > result_get(getProtocolStackSettings(&baudrate, &timeout));
    return result_get.isSuccess() ? ResultUI::success(baudrate)
                                  : ResultUI::error(result_get.errorCode());
  }

  Result< void > setTimeout(const unsigned int timeout_ms) {
    typedef Result< void > ResultV;

    const Result< unsigned int > baudrate(getBaudrate());
    if (baudrate.isError()) {
      return ResultV::error(baudrate.errorCode());
    }

    const ResultV result_set(setProtocolStackSettings(*baudrate, timeout_ms));
    if (result_set.isError()) {
      return ResultV::error(result_set.errorCode());
    }

    return ResultV::success();
  }

  Result< void > setTimeout(const double timeout_sec) {
    return setTimeout(static_cast< unsigned int >(timeout_sec * 1000.));
  }

  Result< unsigned int > getTimeout() const {
    typedef Result< unsigned int > ResultUI;
    unsigned int baudrate, timeout_ms;
    const Result< void > result_get(getProtocolStackSettings(&baudrate, &timeout_ms));
    return result_get.isSuccess() ? ResultUI::success(timeout_ms)
                                  : ResultUI::error(result_get.errorCode());
  }

  // TODO: implement getTmeoutInSec()

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
      // get error info
      std::string error_info;
      try {
        error_info = Result< void >::error(error_code).errorInfo();
      } catch (const NoErrorInfoException &ex) {
        error_info = ex.what();
      }
      // just print error info because the deleter of a shared_ptr must not throw
      std::cerr << "Device::close(): Failed to close a device handle: " << error_info << std::endl;
      std::cout << handle << std::endl;
    }
  }

private:
  boost::shared_ptr< void > handle_;
};
} // namespace epos_command_library_cpp

#endif