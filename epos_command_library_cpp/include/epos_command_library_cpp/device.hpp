#ifndef EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_DEVICE_HPP

#include <iostream>
#include <memory>
#include <string>
#include <utility> // for std::move()

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/result.hpp>

namespace epos_command_library_cpp {

class Device {
  friend class Node; // to allow access to the device handle

public:
  // ===============
  // factory method

  static Result<Device> open(const std::string &device_name, const std::string &protocol_stack_name,
                             const std::string &interface_name, const std::string &port_name) {
    using ResultD = Result<Device>;

    unsigned int error_code;
    void *const handle = VCS_OpenDevice(const_cast<char *>(device_name.c_str()),
                                        const_cast<char *>(protocol_stack_name.c_str()),
                                        const_cast<char *>(interface_name.c_str()),
                                        const_cast<char *>(port_name.c_str()), &error_code);
    if (!handle) {
      return ResultD::error(error_code);
    }

    Device device;
    device.handle_.reset(handle, Device::close);
    return ResultD::success(std::move(device));
  }

  // ========================
  // protocol stack settings

  Result<void> set_protocol_stack_settings(const unsigned int baudrate,
                                           const unsigned int timeout_ms) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return VCS_SetProtocolStackSettings(handle_.get(), baudrate, timeout_ms, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_protocol_stack_settings_si(const unsigned int baudrate,
                                              const double timeout_sec) {
    return set_protocol_stack_settings(baudrate, static_cast<unsigned int>(timeout_sec * 1000.));
  }

  Result<void> get_protocol_stack_settings(unsigned int *const baudrate,
                                           unsigned int *const timeout_ms) const {
    using ResultV = Result<void>;
    unsigned int error_code;
    return VCS_GetProtocolStackSettings(handle_.get(), baudrate, timeout_ms, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> get_protocol_stack_settings_si(unsigned int *const baudrate,
                                              double *const timeout_sec) const {
    using ResultV = Result<void>;

    unsigned int timeout_ms;
    const auto result_get = get_protocol_stack_settings(baudrate, &timeout_ms);
    if (result_get.is_error()) {
      return ResultV::error(result_get.error_code());
    }

    *timeout_sec = timeout_ms / 1000.;
    return ResultV::success();
  }

  Result<void> set_baudrate(const unsigned int baudrate) {
    using ResultV = Result<void>;

    const auto timeout = get_timeout();
    if (timeout.is_error()) {
      return ResultV::error(timeout.error_code());
    }

    const auto result_set = set_protocol_stack_settings(baudrate, *timeout);
    if (result_set.is_error()) {
      return ResultV::error(result_set.error_code());
    }

    return ResultV::success();
  }

  Result<unsigned int> get_baudrate() const {
    using ResultUI = Result<unsigned int>;
    unsigned int baudrate, timeout;
    const auto result_get = get_protocol_stack_settings(&baudrate, &timeout);
    return result_get.is_success() ? ResultUI::success(baudrate)
                                   : ResultUI::error(result_get.error_code());
  }

  Result<void> set_timeout(const unsigned int timeout_ms) {
    using ResultV = Result<void>;

    const auto baudrate = get_baudrate();
    if (baudrate.is_error()) {
      return ResultV::error(baudrate.error_code());
    }

    const auto result_set = set_protocol_stack_settings(*baudrate, timeout_ms);
    if (result_set.is_error()) {
      return ResultV::error(result_set.error_code());
    }

    return ResultV::success();
  }

  Result<void> set_timeout_si(const double timeout_sec) {
    return set_timeout(static_cast<unsigned int>(timeout_sec * 1000.));
  }

  Result<unsigned int> get_timeout() const {
    using ResultUI = Result<unsigned int>;
    unsigned int baudrate, timeout_ms;
    const auto result_get = get_protocol_stack_settings(&baudrate, &timeout_ms);
    return result_get.is_success() ? ResultUI::success(timeout_ms)
                                   : ResultUI::error(result_get.error_code());
  }

  Result<double> get_timeout_si() const {
    using ResultD = Result<double>;
    const auto timeout_ms = get_timeout();
    return timeout_ms.is_success() ? ResultD::success(*timeout_ms / 1000.)
                                   : ResultD::error(timeout_ms.error_code());
  }

  // ============
  // information

  Result<std::string> get_device_name() const {
    typedef Result<std::string> ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetDeviceName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result<std::string> get_protocol_stack_name() const {
    typedef Result<std::string> ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetProtocolStackName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result<std::string> get_interface_name() const {
    typedef Result<std::string> ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetInterfaceName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

  Result<std::string> get_port_name() const {
    typedef Result<std::string> ResultS;
    char buffer[256];
    unsigned int error_code;
    return VCS_GetPortName(handle_.get(), buffer, 256, &error_code) != 0
               ? ResultS::success(buffer)
               : ResultS::error(error_code);
  }

private:
  Device() = default;

  static void close(void *const handle) {
    unsigned int error_code;
    if (VCS_CloseDevice(handle, &error_code) == 0) {
      // get error info
      std::string error_info;
      try {
        error_info = Result<void>::error(error_code).error_info();
      } catch (const NoErrorInfoException &ex) {
        error_info = ex.what();
      }
      // just print error info because the deleter of a shared_ptr must not throw
      std::cerr << "Device::close(): Failed to close a device handle (" << handle
                << "): " << error_info << std::endl;
    }
  }

private:
  std::shared_ptr<void> handle_;
};
} // namespace epos_command_library_cpp

#endif