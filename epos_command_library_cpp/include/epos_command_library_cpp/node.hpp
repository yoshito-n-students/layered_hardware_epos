#ifndef EPOS_COMMAND_LIBRARY_CPP_NODE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_NODE_HPP

#include <string>
#include <vector>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/result.hpp>

#include <boost/cstdint.hpp>

namespace epos_command_library_cpp {

class Node {
public:
  Node(const Device &device, const unsigned short id) : device_(device), id_(id) {}

  virtual ~Node() {}

  // =====
  // info

  Result< unsigned short > getHardwareVersion() const {
    typedef Result< unsigned short > result;
    unsigned short hw_version, sw_version, app_num, app_version;
    unsigned int error_code;
    return VCS_GetVersion(device_.handle_.get(), id_, &hw_version, &sw_version, &app_num,
                          &app_version, &error_code) != 0
               ? result::success(hw_version)
               : result::error(error_code);
  }

  Result< unsigned short > getSoftwareVersion() const {
    typedef Result< unsigned short > result;
    unsigned short hw_version, sw_version, app_num, app_version;
    unsigned int error_code;
    return VCS_GetVersion(device_.handle_.get(), id_, &hw_version, &sw_version, &app_num,
                          &app_version, &error_code) != 0
               ? result::success(sw_version)
               : result::error(error_code);
  }

  Result< boost::uint64_t > getSerialNumber() const {
    typedef Result< boost::uint64_t > result;

    // get device name
    const Result< std::string > result_device_name(device_.getDeviceName());
    if (result_device_name.isError()) {
      return result::error(result_device_name.errorCode());
    }

    // find object id of the serial number according to the device name
    const std::string device_name(*result_device_name);
    unsigned short object_id, object_sub_id;
    if (device_name == "EPOS" || device_name == "EPOS2") {
      object_id = 0x2004;
      object_sub_id = 0x00;
    } else if (device_name == "EPOS4") {
      object_id = 0x2100;
      object_sub_id = 0x01;
    } else {
      // should never reach here
      std::cerr << "Node::getSerialNumber(): Unknown device name '" << device_name << "'"
                << std::endl;
      return result::error(0);
    }

    // get the serial number
    boost::uint64_t serial_number;
    const Result< unsigned int > result_get_obj(
        getObject(object_id, object_sub_id, &serial_number, 8));
    if (result_get_obj.isError()) {
      return result::error(result_get_obj.errorCode());
    }

    return result::success(serial_number);
  }

  // =======
  // object

  Result< unsigned int > setObject(const unsigned short object_id,
                                   const unsigned short object_sub_id, void *const data,
                                   const unsigned int n_bytes_to_write) {
    typedef Result< unsigned int > result;
    unsigned int n_bytes_written, error_code;
    return VCS_SetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                         n_bytes_to_write, &n_bytes_written, &error_code) != 0
               ? result::success(n_bytes_written)
               : result::error(error_code);
  }

  Result< unsigned int > getObject(const unsigned short object_id,
                                   const unsigned short object_sub_id, void *const data,
                                   const unsigned int n_bytes_to_read) const {
    typedef Result< unsigned int > result;
    unsigned int n_bytes_read, error_code;
    return VCS_GetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                         n_bytes_to_read, &n_bytes_read, &error_code) != 0
               ? result::success(n_bytes_read)
               : result::error(error_code);
  }

  // ===============
  // operation mode

  Result< char > setOperationMode(const char operation_mode) {
    typedef Result< char > result;
    unsigned int error_code;
    return VCS_SetOperationMode(device_.handle_.get(), id_, operation_mode, &error_code) != 0
               ? result::success(operation_mode)
               : result::error(error_code);
  }

  Result< char > getOperationMode() const {
    typedef Result< char > result;
    char operation_mode;
    unsigned int error_code;
    return VCS_GetOperationMode(device_.handle_.get(), id_, &operation_mode, &error_code) != 0
               ? result::success(operation_mode)
               : result::error(error_code);
  }

  // ==============
  // state machine

  Result< bool > resetDevice() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_ResetDevice(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< unsigned short > setState(const unsigned short state) {
    typedef Result< unsigned short > result;
    unsigned int error_code;
    return VCS_SetState(device_.handle_.get(), id_, state, &error_code) != 0
               ? result::success(state)
               : result::error(error_code);
  }

  Result< bool > setEnableState() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_SetEnableState(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< bool > setDisableState() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_SetDisableState(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< bool > setQuickStepState() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_SetQuickStopState(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< bool > clearFault() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_ClearFault(device_.handle_.get(), id_, &error_code) != 0 ? result::success(true)
                                                                        : result::error(error_code);
  }

  Result< unsigned short > getState() const {
    typedef Result< unsigned short > result;
    unsigned short state;
    unsigned int error_code;
    return VCS_GetState(device_.handle_.get(), id_, &state, &error_code) != 0
               ? result::success(state)
               : result::error(error_code);
  }

  Result< bool > getEnableState() const {
    typedef Result< bool > result;
    int is_enabled;
    unsigned int error_code;
    return VCS_GetEnableState(device_.handle_.get(), id_, &is_enabled, &error_code) != 0
               ? result::success(is_enabled == 1)
               : result::error(error_code);
  }

  Result< bool > getDisableState() const {
    typedef Result< bool > result;
    int is_disabled;
    unsigned int error_code;
    return VCS_GetDisableState(device_.handle_.get(), id_, &is_disabled, &error_code) != 0
               ? result::success(is_disabled == 1)
               : result::error(error_code);
  }

  Result< bool > getQuickStopState() const {
    typedef Result< bool > result;
    int is_quick_stopped;
    unsigned int error_code;
    return VCS_GetQuickStopState(device_.handle_.get(), id_, &is_quick_stopped, &error_code) != 0
               ? result::success(is_quick_stopped == 1)
               : result::error(error_code);
  }

  Result< bool > getFaultState() const {
    typedef Result< bool > result;
    int is_in_fault;
    unsigned int error_code;
    return VCS_GetFaultState(device_.handle_.get(), id_, &is_in_fault, &error_code) != 0
               ? result::success(is_in_fault == 1)
               : result::error(error_code);
  }

  // ===============
  // error handling

  Result< unsigned char > getNbOfDeviceError() const {
    typedef Result< unsigned char > result;
    unsigned char n_device_error;
    unsigned int error_code;
    return VCS_GetNbOfDeviceError(device_.handle_.get(), id_, &n_device_error, &error_code) != 0
               ? result::success(n_device_error)
               : result::error(error_code);
  }

  Result< unsigned int > getDeviceErrorCode(const unsigned char device_error_id) const {
    typedef Result< unsigned int > result;
    unsigned int device_error_code, error_code;
    return VCS_GetDeviceErrorCode(device_.handle_.get(), id_, device_error_id, &device_error_code,
                                  &error_code) != 0
               ? result::success(device_error_code)
               : result::error(error_code);
  }

  // =============
  // motion info

  Result< int > getPositionIs() const {
    typedef Result< int > result;
    int position_is;
    unsigned int error_code;
    return VCS_GetPositionIs(device_.handle_.get(), id_, &position_is, &error_code) != 0
               ? result::success(position_is)
               : result::error(error_code);
  }

  Result< int > getVelocityIs() const {
    typedef Result< int > result;
    int velocity_is;
    unsigned int error_code;
    return VCS_GetVelocityIs(device_.handle_.get(), id_, &velocity_is, &error_code) != 0
               ? result::success(velocity_is)
               : result::error(error_code);
  }

  Result< short > getCurrentIs() const {
    typedef Result< short > result;
    short current_is;
    unsigned int error_code;
    return VCS_GetCurrentIs(device_.handle_.get(), id_, &current_is, &error_code) != 0
               ? result::success(current_is)
               : result::error(error_code);
  }

  // ======================
  // profile position mode

  Result< bool > activateProfilePositionMode() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_ActivateProfilePositionMode(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< long > moveToPosition(const long target_position, const bool absolute,
                                const bool immediately) {
    typedef Result< long > result;
    unsigned int error_code;
    return VCS_MoveToPosition(device_.handle_.get(), id_, target_position, absolute ? 1 : 0,
                              immediately ? 1 : 0, &error_code)
               ? result::success(target_position)
               : result::error(error_code);
  }

  // ======================
  // profile velocity mode

  Result< bool > activateProfileVelocityMode() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_ActivateProfileVelocityMode(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< long > moveWithVelocity(const long target_velocity) {
    typedef Result< long > result;
    unsigned int error_code;
    return VCS_MoveWithVelocity(device_.handle_.get(), id_, target_velocity, &error_code)
               ? result::success(target_velocity)
               : result::error(error_code);
  }

  // =============
  // current mode

  Result< bool > activateCurrentMode() {
    typedef Result< bool > result;
    unsigned int error_code;
    return VCS_ActivateCurrentMode(device_.handle_.get(), id_, &error_code) != 0
               ? result::success(true)
               : result::error(error_code);
  }

  Result< short > setCurrentMust(const short current_must) {
    typedef Result< short > result;
    unsigned int error_code;
    return VCS_SetCurrentMust(device_.handle_.get(), id_, current_must, &error_code)
               ? result::success(current_must)
               : result::error(error_code);
  }

private:
  Device device_;
  const unsigned short id_;
};
} // namespace epos_command_library_cpp
#endif