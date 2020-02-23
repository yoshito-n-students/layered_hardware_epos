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
    typedef Result< unsigned short > ResultUS;
    unsigned short hw_version, sw_version, app_num, app_version;
    unsigned int error_code;
    return VCS_GetVersion(device_.handle_.get(), id_, &hw_version, &sw_version, &app_num,
                          &app_version, &error_code) != 0
               ? ResultUS::success(hw_version)
               : ResultUS::error(error_code);
  }

  Result< unsigned short > getSoftwareVersion() const {
    typedef Result< unsigned short > ResultUS;
    unsigned short hw_version, sw_version, app_num, app_version;
    unsigned int error_code;
    return VCS_GetVersion(device_.handle_.get(), id_, &hw_version, &sw_version, &app_num,
                          &app_version, &error_code) != 0
               ? ResultUS::success(sw_version)
               : ResultUS::error(error_code);
  }

  Result< boost::uint64_t > getSerialNumber() const {
    typedef Result< boost::uint64_t > ResultU64;

    // get device name
    const Result< std::string > result_device_name(device_.getDeviceName());
    if (result_device_name.isError()) {
      return ResultU64::error(result_device_name.errorCode());
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
      return ResultU64::error(0);
    }

    // get the serial number
    boost::uint64_t serial_number;
    const Result< unsigned int > result_get_obj(
        getObject(object_id, object_sub_id, &serial_number, 8));
    if (result_get_obj.isError()) {
      return ResultU64::error(result_get_obj.errorCode());
    }

    return ResultU64::success(serial_number);
  }

  // =======
  // object

  Result< unsigned int > setObject(const unsigned short object_id,
                                   const unsigned short object_sub_id, void *const data,
                                   const unsigned int n_bytes_to_write) {
    typedef Result< unsigned int > ResultUI;
    unsigned int n_bytes_written, error_code;
    return VCS_SetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                         n_bytes_to_write, &n_bytes_written, &error_code) != 0
               ? ResultUI::success(n_bytes_written)
               : ResultUI::error(error_code);
  }

  Result< unsigned int > getObject(const unsigned short object_id,
                                   const unsigned short object_sub_id, void *const data,
                                   const unsigned int n_bytes_to_read) const {
    typedef Result< unsigned int > ResultUI;
    unsigned int n_bytes_read, error_code;
    return VCS_GetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                         n_bytes_to_read, &n_bytes_read, &error_code) != 0
               ? ResultUI::success(n_bytes_read)
               : ResultUI::error(error_code);
  }

  // ===============
  // operation mode

  Result< void > setOperationMode(const char operation_mode) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetOperationMode(device_.handle_.get(), id_, operation_mode, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< char > getOperationMode() const {
    typedef Result< char > ResultC;
    char operation_mode;
    unsigned int error_code;
    return VCS_GetOperationMode(device_.handle_.get(), id_, &operation_mode, &error_code) != 0
               ? ResultC::success(operation_mode)
               : ResultC::error(error_code);
  }

  // ==============
  // state machine

  Result< void > resetDevice() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ResetDevice(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setState(const unsigned short state) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetState(device_.handle_.get(), id_, state, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setEnableState() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetEnableState(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setDisableState() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetDisableState(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setQuickStepState() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetQuickStopState(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > clearFault() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ClearFault(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< unsigned short > getState() const {
    typedef Result< unsigned short > ResultUS;
    unsigned short state;
    unsigned int error_code;
    return VCS_GetState(device_.handle_.get(), id_, &state, &error_code) != 0
               ? ResultUS::success(state)
               : ResultUS::error(error_code);
  }

  Result< bool > getEnableState() const {
    typedef Result< bool > ResultB;
    int is_enabled;
    unsigned int error_code;
    return VCS_GetEnableState(device_.handle_.get(), id_, &is_enabled, &error_code) != 0
               ? ResultB::success(is_enabled == 1)
               : ResultB::error(error_code);
  }

  Result< bool > getDisableState() const {
    typedef Result< bool > ResultB;
    int is_disabled;
    unsigned int error_code;
    return VCS_GetDisableState(device_.handle_.get(), id_, &is_disabled, &error_code) != 0
               ? ResultB::success(is_disabled == 1)
               : ResultB::error(error_code);
  }

  Result< bool > getQuickStopState() const {
    typedef Result< bool > ResultB;
    int is_quick_stopped;
    unsigned int error_code;
    return VCS_GetQuickStopState(device_.handle_.get(), id_, &is_quick_stopped, &error_code) != 0
               ? ResultB::success(is_quick_stopped == 1)
               : ResultB::error(error_code);
  }

  Result< bool > getFaultState() const {
    typedef Result< bool > ResultB;
    int is_in_fault;
    unsigned int error_code;
    return VCS_GetFaultState(device_.handle_.get(), id_, &is_in_fault, &error_code) != 0
               ? ResultB::success(is_in_fault == 1)
               : ResultB::error(error_code);
  }

  // ===============
  // error handling

  Result< unsigned char > getNbOfDeviceError() const {
    typedef Result< unsigned char > ResultUC;
    unsigned char n_device_error;
    unsigned int error_code;
    return VCS_GetNbOfDeviceError(device_.handle_.get(), id_, &n_device_error, &error_code) != 0
               ? ResultUC::success(n_device_error)
               : ResultUC::error(error_code);
  }

  Result< unsigned int > getDeviceErrorCode(const unsigned char device_error_id) const {
    typedef Result< unsigned int > ResultUI;
    unsigned int device_error_code, error_code;
    return VCS_GetDeviceErrorCode(device_.handle_.get(), id_, device_error_id, &device_error_code,
                                  &error_code) != 0
               ? ResultUI::success(device_error_code)
               : ResultUI::error(error_code);
  }

  // =============
  // motion info

  Result< int > getPositionIs() const {
    typedef Result< int > ResultI;
    int position_is;
    unsigned int error_code;
    return VCS_GetPositionIs(device_.handle_.get(), id_, &position_is, &error_code) != 0
               ? ResultI::success(position_is)
               : ResultI::error(error_code);
  }

  Result< int > getVelocityIs() const {
    typedef Result< int > ResultI;
    int velocity_is;
    unsigned int error_code;
    return VCS_GetVelocityIs(device_.handle_.get(), id_, &velocity_is, &error_code) != 0
               ? ResultI::success(velocity_is)
               : ResultI::error(error_code);
  }

  Result< short > getCurrentIs() const {
    typedef Result< short > ResultS;
    short current_is;
    unsigned int error_code;
    return VCS_GetCurrentIs(device_.handle_.get(), id_, &current_is, &error_code) != 0
               ? ResultS::success(current_is)
               : ResultS::error(error_code);
  }

  // ======================
  // profile position mode

  Result< void > activateProfilePositionMode() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ActivateProfilePositionMode(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveToPosition(const long target_position, const bool absolute,
                                const bool immediately) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_MoveToPosition(device_.handle_.get(), id_, target_position, absolute ? 1 : 0,
                              immediately ? 1 : 0, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // ==============
  // position mode

  Result< void > activatePositionMode() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ActivatePositionMode(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setPositionMust(const long position_must) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetPositionMust(device_.handle_.get(), id_, position_must, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // ======================
  // profile velocity mode

  Result< void > activateProfileVelocityMode() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ActivateProfileVelocityMode(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveWithVelocity(const long target_velocity) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_MoveWithVelocity(device_.handle_.get(), id_, target_velocity, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // ==============
  // velocity mode

  Result< void > activateVelocityMode() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ActivateVelocityMode(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setVelocityMust(const long velocity_must) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetVelocityMust(device_.handle_.get(), id_, velocity_must, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // =============
  // current mode

  Result< void > activateCurrentMode() {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_ActivateCurrentMode(device_.handle_.get(), id_, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setCurrentMust(const short current_must) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetCurrentMust(device_.handle_.get(), id_, current_must, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

private:
  Device device_;
  const unsigned short id_;
};
} // namespace epos_command_library_cpp
#endif