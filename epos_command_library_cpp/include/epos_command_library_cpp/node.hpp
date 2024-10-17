#ifndef EPOS_COMMAND_LIBRARY_CPP_NODE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_NODE_HPP

#include <cmath> // for M_PI
#include <cstdint>
#include <string>
#include <utility> //for std::move()
#include <vector>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/result.hpp>

namespace epos_command_library_cpp {

class Node {
public:
  Node(const Device &device, const unsigned short id) : device_(device), id_(id) {}

  virtual ~Node() {}

  // =====
  // info

  unsigned short get_id() const { return id_; }

  Result<void> get_version(unsigned short *const hardware_version,
                           unsigned short *const software_version,
                           unsigned short *const application_number,
                           unsigned short *const application_version) const {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_GetVersion(device_.handle_.get(), id_, hardware_version, software_version,
                           application_number, application_version, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<std::uint64_t> get_serial_number() const {
    using ResultU64 = Result<std::uint64_t>;

    // get device name
    const auto device_name = device_.get_device_name();
    if (device_name.is_error()) {
      return ResultU64::error(device_name.error_code());
    }

    // find object id of the serial number according to the device name
    unsigned short object_id, object_sub_id;
    if (*device_name == "EPOS" || *device_name == "EPOS2") {
      object_id = 0x2004;
      object_sub_id = 0x00;
    } else if (*device_name == "EPOS4") {
      object_id = 0x2100;
      object_sub_id = 0x01;
    } else {
      // should never reach here
      std::cerr << "Node::get_serial_number(): Unknown device name '" << *device_name << "'"
                << std::endl;
      return ResultU64::error(0);
    }

    // get the serial number
    std::uint64_t serial_number;
    const auto n_bytes_read = get_object(object_id, object_sub_id, &serial_number, 8);
    if (n_bytes_read.is_error()) {
      return ResultU64::error(n_bytes_read.error_code());
    }

    return ResultU64::success(serial_number);
  }

  // ==============
  // configuration

  Result<unsigned int> set_object(const unsigned short object_id,
                                  const unsigned short object_sub_id, void *const data,
                                  const unsigned int n_bytes_to_write) {
    using ResultUI = Result<unsigned int>;
    unsigned int n_bytes_written, error_code;
    return (VCS_SetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                          n_bytes_to_write, &n_bytes_written, &error_code) != 0)
               ? ResultUI::success(n_bytes_written)
               : ResultUI::error(error_code);
  }

  Result<unsigned int> get_object(const unsigned short object_id,
                                  const unsigned short object_sub_id, void *const data,
                                  const unsigned int n_bytes_to_read) const {
    using ResultUI = Result<unsigned int>;
    unsigned int n_bytes_read, error_code;
    return (VCS_GetObject(device_.handle_.get(), id_, object_id, object_sub_id, data,
                          n_bytes_to_read, &n_bytes_read, &error_code) != 0)
               ? ResultUI::success(n_bytes_read)
               : ResultUI::error(error_code);
  }

  Result<unsigned short> get_motor_type() const {
    using ResultUS = Result<unsigned short>;
    unsigned short motor_type;
    unsigned int error_code;
    return (VCS_GetMotorType(device_.handle_.get(), id_, &motor_type, &error_code) != 0)
               ? ResultUS::success(motor_type)
               : ResultUS::error(error_code);
  }

  Result<unsigned short> get_sensor_type() const {
    using ResultUS = Result<unsigned short>;
    unsigned short sensor_type;
    unsigned int error_code;
    return (VCS_GetSensorType(device_.handle_.get(), id_, &sensor_type, &error_code) != 0)
               ? ResultUS::success(sensor_type)
               : ResultUS::error(error_code);
  }

  // ===============
  // operation mode

  Result<void> set_operation_mode(const char operation_mode) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetOperationMode(device_.handle_.get(), id_, operation_mode, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<char> get_operation_mode() const {
    using ResultC = Result<char>;
    char operation_mode;
    unsigned int error_code;
    return (VCS_GetOperationMode(device_.handle_.get(), id_, &operation_mode, &error_code) != 0)
               ? ResultC::success(operation_mode)
               : ResultC::error(error_code);
  }

  // ==============
  // state machine

  Result<void> reset_device() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ResetDevice(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_state(const unsigned short state) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetState(device_.handle_.get(), id_, state, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_enable_state() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetEnableState(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_disable_state() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetDisableState(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_quick_stop_state() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetQuickStopState(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> clear_fault() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ClearFault(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<unsigned short> get_state() const {
    using ResultUS = Result<unsigned short>;
    unsigned short state;
    unsigned int error_code;
    return (VCS_GetState(device_.handle_.get(), id_, &state, &error_code) != 0)
               ? ResultUS::success(state)
               : ResultUS::error(error_code);
  }

  Result<bool> get_enable_state() const {
    using ResultB = Result<bool>;
    int is_enabled;
    unsigned int error_code;
    return (VCS_GetEnableState(device_.handle_.get(), id_, &is_enabled, &error_code) != 0)
               ? ResultB::success(is_enabled == 1)
               : ResultB::error(error_code);
  }

  Result<bool> get_disable_state() const {
    using ResultB = Result<bool>;
    int is_disabled;
    unsigned int error_code;
    return (VCS_GetDisableState(device_.handle_.get(), id_, &is_disabled, &error_code) != 0)
               ? ResultB::success(is_disabled == 1)
               : ResultB::error(error_code);
  }

  Result<bool> get_quick_stop_state() const {
    using ResultB = Result<bool>;
    int is_quick_stopped;
    unsigned int error_code;
    return (VCS_GetQuickStopState(device_.handle_.get(), id_, &is_quick_stopped, &error_code) != 0)
               ? ResultB::success(is_quick_stopped == 1)
               : ResultB::error(error_code);
  }

  Result<bool> get_fault_state() const {
    using ResultB = Result<bool>;
    int is_in_fault;
    unsigned int error_code;
    return (VCS_GetFaultState(device_.handle_.get(), id_, &is_in_fault, &error_code) != 0)
               ? ResultB::success(is_in_fault == 1)
               : ResultB::error(error_code);
  }

  // ===============
  // error handling

  Result<std::vector<unsigned int>> get_device_error_codes() const {
    using ResultUIV = Result<std::vector<unsigned int>>;
    unsigned int error_code;

    unsigned char n_dev_errors;
    if (VCS_GetNbOfDeviceError(device_.handle_.get(), id_, &n_dev_errors, &error_code) == 0) {
      return ResultUIV::error(error_code);
    }

    std::vector<unsigned int> dev_errors;
    dev_errors.reserve(n_dev_errors);
    for (unsigned char i = 1; i <= n_dev_errors; ++i) {
      unsigned int dev_error;
      if (VCS_GetDeviceErrorCode(device_.handle_.get(), id_, i, &dev_error, &error_code) == 0) {
        return ResultUIV::error(error_code);
      }
      dev_errors.push_back(dev_error);
    }

    return ResultUIV::success(std::move(dev_errors));
  }

  // =============
  // motion info

  // TODO:
  // the following methods assume units of raw position, velocity, and current
  // are counts-of-the-encoder, rpm, and mA, which is the default setting of EPOS.
  // handle other unit settings.

  // get position in rad
  Result<double> get_position(const int count_per_revolution) const {
    using ResultD = Result<double>;
    int position_count;
    unsigned int error_code;
    return (VCS_GetPositionIs(device_.handle_.get(), id_, &position_count, &error_code) != 0)
               ? ResultD::success(count_to_rad(position_count, count_per_revolution))
               : ResultD::error(error_code);
  }

  // get velocity in rad/s
  Result<double> get_velocity() const {
    using ResultD = Result<double>;
    int velocity_rpm;
    unsigned int error_code;
    return (VCS_GetVelocityIs(device_.handle_.get(), id_, &velocity_rpm, &error_code) != 0)
               ? ResultD::success(rpm_to_radps(velocity_rpm))
               : ResultD::error(error_code);
  }

  // get current in mA
  Result<short> get_current_is() const {
    using ResultS = Result<short>;
    short current_ma;
    unsigned int error_code;
    return (VCS_GetCurrentIs(device_.handle_.get(), id_, &current_ma, &error_code) != 0)
               ? ResultS::success(current_ma)
               : ResultS::error(error_code);
  }

  // get current in A
  Result<double> get_current() const {
    using ResultD = Result<double>;
    short current_ma;
    unsigned int error_code;
    return (VCS_GetCurrentIs(device_.handle_.get(), id_, &current_ma, &error_code) != 0)
               ? ResultD::success(ma_to_a(current_ma))
               : ResultD::error(error_code);
  }

  Result<double> get_torque(const double torque_constant) const {
    using ResultD = Result<double>;
    const auto current_a = get_current();
    return current_a.is_success() ? ResultD::success(a_to_nm(*current_a, torque_constant))
                                  : ResultD::error(current_a.error_code());
  }

  // ======================
  // profile position mode

  Result<void> activate_profile_position_mode() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ActivateProfilePositionMode(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> move_to_position(const double target_position_rad, const int count_per_revolution,
                                const bool absolute, const bool immediately) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_MoveToPosition(device_.handle_.get(), id_,
                               rad_to_count(target_position_rad, count_per_revolution),
                               absolute ? 1 : 0, immediately ? 1 : 0, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_position_profile(const double profile_velocity_radps,
                                    const double profile_acceleration_radps2,
                                    const double profile_deceleration_radps2) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetPositionProfile(
                device_.handle_.get(), id_,
                static_cast<unsigned int>(radps_to_rpm(profile_velocity_radps)),
                static_cast<unsigned int>(radps_to_rpm(profile_acceleration_radps2)),
                static_cast<unsigned int>(radps_to_rpm(profile_deceleration_radps2)),
                &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> get_position_profile(double *const profile_velocity_radps,
                                    double *const profile_acceleration_radps2,
                                    double *const profile_deceleration_radps2) const {
    using ResultV = Result<void>;

    unsigned int vel_rpm, acc_rpmps, dec_rpmps, error_code;
    if (VCS_GetPositionProfile(device_.handle_.get(), id_, &vel_rpm, &acc_rpmps, &dec_rpmps,
                               &error_code) == 0) {
      return ResultV::error(error_code);
    }

    *profile_velocity_radps = rpm_to_radps(vel_rpm);
    *profile_acceleration_radps2 = rpm_to_radps(acc_rpmps);
    *profile_deceleration_radps2 = rpm_to_radps(dec_rpmps);
    return ResultV::success();
  }

  // ==============
  // position mode

  Result<void> activate_position_mode() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ActivatePositionMode(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_position_must(const double position_must_rad, const int count_per_revolution) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetPositionMust(device_.handle_.get(), id_,
                                rad_to_count(position_must_rad, count_per_revolution),
                                &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // ======================
  // profile velocity mode

  Result<void> activate_profile_velocity_mode() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ActivateProfileVelocityMode(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> move_with_velocity(const double target_velocity_radps) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_MoveWithVelocity(device_.handle_.get(), id_, radps_to_rpm(target_velocity_radps),
                                 &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_velocity_profile(const double profile_acceleration_radps2,
                                    const double profile_deceleration_radps2) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return VCS_SetVelocityProfile(
               device_.handle_.get(), id_,
               static_cast<unsigned int>(radps_to_rpm(profile_acceleration_radps2)),
               static_cast<unsigned int>(radps_to_rpm(profile_deceleration_radps2)),
               &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> get_velocity_profile(double *const profile_acceleration_radps2,
                                    double *const profile_deceleration_radps2) const {
    using ResultV = Result<void>;
    unsigned int acc_rpmps, dec_rpmps, error_code;
    if (VCS_GetVelocityProfile(device_.handle_.get(), id_, &acc_rpmps, &dec_rpmps, &error_code) ==
        0) {
      return ResultV::error(error_code);
    }

    *profile_acceleration_radps2 = rpm_to_radps(acc_rpmps);
    *profile_deceleration_radps2 = rpm_to_radps(dec_rpmps);
    return ResultV::success();
  }

  // ==============
  // velocity mode

  Result<void> activate_velocity_mode() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ActivateVelocityMode(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_velocity_must(const double velocity_must_radps) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_SetVelocityMust(device_.handle_.get(), id_, radps_to_rpm(velocity_must_radps),
                                &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // =============
  // current mode

  Result<void> activate_current_mode() {
    using ResultV = Result<void>;
    unsigned int error_code;
    return (VCS_ActivateCurrentMode(device_.handle_.get(), id_, &error_code) != 0)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_current_must(const double current_must_a) {
    using ResultV = Result<void>;
    unsigned int error_code;
    return VCS_SetCurrentMust(device_.handle_.get(), id_, a_to_ma(current_must_a), &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result<void> set_torque_must(const double torque_must, const double torque_constant) {
    return set_current_must(nm_to_a(torque_must, torque_constant));
  }

  // ================
  // unit conversion

  static double count_to_rad(const long count, const int count_per_revolution) {
    return 2. * M_PI * count / count_per_revolution;
  }

  static long rad_to_count(const double rad, const int count_per_revolution) {
    return static_cast<long>(rad / (2. * M_PI) * count_per_revolution);
  }

  static double rpm_to_radps(const long rpm) { return rpm * M_PI / 30.; }

  static long radps_to_rpm(const double radps) { return static_cast<long>(radps / M_PI * 30.); }

  static double ma_to_a(const short ma) { return ma / 1000.; }

  static short a_to_ma(const double a) { return static_cast<short>(a * 1000.); }

  static double a_to_nm(const double a, const double torque_constant) {
    return a * torque_constant;
  }

  static double nm_to_a(const double nm, const double torque_constant) {
    return nm / torque_constant;
  }

private:
  Device device_;
  const unsigned short id_;
};
} // namespace epos_command_library_cpp
#endif