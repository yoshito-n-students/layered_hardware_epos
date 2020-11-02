#ifndef EPOS_COMMAND_LIBRARY_CPP_NODE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_NODE_HPP

#include <cmath> // for M_PI
#include <cstdint>
#include <string>
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

  unsigned short getId() const { return id_; }

  Result< void > getVersion(unsigned short *const hardware_version,
                            unsigned short *const software_version,
                            unsigned short *const application_number,
                            unsigned short *const application_version) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetVersion(device_.handle_.get(), id_, hardware_version, software_version,
                          application_number, application_version, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< std::uint64_t > getSerialNumber() const {
    typedef Result< std::uint64_t > ResultU64;

    // get device name
    const Result< std::string > device_name(device_.getDeviceName());
    if (device_name.isError()) {
      return ResultU64::error(device_name.errorCode());
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
      std::cerr << "Node::getSerialNumber(): Unknown device name '" << *device_name << "'"
                << std::endl;
      return ResultU64::error(0);
    }

    // get the serial number
    std::uint64_t serial_number;
    const Result< unsigned int > n_bytes_read(
        getObject(object_id, object_sub_id, &serial_number, 8));
    if (n_bytes_read.isError()) {
      return ResultU64::error(n_bytes_read.errorCode());
    }

    return ResultU64::success(serial_number);
  }

  // ==============
  // configuration

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

  Result< void > setMotorType(const unsigned short motor_type) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetMotorType(device_.handle_.get(), id_, motor_type, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setDcMotorParameter(const unsigned short nominal_current_ma,
                                     const unsigned short max_output_current_ma,
                                     const unsigned short thermal_time_constant_01sec) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetDcMotorParameter(device_.handle_.get(), id_, nominal_current_ma,
                                   max_output_current_ma, thermal_time_constant_01sec,
                                   &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setDcMotorParameterSI(const double nominal_current_a,
                                       const double max_output_current_a,
                                       const double thermal_time_constant_sec) {
    return setDcMotorParameter(aToMa(nominal_current_a), aToMa(max_output_current_a),
                               static_cast< unsigned short >(thermal_time_constant_sec * 10.));
  }

  Result< void > setEcMotorParameter(const unsigned short nominal_current_ma,
                                     const unsigned short max_output_current_ma,
                                     const unsigned short thermal_time_constant_01sec,
                                     const unsigned char n_pole_pairs) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetEcMotorParameter(device_.handle_.get(), id_, nominal_current_ma,
                                   max_output_current_ma, thermal_time_constant_01sec, n_pole_pairs,
                                   &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setEcMotorParameterSI(const double nominal_current_a,
                                       const double max_output_current_a,
                                       const double thermal_time_constant_sec,
                                       const unsigned char n_pole_pairs) {
    return setEcMotorParameter(aToMa(nominal_current_a), aToMa(max_output_current_a),
                               static_cast< unsigned short >(thermal_time_constant_sec * 10.),
                               n_pole_pairs);
  }

  Result< unsigned short > getMotorType() const {
    typedef Result< unsigned short > ResultUS;
    unsigned short motor_type;
    unsigned int error_code;
    return VCS_GetMotorType(device_.handle_.get(), id_, &motor_type, &error_code) != 0
               ? ResultUS::success(motor_type)
               : ResultUS::error(error_code);
  }

  Result< void > getDcMotorParameter(unsigned short *const nominal_current_ma,
                                     unsigned short *const max_output_current_ma,
                                     unsigned short *const thermal_time_constant_01sec) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetDcMotorParameter(device_.handle_.get(), id_, nominal_current_ma,
                                   max_output_current_ma, thermal_time_constant_01sec,
                                   &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > getDcMotorParameterSI(double *const nominal_current_a,
                                       double *const max_output_current_a,
                                       double *const thermal_time_constant_sec) const {
    typedef Result< void > ResultV;

    unsigned short nominal_current_ma, max_output_current_ma, thermal_time_constant_01sec;
    const ResultV result_get(getDcMotorParameter(&nominal_current_ma, &max_output_current_ma,
                                                 &thermal_time_constant_01sec));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *nominal_current_a = maToA(nominal_current_ma);
    *max_output_current_a = maToA(max_output_current_ma);
    *thermal_time_constant_sec = thermal_time_constant_01sec / 10.;
    return ResultV::success();
  }

  Result< void > getEcMotorParameter(unsigned short *const nominal_current_ma,
                                     unsigned short *const max_output_current_ma,
                                     unsigned short *const thermal_time_constant_01sec,
                                     unsigned char *const n_pole_pairs) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetEcMotorParameter(device_.handle_.get(), id_, nominal_current_ma,
                                   max_output_current_ma, thermal_time_constant_01sec, n_pole_pairs,
                                   &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > getEcMotorParameterSI(double *const nominal_current_a,
                                       double *const max_output_current_a,
                                       double *const thermal_time_constant_sec,
                                       unsigned char *const n_pole_pairs) const {
    typedef Result< void > ResultV;

    unsigned short nominal_current_ma, max_output_current_ma, thermal_time_constant_01sec;
    const ResultV result_get(getEcMotorParameter(&nominal_current_ma, &max_output_current_ma,
                                                 &thermal_time_constant_01sec, n_pole_pairs));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *nominal_current_a = maToA(nominal_current_ma);
    *max_output_current_a = maToA(max_output_current_ma);
    *thermal_time_constant_sec = thermal_time_constant_01sec / 10.;
    return ResultV::success();
  }

  Result< void > setSensorType(const unsigned short sensor_type) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetSensorType(device_.handle_.get(), id_, sensor_type, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setIncEncoderParameter(const unsigned int encoder_resolution,
                                        const bool inverted_polarity) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetIncEncoderParameter(device_.handle_.get(), id_, encoder_resolution,
                                      inverted_polarity ? 1 : 0, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setIncHallSensorParameter(const bool inverted_polarity) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetHallSensorParameter(device_.handle_.get(), id_, inverted_polarity ? 1 : 0,
                                      &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setSsiAbsEncoderParameter(const unsigned short data_rate,
                                           const unsigned short n_multi_turn_data_bits,
                                           const unsigned short n_single_turn_data_bits,
                                           const bool inverted_polarity) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetSsiAbsEncoderParameter(device_.handle_.get(), id_, data_rate,
                                         n_multi_turn_data_bits, n_single_turn_data_bits,
                                         inverted_polarity ? 1 : 0, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setSsiAbsEncoderParameterEx(const unsigned short data_rate,
                                             const unsigned short n_multi_turn_data_bits,
                                             const unsigned short n_single_turn_data_bits,
                                             const unsigned short n_special_data_bits,
                                             const bool inverted_polarity,
                                             const unsigned short timeout,
                                             const unsigned short powerup_time) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetSsiAbsEncoderParameterEx(device_.handle_.get(), id_, data_rate,
                                           n_multi_turn_data_bits, n_single_turn_data_bits,
                                           n_special_data_bits, inverted_polarity ? 1 : 0, timeout,
                                           powerup_time, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  // TODO: setSsiAbsEncoderParameterExSI()

  Result< unsigned short > getSensorType() const {
    typedef Result< unsigned short > ResultUS;
    unsigned short sensor_type;
    unsigned int error_code;
    return VCS_GetSensorType(device_.handle_.get(), id_, &sensor_type, &error_code) != 0
               ? ResultUS::success(sensor_type)
               : ResultUS::error(error_code);
  }

  Result< void > getIncEncoderParameter(unsigned int *const encoder_resolution,
                                        bool *const inverted_polarity) const {
    typedef Result< void > ResultV;
    int inverted_polarity_int;
    unsigned int error_code;
    if (VCS_GetIncEncoderParameter(device_.handle_.get(), id_, encoder_resolution,
                                   &inverted_polarity_int, &error_code) == 0) {
      return ResultV::error(error_code);
    }

    *inverted_polarity = (inverted_polarity_int != 0);
    return ResultV::success();
  }

  Result< void > getHallSensorParameter(bool *const inverted_polarity) const {
    typedef Result< void > ResultV;
    int inverted_polarity_int;
    unsigned int error_code;
    if (VCS_GetHallSensorParameter(device_.handle_.get(), id_, &inverted_polarity_int,
                                   &error_code) == 0) {
      return ResultV::error(error_code);
    }

    *inverted_polarity = (inverted_polarity_int != 0);
    return ResultV::success();
  }

  Result< void > getSsiAbsEncoderParameter(unsigned short *const data_rate,
                                           unsigned short *const n_multi_turn_data_bits,
                                           unsigned short *const n_single_turn_data_bits,
                                           bool *const inverted_polarity) const {
    typedef Result< void > ResultV;

    int inverted_polarity_int;
    unsigned int error_code;
    if (VCS_GetSsiAbsEncoderParameter(device_.handle_.get(), id_, data_rate, n_multi_turn_data_bits,
                                      n_single_turn_data_bits, &inverted_polarity_int,
                                      &error_code) == 0) {
      return ResultV::error(error_code);
    }

    *inverted_polarity = (inverted_polarity_int != 0);
    return ResultV::success();
  }

  Result< void > getSsiAbsEncoderParameterEx(unsigned short *const data_rate,
                                             unsigned short *const n_multi_turn_data_bits,
                                             unsigned short *const n_single_turn_data_bits,
                                             unsigned short *const n_special_data_bits,
                                             bool *const inverted_polarity,
                                             unsigned short *const timeout,
                                             unsigned short *const powerup_time) const {
    typedef Result< void > ResultV;

    int inverted_polarity_int;
    unsigned int error_code;
    if (VCS_GetSsiAbsEncoderParameterEx(
            device_.handle_.get(), id_, data_rate, n_multi_turn_data_bits, n_single_turn_data_bits,
            n_special_data_bits, &inverted_polarity_int, timeout, powerup_time, &error_code) == 0) {
      return ResultV::error(error_code);
    }

    *inverted_polarity = (inverted_polarity_int != 0);
    return ResultV::success();
  }

  // TODO: setSsiAbsEncoderParameterExSI()

  Result< void > setMaxFollowingError(const unsigned int max_following_error_count) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetMaxFollowingError(device_.handle_.get(), id_, max_following_error_count,
                                    &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setMaxFollowingErrorSI(const double max_following_error_rad,
                                        const int count_per_revolution) {
    return setMaxFollowingError(radToCount(max_following_error_rad, count_per_revolution));
  }

  Result< unsigned int > getMaxFollowingError() const {
    typedef Result< unsigned int > ResultUI;
    unsigned int max_following_error_count, error_code;
    return VCS_GetMaxFollowingError(device_.handle_.get(), id_, &max_following_error_count,
                                    &error_code) != 0
               ? ResultUI::success(max_following_error_count)
               : ResultUI::error(error_code);
  }

  Result< double > getMaxFollowingErrorSI(const int count_per_revolution) const {
    typedef Result< double > ResultD;

    const Result< unsigned int > max_following_error_count(getMaxFollowingError());
    if (max_following_error_count.isError()) {
      return ResultD::error(max_following_error_count.errorCode());
    }

    return ResultD::success(countToRad(*max_following_error_count, count_per_revolution));
  }

  Result< void > setControllerGain(const unsigned short e_controller, const unsigned short e_gain,
                                   const unsigned long long value) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetControllerGain(device_.handle_.get(), id_, e_controller, e_gain, value,
                                 &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< unsigned long long > getControllerGain(const unsigned short e_controller,
                                                 const unsigned short e_gain) const {
    typedef Result< unsigned long long > ResultULL;
    unsigned long long value;
    unsigned int error_code;
    return VCS_GetControllerGain(device_.handle_.get(), id_, e_controller, e_gain, &value,
                                 &error_code) != 0
               ? ResultULL::success(value)
               : ResultULL::error(error_code);
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

  Result< std::vector< unsigned int > > getDeviceErrorCodes() const {
    typedef Result< std::vector< unsigned int > > ResultUIV;

    const Result< unsigned char > n_dev_errors(getNbOfDeviceError());
    if (n_dev_errors.isError()) {
      return ResultUIV::error(n_dev_errors.errorCode());
    }

    std::vector< unsigned int > dev_errors;
    for (unsigned char i = 1; i <= *n_dev_errors; ++i) {
      const Result< unsigned int > dev_error(getDeviceErrorCode(i));
      if (dev_error.isError()) {
        return ResultUIV::error(dev_error.errorCode());
      }
      dev_errors.push_back(*dev_error);
    }

    return ResultUIV::success(dev_errors);
  }

  // =============
  // motion info

  // TODO:
  // the following methods assume units of raw position, velocity, and current
  // are counts-of-the-encoder, rpm, and mA, which is the default setting of EPOS.
  // handle other unit settings.

  // get position in counts of encoder
  Result< int > getPositionIs() const {
    typedef Result< int > ResultI;
    int position_count;
    unsigned int error_code;
    return VCS_GetPositionIs(device_.handle_.get(), id_, &position_count, &error_code) != 0
               ? ResultI::success(position_count)
               : ResultI::error(error_code);
  }

  // get position in rad
  Result< double > getPositionSI(const int count_per_revolution) const {
    typedef Result< double > ResultD;
    const Result< int > position_count(getPositionIs());
    return position_count.isSuccess()
               ? ResultD::success(countToRad(*position_count, count_per_revolution))
               : ResultD::error(position_count.errorCode());
  }

  // get velocity in rpm
  Result< int > getVelocityIs() const {
    typedef Result< int > ResultI;
    int velocity_rpm;
    unsigned int error_code;
    return VCS_GetVelocityIs(device_.handle_.get(), id_, &velocity_rpm, &error_code) != 0
               ? ResultI::success(velocity_rpm)
               : ResultI::error(error_code);
  }

  // get velocity in rad/s
  Result< double > getVelocitySI() const {
    typedef Result< double > ResultD;
    const Result< int > velocity_rpm(getVelocityIs());
    return velocity_rpm.isSuccess() ? ResultD::success(rpmToRadps(*velocity_rpm))
                                    : ResultD::error(velocity_rpm.errorCode());
  }

  // get current in mA
  Result< short > getCurrentIs() const {
    typedef Result< short > ResultS;
    short current_ma;
    unsigned int error_code;
    return VCS_GetCurrentIs(device_.handle_.get(), id_, &current_ma, &error_code) != 0
               ? ResultS::success(current_ma)
               : ResultS::error(error_code);
  }

  // get current in A
  Result< double > getCurrentSI() const {
    typedef Result< double > ResultD;
    const Result< short > current_ma(getCurrentIs());
    return current_ma.isSuccess() ? ResultD::success(maToA(*current_ma))
                                  : ResultD::error(current_ma.errorCode());
  }

  Result< double > getTorqueSI(const double torque_constant) const {
    typedef Result< double > ResultD;
    const ResultD current_a(getCurrentSI());
    return current_a.isSuccess() ? ResultD::success(aToN(*current_a, torque_constant))
                                 : ResultD::error(current_a.errorCode());
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

  Result< void > moveToPosition(const long target_position_count, const bool absolute,
                                const bool immediately) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_MoveToPosition(device_.handle_.get(), id_, target_position_count, absolute ? 1 : 0,
                              immediately ? 1 : 0, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveToPositionSI(const double target_position_rad, const int count_per_revolution,
                                  const bool absolute, const bool immediately) {
    return moveToPosition(radToCount(target_position_rad, count_per_revolution), absolute,
                          immediately);
  }

  Result< void > setPositionProfile(const unsigned int profile_velocity_rpm,
                                    const unsigned int profile_acceleration_rpmps,
                                    const unsigned int profile_deceleration_rpmps) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetPositionProfile(device_.handle_.get(), id_, profile_velocity_rpm,
                                  profile_acceleration_rpmps, profile_deceleration_rpmps,
                                  &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setPositionProfileSI(const double profile_velocity_radps,
                                      const double profile_acceleration_radps2,
                                      const double profile_deceleration_radps2) {
    return setPositionProfile(static_cast< unsigned int >(radpsToRpm(profile_velocity_radps)),
                              static_cast< unsigned int >(radpsToRpm(profile_acceleration_radps2)),
                              static_cast< unsigned int >(radpsToRpm(profile_deceleration_radps2)));
  }

  Result< void > getPositionProfile(unsigned int *const profile_velocity_rpm,
                                    unsigned int *const profile_acceleration_rpmps,
                                    unsigned int *const profile_deceleration_rpmps) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetPositionProfile(device_.handle_.get(), id_, profile_velocity_rpm,
                                  profile_acceleration_rpmps, profile_deceleration_rpmps,
                                  &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > getPositionProfileSI(double *const profile_velocity_radps,
                                      double *const profile_acceleration_radps2,
                                      double *const profile_deceleration_radps2) const {
    typedef Result< void > ResultV;

    unsigned int vel_rpm, acc_rpmps, dec_rpmps;
    const ResultV result_get(getPositionProfile(&vel_rpm, &acc_rpmps, &dec_rpmps));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *profile_velocity_radps = rpmToRadps(vel_rpm);
    *profile_acceleration_radps2 = rpmToRadps(acc_rpmps);
    *profile_deceleration_radps2 = rpmToRadps(dec_rpmps);
    return ResultV::success();
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

  Result< void > setPositionMust(const long position_must_count) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetPositionMust(device_.handle_.get(), id_, position_must_count, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setPositionMustSI(const double position_must_rad, const int count_per_revolution) {
    return setPositionMust(radToCount(position_must_rad, count_per_revolution));
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

  Result< void > moveWithVelocity(const long target_velocity_rpm) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_MoveWithVelocity(device_.handle_.get(), id_, target_velocity_rpm, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveWithVelocitySI(const double target_velocity_radps) {
    return moveWithVelocity(radpsToRpm(target_velocity_radps));
  }

  Result< void > setVelocityProfile(const unsigned int profile_acceleration_rpmps,
                                    const unsigned int profile_deceleration_rpmps) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetVelocityProfile(device_.handle_.get(), id_, profile_acceleration_rpmps,
                                  profile_deceleration_rpmps, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setVelocityProfileSI(const double profile_acceleration_radps2,
                                      const double profile_deceleration_radps2) {
    return setVelocityProfile(static_cast< unsigned int >(radpsToRpm(profile_acceleration_radps2)),
                              static_cast< unsigned int >(radpsToRpm(profile_deceleration_radps2)));
  }

  Result< void > getVelocityProfile(unsigned int *const profile_acceleration_rpmps,
                                    unsigned int *const profile_deceleration_rpmps) const {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_GetVelocityProfile(device_.handle_.get(), id_, profile_acceleration_rpmps,
                                  profile_deceleration_rpmps, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > getVelocityProfileSI(double *const profile_acceleration_radps2,
                                      double *const profile_deceleration_radps2) const {
    typedef Result< void > ResultV;

    unsigned int acc_rpmps, dec_rpmps;
    const ResultV result_get(getVelocityProfile(&acc_rpmps, &dec_rpmps));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *profile_acceleration_radps2 = rpmToRadps(acc_rpmps);
    *profile_deceleration_radps2 = rpmToRadps(dec_rpmps);
    return ResultV::success();
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

  Result< void > setVelocityMust(const long velocity_must_rpm) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetVelocityMust(device_.handle_.get(), id_, velocity_must_rpm, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setVelocityMustSI(const double velocity_must_radps) {
    return setVelocityMust(radpsToRpm(velocity_must_radps));
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

  Result< void > setCurrentMust(const short current_must_ma) {
    typedef Result< void > ResultV;
    unsigned int error_code;
    return VCS_SetCurrentMust(device_.handle_.get(), id_, current_must_ma, &error_code) != 0
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setCurrentMustSI(const double current_must_a) {
    return setCurrentMust(aToMa(current_must_a));
  }

  Result< void > setTorqueMustSI(const double torque_must, const double torque_constant) {
    return setCurrentMust(nToA(torque_must, torque_constant));
  }

  // ================
  // unit conversion

  static double countToRad(const long count, const int count_per_revolution) {
    return 2. * M_PI * count / count_per_revolution;
  }

  static long radToCount(const double rad, const int count_per_revolution) {
    return static_cast< long >(rad / (2. * M_PI) * count_per_revolution);
  }

  static double rpmToRadps(const long rpm) { return rpm * M_PI / 30.; }

  static long radpsToRpm(const double radps) { return static_cast< long >(radps / M_PI * 30.); }

  static double maToA(const short ma) { return ma / 1000.; }

  static short aToMa(const double a) { return static_cast< short >(a * 1000.); }

  static double aToN(const double a, const double torque_constant) { return a * torque_constant; }

  static double nToA(const double n, const double torque_constant) { return n / torque_constant; }

private:
  Device device_;
  const unsigned short id_;
};
} // namespace epos_command_library_cpp
#endif