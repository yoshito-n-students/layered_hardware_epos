#ifndef EPOS_COMMAND_LIBRARY_CPP_NODE_HPP
#define EPOS_COMMAND_LIBRARY_CPP_NODE_HPP

#include <cmath> // for M_PI
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

  Result< boost::uint64_t > getSerialNumber() const {
    typedef Result< boost::uint64_t > ResultU64;

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
    boost::uint64_t serial_number;
    const Result< unsigned int > n_bytes_read(
        getObject(object_id, object_sub_id, &serial_number, 8));
    if (n_bytes_read.isError()) {
      return ResultU64::error(n_bytes_read.errorCode());
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
  Result< double > getPosition(const int count_per_revolution) const {
    typedef Result< double > ResultD;
    const Result< int > position_count(getPositionIs());
    return position_count.isSuccess()
               ? ResultD::success(2. * M_PI * *position_count / count_per_revolution)
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
  Result< double > getVelocity() const {
    typedef Result< double > ResultD;
    const Result< int > velocity_rpm(getVelocityIs());
    return velocity_rpm.isSuccess() ? ResultD::success(M_PI * *velocity_rpm / 30.)
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
  Result< double > getCurrent() const {
    typedef Result< double > ResultD;
    const Result< short > current_ma(getCurrentIs());
    return current_ma.isSuccess() ? ResultD::success(*current_ma / 1000.)
                                  : ResultD::error(current_ma.errorCode());
  }

  Result< double > getTorque(const double torque_constant) const {
    typedef Result< double > ResultD;
    const ResultD current_a(getCurrent());
    return current_a.isSuccess() ? ResultD::success(*current_a * torque_constant)
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
                              immediately ? 1 : 0, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveToPosition(const double target_position_rad, const int count_per_revolution,
                                const bool absolute, const bool immediately) {
    return moveToPosition(
        static_cast< long >(target_position_rad / (2. * M_PI) * count_per_revolution), absolute,
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

  Result< void > setPositionProfile(const double profile_velocity_radps,
                                    const double profile_acceleration_radps2,
                                    const double profile_deceleration_radps2) {
    return setPositionProfile(
        static_cast< unsigned int >(profile_velocity_radps / M_PI * 30.),
        static_cast< unsigned int >(profile_acceleration_radps2 / M_PI * 30.),
        static_cast< unsigned int >(profile_deceleration_radps2 / M_PI * 30.));
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

  Result< void > getPositionProfile(double *const profile_velocity_radps,
                                    double *const profile_acceleration_radps2,
                                    double *const profile_deceleration_radps2) const {
    typedef Result< void > ResultV;

    unsigned int vel_rpm, acc_rpmps, dec_rpmps;
    const ResultV result_get(getPositionProfile(&vel_rpm, &acc_rpmps, &dec_rpmps));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *profile_velocity_radps = vel_rpm * M_PI / 30.;
    *profile_acceleration_radps2 = acc_rpmps * M_PI / 30.;
    *profile_deceleration_radps2 = dec_rpmps * M_PI / 30.;
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
    return VCS_SetPositionMust(device_.handle_.get(), id_, position_must_count, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setPositionMust(const double position_must_rad, const int count_per_revolution) {
    return setPositionMust(
        static_cast< long >(position_must_rad / (2. * M_PI) * count_per_revolution));
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
    return VCS_MoveWithVelocity(device_.handle_.get(), id_, target_velocity_rpm, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > moveWithVelocity(const double target_velocity_radps) {
    return moveWithVelocity(static_cast< long >(target_velocity_radps / M_PI * 30.));
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

  Result< void > setVelocityProfile(const double profile_acceleration_radps2,
                                    const double profile_deceleration_radps2) {
    return setVelocityProfile(
        static_cast< unsigned int >(profile_acceleration_radps2 / M_PI * 30.),
        static_cast< unsigned int >(profile_deceleration_radps2 / M_PI * 30.));
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

  Result< void > getVelocityProfile(double *const profile_acceleration_radps2,
                                    double *const profile_deceleration_radps2) const {
    typedef Result< void > ResultV;

    unsigned int acc_rpmps, dec_rpmps;
    const ResultV result_get(getVelocityProfile(&acc_rpmps, &dec_rpmps));
    if (result_get.isError()) {
      return ResultV::error(result_get.errorCode());
    }

    *profile_acceleration_radps2 = acc_rpmps * M_PI / 30.;
    *profile_deceleration_radps2 = dec_rpmps * M_PI / 30.;
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
    return VCS_SetVelocityMust(device_.handle_.get(), id_, velocity_must_rpm, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setVelocityMust(const double velocity_must_radps) {
    return setVelocityMust(static_cast< long >(velocity_must_radps / M_PI * 30.));
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
    return VCS_SetCurrentMust(device_.handle_.get(), id_, current_must_ma, &error_code)
               ? ResultV::success()
               : ResultV::error(error_code);
  }

  Result< void > setCurrentMust(const double current_must_a) {
    return setCurrentMust(static_cast< short >(current_must_a * 1000.));
  }

  Result< void > setTorqueMust(const double torque_must, const double torque_constant) {
    return setCurrentMust(torque_must / torque_constant);
  }

private:
  Device device_;
  const unsigned short id_;
};
} // namespace epos_command_library_cpp
#endif