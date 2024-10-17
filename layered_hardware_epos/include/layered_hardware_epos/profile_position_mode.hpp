#ifndef LAYERED_HARDWARE_EPOS_PROFILE_POSITION_MODE_HPP
#define LAYERED_HARDWARE_EPOS_PROFILE_POSITION_MODE_HPP

#include <cmath>
#include <limits>
#include <memory>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_context.hpp>
#include <layered_hardware_epos/logging_utils.hpp>
#include <layered_hardware_epos/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_epos {

class ProfilePositionMode : public OperationModeInterface {
public:
  ProfilePositionMode(const std::shared_ptr<EposActuatorContext> &context)
      : OperationModeInterface("profile_position", context) {}

  virtual void starting() override {
    try {
      // switch to position mode
      *context_->node.set_enable_state();
      *context_->node.activate_profile_position_mode();

      // initialize position command using present position
      context_->pos_cmd = *context_->node.get_position();
      prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();

      // initialize profile velocity command using present value
      *context_->node.get_position_profile(&context_->vel_cmd, &prof_acc_, &prof_dec_);
      prev_vel_cmd_ = context_->vel_cmd;

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      LHE_ERROR("ProfilePositionMode::starting(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
      has_started_ = false;
    }
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!has_started_) {
      return;
    }

    try {
      // read actuator states
      context_->pos = *context_->node.get_position();
      context_->vel = *context_->node.get_velocity();
      context_->eff = *context_->node.get_torque();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("ProfilePositionMode::read(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!has_started_) {
      return;
    }

    try {
      // write profile velocity if command has been updated to be positive
      if (context_->vel_cmd > 0. && context_->vel_cmd != prev_vel_cmd_) {
        *context_->node.set_position_profile(context_->vel_cmd, prof_acc_, prof_dec_);
        prev_vel_cmd_ = context_->vel_cmd;
      }

      // write position command if updated
      if (!std::isnan(context_->pos_cmd) && context_->pos_cmd != prev_pos_cmd_) {
        *context_->node.move_to_position(context_->pos_cmd, /* absolute = */ true,
                                         /* immediately = */ true);
        prev_pos_cmd_ = context_->pos_cmd;
      }
    } catch (const eclc::Exception &error) {
      LHE_ERROR("ProfilePositionMode::write(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void stopping() override {
    try {
      *context_->node.set_disable_state();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("ProfilePositionMode::stopping(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

private:
  bool has_started_;
  double prev_pos_cmd_, prev_vel_cmd_, prof_acc_, prof_dec_;
};
} // namespace layered_hardware_epos

#endif