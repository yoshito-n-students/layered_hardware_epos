#ifndef LAYERED_HARDWARE_EPOS_CURRENT_MODE_HPP
#define LAYERED_HARDWARE_EPOS_CURRENT_MODE_HPP

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

class CurrentMode : public OperationModeInterface {
public:
  CurrentMode(const std::shared_ptr<EposActuatorContext> &context)
      : OperationModeInterface("current", context) {}

  virtual void starting() override {
    try {
      // switch to current mode
      *context_->node.set_enable_state();
      *context_->node.activate_current_mode();

      // set reasonable initial command
      context_->eff_cmd = 0.;
      prev_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      LHE_ERROR("CurrentMode::starting(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
      has_started_ = false;
    }
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!has_started_) {
      return;
    }

    try {
      context_->pos = *context_->node.get_position();
      context_->vel = *context_->node.get_velocity();
      context_->eff = *context_->node.get_torque();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("CurrentMode::read(): %s: %s", get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (!has_started_) {
      return;
    }

    try {
      if (!std::isnan(context_->eff_cmd) && context_->eff_cmd != prev_eff_cmd_) {
        *context_->node.set_torque_must(context_->eff_cmd);
        prev_eff_cmd_ = context_->eff_cmd;
      }
    } catch (const eclc::Exception &error) {
      LHE_ERROR("CurrentMode::write(): %s: %s", get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void stopping() override {
    try {
      *context_->node.set_disable_state();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("CurrentMode::stopping(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

private:
  bool has_started_;
  double prev_eff_cmd_;
};
} // namespace layered_hardware_epos

#endif