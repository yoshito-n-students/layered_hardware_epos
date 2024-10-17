#ifndef LAYERED_HARDWARE_EPOS_DISABLE_MODE_HPP
#define LAYERED_HARDWARE_EPOS_DISABLE_MODE_HPP

#include <memory>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_context.hpp>
#include <layered_hardware_epos/logging_utils.hpp>
#include <layered_hardware_epos/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_epos {

// TODO: consider naming; passive mode? torque disable mode? monitor mode?
class DisableMode : public OperationModeInterface {
public:
  DisableMode(const std::shared_ptr<EposActuatorContext> &context)
      : OperationModeInterface("disable", context) {}

  virtual void starting() override {
    try {
      *context_->node.set_disable_state();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("DisableMode::starting(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    try {
      context_->pos = *context_->node.get_position();
      context_->vel = *context_->node.get_velocity();
      context_->eff = *context_->node.get_torque();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("DisableMode::read(): %s: %s", get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do
  }

  virtual void stopping() override {
    // nothing to do
  }
};

} // namespace layered_hardware_epos

#endif