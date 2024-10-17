#ifndef LAYERED_HARDWARE_EPOS_CLEAR_FAULT_MODE_HPP
#define LAYERED_HARDWARE_EPOS_CLEAR_FAULT_MODE_HPP

#include <memory>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_context.hpp>
#include <layered_hardware_epos/logging_utils.hpp>
#include <layered_hardware_epos/operation_mode_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware_epos {

class ClearFaultMode : public OperationModeInterface {
public:
  ClearFaultMode(const std::shared_ptr<EposActuatorContext> &context)
      : OperationModeInterface("clear_fault", context) {}

  virtual void starting() override {
    try {
      *context_->node.clear_fault();
    } catch (const eclc::Exception &error) {
      LHE_ERROR("ClearFaultMode::starting(): %s: %s", //
                get_display_name(*context_).c_str(), error.what());
    }
  }

  virtual void read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // nothing to do
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