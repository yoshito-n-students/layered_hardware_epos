#ifndef LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_HPP
#define LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_HPP

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/node.hpp>
#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/string_registry.hpp>
#include <layered_hardware_epos/clear_fault_mode.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/current_mode.hpp>
#include <layered_hardware_epos/disable_mode.hpp>
#include <layered_hardware_epos/epos_actuator_context.hpp>
#include <layered_hardware_epos/logging_utils.hpp>
#include <layered_hardware_epos/operation_mode_interface.hpp>
#include <layered_hardware_epos/position_mode.hpp>
#include <layered_hardware_epos/profile_position_mode.hpp>
#include <layered_hardware_epos/profile_velocity_mode.hpp>
#include <layered_hardware_epos/reset_mode.hpp>
#include <layered_hardware_epos/velocity_mode.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware_epos {

class EposActuator {
public:
  EposActuator(const std::string &name, const YAML::Node &params, const eclc::Device &device) {
    // parse parameters for this actuator
    unsigned short id;
    int count_per_revolution;
    double torque_constant;
    std::vector<std::string> mapped_mode_names;
    try {
      id = static_cast<unsigned short>(params["id"].as<int>());
      count_per_revolution = params["count_per_revolution"].as<int>();
      torque_constant = params["torque_constant"].as<double>();
      for (const auto &iface_mode_name_pair : params["operation_mode_map"]) {
        bound_interfaces_.emplace_back(iface_mode_name_pair.first.as<std::string>());
        mapped_mode_names.emplace_back(iface_mode_name_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameters for \"" + name +
                               "\" actuator: " + error.what());
    }

    // allocate context
    context_.reset(new EposActuatorContext{
        name, eclc::Node(device, id, count_per_revolution, torque_constant)});

    // make operating mode map from ros-controller name to EPOS's operation mode
    for (const auto &mode_name : mapped_mode_names) {
      try {
        mapped_modes_.emplace_back(make_operation_mode(mode_name));
      } catch (const std::runtime_error &error) {
        throw std::runtime_error("Invalid value in \"operation_mode_map\" parameter for \"" + name +
                                 "\" actuator: " + error.what());
      }
    }
  }

  virtual ~EposActuator() {
    // finalize the present mode
    switch_operation_modes(/* new_mode = */ nullptr);
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to actuator states owned by this actuator
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to actuator commands owned by this actuator
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(context_->name, hi::HW_IF_POSITION, &context_->pos_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_VELOCITY, &context_->vel_cmd);
    ifaces.emplace_back(context_->name, hi::HW_IF_EFFORT, &context_->eff_cmd);
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      LHE_ERROR("EposActuator::prepare_command_mode_switch(): "
                "Reject mode switching of \"%s\" actuator "
                "because %zd bound interfaces are about to be active",
                context_->name.c_str(), active_bound_ifaces.size());
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const lh::StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      LHE_ERROR("EposActuator::perform_command_mode_switch(): "
                "Could not switch mode of \"%s\" actuator "
                "because %zd bound interfaces are active",
                context_->name.c_str(), bound_interfaces_.size());
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    if (!active_bound_ifaces.empty()) { // active_bound_ifaces.size() == 1
      switch_operation_modes(mapped_modes_[active_bound_ifaces.front()]);
    } else { // active_bound_ifaces.size() == 0
      switch_operation_modes(nullptr);
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
    return hi::return_type::OK; // TODO: return result of read
  }

  hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
    return hi::return_type::OK; // TODO: return result of write
  }

private:
  std::shared_ptr<OperationModeInterface> make_operation_mode(const std::string &mode_str) const {
    if (mode_str == "clear_fault") {
      return std::make_shared<ClearFaultMode>(context_);
    } else if (mode_str == "current") {
      return std::make_shared<CurrentMode>(context_);
    } else if (mode_str == "disable") {
      return std::make_shared<DisableMode>(context_);
    } else if (mode_str == "position") {
      return std::make_shared<PositionMode>(context_);
    } else if (mode_str == "profile_position") {
      return std::make_shared<ProfilePositionMode>(context_);
    } else if (mode_str == "profile_velocity") {
      return std::make_shared<ProfileVelocityMode>(context_);
    } else if (mode_str == "reset") {
      return std::make_shared<ResetMode>(context_);
    } else if (mode_str == "velocity") {
      return std::make_shared<VelocityMode>(context_);
    } else {
      throw std::runtime_error("Unknown operation mode name \"" + mode_str + "\"");
    }
  }

  void switch_operation_modes(const std::shared_ptr<OperationModeInterface> &new_mode) {
    // do nothing if no mode switch is requested
    if (present_mode_ == new_mode) {
      return;
    }
    // stop present mode
    if (present_mode_) {
      LHE_INFO("EposActuator::switch_operation_modes(): "
               "Stopping \"%s\" operation mode for \"%s\" actuator",
               present_mode_->get_name().c_str(), context_->name.c_str());
      present_mode_->stopping();
      present_mode_.reset();
    }
    // start new mode
    if (new_mode) {
      LHE_INFO("EposActuator::switch_operation_modes(): "
               "Starting \"%s\" operation mode for \"%s\" actuator",
               new_mode->get_name().c_str(), context_->name.c_str());
      new_mode->starting();
      present_mode_ = new_mode;
    }
  }

private:
  std::shared_ptr<EposActuatorContext> context_;

  // present operating mode
  std::shared_ptr<OperationModeInterface> present_mode_;
  // map from command interface to operating mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<std::shared_ptr<OperationModeInterface>> mapped_modes_;
};

} // namespace layered_hardware_epos

#endif