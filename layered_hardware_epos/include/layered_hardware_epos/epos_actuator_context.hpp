#ifndef LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_CONTEXT_HPP
#define LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_CONTEXT_HPP

#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <epos_command_library_cpp/node.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>

namespace layered_hardware_epos {

struct EposActuatorContext {
  // handles
  const std::string name;
  eclc::Node node;

  // states
  double pos = std::numeric_limits<double>::quiet_NaN(),
         vel = std::numeric_limits<double>::quiet_NaN(),
         eff = std::numeric_limits<double>::quiet_NaN();

  // commands
  double pos_cmd = std::numeric_limits<double>::quiet_NaN(),
         vel_cmd = std::numeric_limits<double>::quiet_NaN(),
         eff_cmd = std::numeric_limits<double>::quiet_NaN();
};

// utility functions

static inline std::string get_display_name(const EposActuatorContext &context) {
  std::ostringstream disp_name;
  disp_name << "\"" << context.name
            << "\" actuator (id: " << static_cast<int>(context.node.get_id()) << ")";
  return disp_name.str();
}

} // namespace layered_hardware_epos

#endif