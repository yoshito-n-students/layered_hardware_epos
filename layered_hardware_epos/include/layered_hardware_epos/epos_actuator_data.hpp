#ifndef LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_DATA_HPP
#define LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_DATA_HPP

#include <string>

#include <epos_command_library_cpp/node.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_epos {

struct EposActuatorData {
  EposActuatorData(const std::string &_name, const eclc::Node &_node,
                   const int _count_per_revolution, const double _torque_constant)
      : name(_name), node(_node), count_per_revolution(_count_per_revolution),
        torque_constant(_torque_constant), pos(0.), vel(0.), eff(0.), pos_cmd(0.), vel_cmd(0.),
        eff_cmd(0.) {}

  // useful when printing node-specific error info
  std::string nodeDescription() const {
    return "'" + name + "' (id = " + boost::lexical_cast< std::string >(node.getId()) + ")";
  }

  // handles
  const std::string name;
  eclc::Node node;

  // params
  const int count_per_revolution;
  const double torque_constant;

  // states
  double pos, vel, eff;

  // commands
  double pos_cmd, vel_cmd, eff_cmd;
};

typedef boost::shared_ptr< EposActuatorData > EposActuatorDataPtr;
typedef boost::shared_ptr< const EposActuatorData > EposActuatorDataConstPtr;

} // namespace layered_hardware_epos

#endif