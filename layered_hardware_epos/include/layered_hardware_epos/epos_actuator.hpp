#ifndef LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_HPP
#define LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_HPP

#include <list>
#include <map>
#include <string>

#include <epos_command_library_cpp/device.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/current_mode.hpp>
#include <layered_hardware_epos/disable_mode.hpp>
#include <layered_hardware_epos/operating_mode_base.hpp>
#include <layered_hardware_epos/position_mode.hpp>
#include <layered_hardware_epos/profile_position_mode.hpp>
#include <layered_hardware_epos/profile_velocity_mode.hpp>
#include <layered_hardware_epos/velocity_mode.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_epos {

class EposActuator {
public:
  EposActuator() {}

  virtual ~EposActuator() {
    // finalize the present mode
    if (present_mode_) {
      ROS_INFO_STREAM("EposActuator::~EposActuator(): Stopping operating mode '"
                      << present_mode_->getName() << "' for actuator '" << data_->name
                      << "' (id: " << data_->node.getId() << ")");
      present_mode_->stopping();
      present_mode_ = OperatingModePtr();
    }
  }

  bool init(const std::string &name, const eclc::Device &device, hi::RobotHW *const hw,
            const ros::NodeHandle &param_nh) {
    // epos node id from param
    int id;
    if (!param_nh.getParam("id", id)) {
      ROS_ERROR_STREAM("EposActuator::init(): Failed to get param '" << param_nh.resolveName("id")
                                                                     << "'");
      return false;
    }

    // encoder count per resolution from param
    int count_per_revolution;
    if (!param_nh.getParam("count_per_revolution", count_per_revolution)) {
      ROS_ERROR_STREAM("EposActuator::init(): Failed to get param '"
                       << param_nh.resolveName("count_per_revolution") << "'");
      return false;
    }

    // torque constant from param
    double torque_constant;
    if (!param_nh.getParam("torque_constant", torque_constant)) {
      ROS_ERROR_STREAM("EposActuator::init(): Failed to get param '"
                       << param_nh.resolveName("torque_constant") << "'");
      return false;
    }

    // allocate data structure
    data_.reset(
        new EposActuatorData(name, eclc::Node(device, id), count_per_revolution, torque_constant));

    // register actuator states & commands to corresponding hardware interfaces
    const hi::ActuatorStateHandle state_handle(data_->name, &data_->pos, &data_->vel, &data_->eff);
    if (!registerActuatorTo< hi::ActuatorStateInterface >(hw, state_handle) ||
        !registerActuatorTo< hi::PositionActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->pos_cmd)) ||
        !registerActuatorTo< hi::VelocityActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->vel_cmd)) ||
        !registerActuatorTo< hi::EffortActuatorInterface >(
            hw, hi::ActuatorHandle(state_handle, &data_->eff_cmd))) {
      return false;
    }

    // make operating mode map from ros-controller name to dynamixel's operating mode
    typedef std::map< std::string, std::string > ModeNameMap;
    ModeNameMap mode_name_map;
    if (!param_nh.getParam("operating_mode_map", mode_name_map)) {
      ROS_ERROR_STREAM("EposActuator::init(): Failed to get param '"
                       << param_nh.resolveName("operating_mode_map") << "'");
      return false;
    }

    return true;
  }

  bool prepareSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                     const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // check if switching is possible by counting number of operating modes after switching

    // number of modes before switching
    std::size_t n_modes(present_mode_ ? 1 : 0);

    // number of modes after stopping controllers
    if (n_modes != 0) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          n_modes = 0;
          break;
        }
      }
    }

    // number of modes after starting controllers
    BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
      const std::map< std::string, OperatingModePtr >::const_iterator mode_to_start(
          mode_map_.find(starting_controller.name));
      if (mode_to_start != mode_map_.end() && mode_to_start->second) {
        ++n_modes;
      }
    }

    // assert 0 or 1 operating modes. multiple modes are impossible.
    if (n_modes != 0 && n_modes != 1) {
      ROS_ERROR_STREAM("EposActuator::prepareSwitch(): "
                       << data_->nodeDescription() << ": Rejected unfeasible controller switching");
      return false;
    }

    return true;
  }

  void doSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // stop actuator's operating mode according to stopping controller list
    if (present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_stop(
            mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          ROS_INFO_STREAM("EposActuator::doSwitch(): " << data_->nodeDescription()
                                                       << ": Stopping operating mode '"
                                                       << present_mode_->getName() << "'");
          present_mode_->stopping();
          present_mode_ = OperatingModePtr();
          break;
        }
      }
    }

    // start actuator's operating modes according to starting controllers
    if (!present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
        const std::map< std::string, OperatingModePtr >::const_iterator mode_to_start(
            mode_map_.find(starting_controller.name));
        if (mode_to_start != mode_map_.end() && mode_to_start->second) {
          ROS_INFO_STREAM("EposActuator::doSwitch(): " << data_->nodeDescription()
                                                       << ": Starting operating mode '"
                                                       << mode_to_start->second->getName() << "'");
          present_mode_ = mode_to_start->second;
          present_mode_->starting();
          break;
        }
      }
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
  }

private:
  template < typename Interface, typename Handle >
  static bool registerActuatorTo(hi::RobotHW *const hw, const Handle &handle) {
    Interface *const iface(hw->get< Interface >());
    if (!iface) {
      ROS_ERROR("EposActuator::registerActuatorTo(): Failed to get a hardware interface");
      return false;
    }
    iface->registerHandle(handle);
    return true;
  }

  OperatingModePtr makeOperatingMode(const std::string &mode_str,
                                     const std::map< std::string, int > &item_map) {
    if (mode_str == "current") {
      return boost::make_shared< CurrentMode >(data_);
    } else if (mode_str == "disable") {
      return boost::make_shared< DisableMode >(data_);
    } else if (mode_str == "position") {
      return boost::make_shared< PositionMode >(data_);
    } else if (mode_str == "profile_position") {
      return boost::make_shared< ProfilePositionMode >(data_);
    } else if (mode_str == "profile_velocity") {
      return boost::make_shared< ProfileVelocityMode >(data_);
    } else if (mode_str == "reboot") {
      // return boost::make_shared< RebootMode >(data_);
    } else if (mode_str == "velocity") {
      return boost::make_shared< VelocityMode >(data_);
    } else {
      ROS_ERROR_STREAM("EposActuator::makeOperatingMode(): " << data_->nodeDescription()
                                                             << ": Unknown operating mode name '"
                                                             << mode_str << "'");
      return OperatingModePtr();
    }
  }

private:
  EposActuatorDataPtr data_;

  std::map< std::string, OperatingModePtr > mode_map_;
  OperatingModePtr present_mode_;
};

typedef boost::shared_ptr< EposActuator > EposActuatorPtr;
typedef boost::shared_ptr< const EposActuator > EposActuatorConstPtr;
} // namespace layered_hardware_epos

#endif