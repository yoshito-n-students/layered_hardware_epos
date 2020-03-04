#ifndef LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_EPOS_EPOS_ACTUATOR_LAYER_HPP

#include <list>
#include <string>
#include <vector>

#include <epos_command_library_cpp/device.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/foreach.hpp>

namespace layered_hardware_epos {

class EposActuatorLayer : public lh::LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    namespace rp = ros::param;

    // make actuator interfaces registered to the hardware
    // so that other layers can find the interfaces
    makeRegistered< hi::ActuatorStateInterface >(hw);
    makeRegistered< hi::PositionActuatorInterface >(hw);
    makeRegistered< hi::VelocityActuatorInterface >(hw);
    makeRegistered< hi::EffortActuatorInterface >(hw);

    // open epos device
    eclc::Result< eclc::Device > device(eclc::Device::open(
        // equivarent of param_nh.param< std::string >("device", "EPOS4") .
        // but we cannot call this becase NodeHandle::param() is not a const function (why???)
        rp::param< std::string >(param_nh.resolveName("device"), "EPOS4"),
        rp::param< std::string >(param_nh.resolveName("protocol_stack"), "MAXON SERIAL V2"),
        rp::param< std::string >(param_nh.resolveName("interface"), "USB"),
        rp::param< std::string >(param_nh.resolveName("port"), "USB0")));
    if (device.isError()) {
      ROS_ERROR_STREAM(
          "EposActuatorLayer::init(): Failed to open an EPOS device: " << device.errorInfo());
      return false;
    }

    // configure the epos device
    const eclc::Result< void > result_setting(
        device->setProtocolStackSettingsSI(rp::param(param_nh.resolveName("baudrate"), 1000000),
                                           rp::param(param_nh.resolveName("timeout"), 0.5)));
    if (result_setting.isError()) {
      ROS_ERROR_STREAM(
          "EposActuatorLayer::init(): Failed to set protocol stack settings of an EPOS device: "
          << result_setting.errorInfo());
      return false;
    }

    // load actuator names from param "actuators"
    XmlRpc::XmlRpcValue ators_param;
    if (!param_nh.getParam("actuators", ators_param)) {
      ROS_ERROR_STREAM("EposActuatorLayer::init(): Failed to get param '"
                       << param_nh.resolveName("actuators") << "'");
      return false;
    }
    if (ators_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("EposActuatorLayer::init(): Param '" << param_nh.resolveName("actuators")
                                                            << "' must be a struct");
      return false;
    }

    // init actuators with param "actuators/<actuator_name>"
    // (could not use BOOST_FOREACH here to avoid a bug in the library in Kinetic)
    for (XmlRpc::XmlRpcValue::iterator ator_param = ators_param.begin();
         ator_param != ators_param.end(); ++ator_param) {
      EposActuatorPtr ator(new EposActuator());
      ros::NodeHandle ator_param_nh(param_nh, ros::names::append("actuators", ator_param->first));
      if (!ator->init(ator_param->first, *device, hw, ator_param_nh)) {
        return false;
      }
      ROS_INFO_STREAM("EposActuatorLayer::init(): Initialized the actuator '" << ator_param->first
                                                                              << "'");
      actuators_.push_back(ator);
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // ask to all actuators if controller switching is possible
    BOOST_FOREACH (const EposActuatorPtr &ator, actuators_) {
      if (!ator->prepareSwitch(start_list, stop_list)) {
        return false;
      }
    }
    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // notify controller switching to all actuators
    BOOST_FOREACH (const EposActuatorPtr &ator, actuators_) {
      ator->doSwitch(start_list, stop_list);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read from all actuators
    BOOST_FOREACH (const EposActuatorPtr &ator, actuators_) { ator->read(time, period); }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all actuators
    BOOST_FOREACH (const EposActuatorPtr &ator, actuators_) { ator->write(time, period); }
  }

private:
  // make an hardware interface registered. the interface must be in the static memory space
  // to allow access from outside of this plugin.
  template < typename Interface > static void makeRegistered(hi::RobotHW *const hw) {
    if (!hw->get< Interface >()) {
      static Interface iface;
      hw->registerInterface(&iface);
    }
  }

private:
  std::vector< EposActuatorPtr > actuators_;
};
} // namespace layered_hardware_epos

#endif