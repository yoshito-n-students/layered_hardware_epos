#ifndef LAYERED_HARDWARE_EPOS_CLEAR_FAULT_MODE_HPP
#define LAYERED_HARDWARE_EPOS_CLEAR_FAULT_MODE_HPP

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operation_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_epos {

class ClearFaultMode : public OperationModeBase {
public:
  ClearFaultMode(const EposActuatorDataPtr &data) : OperationModeBase("clear_fault", data) {}

  virtual void starting() override {
    try {
      *data_->node.clearFault();
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ClearFaultMode::starting(): " << data_->nodeDescription() << ": "
                                                      << error.what());
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // nothing to do
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // nothing to do
  }

  virtual void stopping() override {
    // nothing to do
  }
};
} // namespace layered_hardware_epos

#endif