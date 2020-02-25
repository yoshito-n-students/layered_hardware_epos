#ifndef LAYERED_HARDWARE_EPOS_RESET_MODE_HPP
#define LAYERED_HARDWARE_EPOS_RESET_MODE_HPP

#include <limits>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operating_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_epos {

class ResetMode : public OperatingModeBase {
public:
  ResetMode(const EposActuatorDataPtr &data) : OperatingModeBase("reset", data) {}

  virtual void starting() {
    try {
      *data_->node.resetDevice();
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("DisableMode::starting(): " << data_->nodeDescription() << ": "
                                                   << error.what());
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void stopping() {
    // nothing to do
  }
};
} // namespace layered_hardware_epos

#endif