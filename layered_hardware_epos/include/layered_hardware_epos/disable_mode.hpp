#ifndef LAYERED_HARDWARE_EPOS_DISABLE_MODE_HPP
#define LAYERED_HARDWARE_EPOS_DISABLE_MODE_HPP

#include <limits>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operation_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_epos {

// TODO: consider naming; passive mode? torque disable mode? monitor mode?
class DisableMode : public OperationModeBase {
public:
  DisableMode(const EposActuatorDataPtr &data) : OperationModeBase("disable", data) {}

  virtual void starting() {
    try {
      // switch to disable state
      *data_->node.setDisableState();

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("DisableMode::starting(): " << data_->nodeDescription() << ": "
                                                   << error.what());
      has_started_ = false;
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      data_->pos = *data_->node.getPositionSI(data_->count_per_revolution);
      data_->vel = *data_->node.getVelocitySI();
      data_->eff = *data_->node.getTorqueSI(data_->torque_constant);
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("DisableMode::read(): " << data_->nodeDescription() << ": " << error.what());
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // nothing to do
  }

  virtual void stopping() {
    // nothing to do
  }

private:
  bool has_started_;
};
} // namespace layered_hardware_epos

#endif