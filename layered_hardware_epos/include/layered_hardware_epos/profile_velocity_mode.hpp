#ifndef LAYERED_HARDWARE_EPOS_PROFILE_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_EPOS_PROFILE_VELOCITY_MODE_HPP

#include <limits>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operation_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_epos {

class ProfileVelocityMode : public OperationModeBase {
public:
  ProfileVelocityMode(const EposActuatorDataPtr &data)
      : OperationModeBase("profile_velocity", data) {}

  virtual void starting() {
    try {
      // switch to velocity mode
      *data_->node.setEnableState();
      *data_->node.activateProfileVelocityMode();

      // set reasonable initial command
      data_->vel_cmd = 0.;
      prev_vel_cmd_ = std::numeric_limits< double >::quiet_NaN();

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfileVelocityMode::starting(): " << data_->nodeDescription() << ": "
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
      ROS_ERROR_STREAM("ProfileVelocityMode::read(): " << data_->nodeDescription() << ": "
                                                       << error.what());
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      if (!boost::math::isnan(data_->vel_cmd) && data_->vel_cmd != prev_vel_cmd_) {
        *data_->node.moveWithVelocitySI(data_->vel_cmd);
        prev_vel_cmd_ = data_->vel_cmd;
      }
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfileVelocityMode::write(): " << data_->nodeDescription() << ": "
                                                        << error.what());
    }
  }

  virtual void stopping() {
    try {
      *data_->node.setDisableState();
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfileVelocityMode::stopping(): " << data_->nodeDescription() << ": "
                                                           << error.what());
    }
  }

private:
  bool has_started_;
  double prev_vel_cmd_;
};
} // namespace layered_hardware_epos

#endif