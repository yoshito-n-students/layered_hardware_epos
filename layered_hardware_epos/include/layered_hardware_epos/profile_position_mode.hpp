#ifndef LAYERED_HARDWARE_EPOS_PROFILE_POSITION_MODE_HPP
#define LAYERED_HARDWARE_EPOS_PROFILE_POSITION_MODE_HPP

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

class ProfilePositionMode : public OperationModeBase {
public:
  ProfilePositionMode(const EposActuatorDataPtr &data)
      : OperationModeBase("profile_position", data) {}

  virtual void starting() {
    try {
      // switch to position mode
      *data_->node.setEnableState();
      *data_->node.activateProfilePositionMode();

      // initialize position command using present position
      data_->pos_cmd = *data_->node.getPosition(data_->count_per_revolution);
      prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();

      // initialize profile velocity command using present value
      *data_->node.getPositionProfile(&data_->vel_cmd, &prof_acc_, &prof_dec_);
      prev_vel_cmd_ = data_->vel_cmd;

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfilePositionMode::starting(): " << error.what());
      has_started_ = false;
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      // read actuator states
      data_->pos = *data_->node.getPosition(data_->count_per_revolution);
      data_->vel = *data_->node.getVelocity();
      data_->eff = *data_->node.getTorque(data_->torque_constant);
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfilePositionMode::read(): " << data_->nodeDescription() << ": "
                                                       << error.what());
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      // write profile velocity if command has been updated to be positive
      if (data_->vel_cmd > 0. && data_->vel_cmd != prev_vel_cmd_) {
        *data_->node.setPositionProfile(data_->vel_cmd, prof_acc_, prof_dec_);
        prev_vel_cmd_ = data_->vel_cmd;
      }

      // write position command if updated
      if (!boost::math::isnan(data_->pos_cmd) && data_->pos_cmd != prev_pos_cmd_) {
        *data_->node.moveToPosition(data_->pos_cmd, data_->count_per_revolution,
                                    /* absolute = */ true, /* immediately = */ true);
        prev_pos_cmd_ = data_->pos_cmd;
      }
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfilePositionMode::write(): " << data_->nodeDescription() << ": "
                                                        << error.what());
    }
  }

  virtual void stopping() {
    try {
      *data_->node.setDisableState();
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("ProfilePositionMode::stopping(): " << data_->nodeDescription() << ": "
                                                           << error.what());
    }
  }

private:
  bool has_started_;
  double prev_pos_cmd_, prev_vel_cmd_, prof_acc_, prof_dec_;
};
} // namespace layered_hardware_epos

#endif