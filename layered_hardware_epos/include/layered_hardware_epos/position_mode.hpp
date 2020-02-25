#ifndef LAYERED_HARDWARE_EPOS_POSITION_MODE_HPP
#define LAYERED_HARDWARE_EPOS_POSITION_MODE_HPP

#include <limits>

#include <epos_command_library_cpp/exception.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>
#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operating_mode_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_epos {

class PositionMode : public OperatingModeBase {
public:
  PositionMode(const EposActuatorDataPtr &data) : OperatingModeBase("position", data) {}

  virtual void starting() {
    try {
      // switch to position mode
      *data_->node.setEnableState();
      *data_->node.activatePositionMode();

      // set reasonable initial command
      data_->pos_cmd = *data_->node.getPosition(data_->count_per_revolution);
      prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();

      has_started_ = true;
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("PositionMode::starting(): " << getNodeDescription() << ": "
                                                    << error.what());
      has_started_ = false;
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      data_->pos = *data_->node.getPosition(data_->count_per_revolution);
      data_->vel = *data_->node.getVelocity();
      data_->eff = *data_->node.getTorque(data_->torque_constant);
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("PositionMode::read(): " << getNodeDescription() << ": " << error.what());
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (!has_started_) {
      return;
    }

    try {
      if (!boost::math::isnan(data_->pos_cmd) && data_->pos_cmd != prev_pos_cmd_) {
        *data_->node.setPositionMust(data_->pos_cmd, data_->count_per_revolution);
        prev_pos_cmd_ = data_->pos_cmd;
      }
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("PositionMode::write(): " << getNodeDescription() << ": " << error.what());
    }
  }

  virtual void stopping() {
    try {
      *data_->node.setDisableState();
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("PositionMode::stopping(): " << getNodeDescription() << ": "
                                                    << error.what());
    }
  }

private:
  bool has_started_;
  double prev_pos_cmd_;
};
} // namespace layered_hardware_epos

#endif