#ifndef LAYERED_HARDWARE_EPOS_CURRENT_MODE_HPP
#define LAYERED_HARDWARE_EPOS_CURRENT_MODE_HPP

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

class CurrentMode : public OperatingModeBase {
public:
  CurrentMode(const EposActuatorDataPtr &data) : OperatingModeBase("current", data) {}

  virtual void starting() {
    // switch to current mode
    data_->node.setEnableState();
    data_->node.activateCurrentMode();

    // set reasonable initial command
    data_->eff_cmd = 0.;
    prev_eff_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    try {
      data_->pos = *data_->node.getPosition(data_->count_per_revolution);
      data_->vel = *data_->node.getVelocity();
      data_->eff = *data_->node.getTorque(data_->torque_constant);
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("CurrentMode::read(): " << error.what());
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (boost::math::isnan(data_->eff_cmd) || data_->eff_cmd == prev_eff_cmd_) {
      return;
    }

    try {
      data_->node.setTorqueMust(data_->eff_cmd, data_->torque_constant);
      prev_eff_cmd_ = data_->eff_cmd;
    } catch (const eclc::Exception &error) {
      ROS_ERROR_STREAM("CurrentMode::write(): " << error.what());
    }
  }

  virtual void stopping() { data_->node.setDisableState(); }

private:
  double prev_eff_cmd_;
};
} // namespace layered_hardware_epos

#endif