#ifndef LAYERED_HARDWARE_EPOS_POSITION_MODE_HPP
#define LAYERED_HARDWARE_EPOS_POSITION_MODE_HPP

#include <limits>

#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <layered_hardware_epos/operating_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_epos {

class PositionMode : public OperatingModeBase {
public:
  PositionMode(const EposActuatorDataPtr &data) : OperatingModeBase("position", data) {}

  virtual void starting() {
    // switch to velocity mode
    data_->node.setEnableState();
    data_->node.activatePositionMode();

    // set reasonable initial command
    // data_->pos_cmd = data_->node.getPosition();
    prev_pos_cmd_ = std::numeric_limits< double >::quiet_NaN();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    /*
    TODO: read actuator states
    data_->pos = data_->node.getPosition();
    data_->vel = data_->node.getVelocity();
    data_->eff = data_->node.getEffort();
    */
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    if (isNotNaN(data_->pos_cmd) && areNotEqual(data_->pos_cmd, prev_pos_cmd_)) {
      // data_->node.setPositionMust();
      prev_pos_cmd_ = data_->pos_cmd;
    }
  }

  virtual void stopping() { data_->node.setDisableState(); }

private:
  double prev_pos_cmd_;
};
} // namespace layered_hardware_epos

#endif