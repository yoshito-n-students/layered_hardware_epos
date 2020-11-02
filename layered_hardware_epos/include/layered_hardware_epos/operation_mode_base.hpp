#ifndef LAYERED_HARDWARE_EPOS_OPERATION_MODE_BASE_HPP
#define LAYERED_HARDWARE_EPOS_OPERATION_MODE_BASE_HPP

#include <string>

#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace layered_hardware_epos {

class OperationModeBase {
public:
  OperationModeBase(const std::string &name, const EposActuatorDataPtr &data)
      : name_(name), data_(data) {}

  virtual ~OperationModeBase() {}

  std::string getName() const { return name_; }

  // TODO: retrun bool to inform result of mode switching to the upper class
  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  const std::string name_;
  const EposActuatorDataPtr data_;
};

typedef std::shared_ptr< OperationModeBase > OperationModePtr;
typedef std::shared_ptr< const OperationModeBase > OperationModeConstPtr;
} // namespace layered_hardware_epos

#endif