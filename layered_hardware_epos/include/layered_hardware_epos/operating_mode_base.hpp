#ifndef LAYERED_HARDWARE_EPOS_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_EPOS_OPERATING_MODE_BASE_HPP

#include <string>

#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace layered_hardware_epos {

class OperatingModeBase {
public:
  OperatingModeBase(const std::string &name, const EposActuatorDataPtr &data)
      : name_(name), data_(data) {}

  virtual ~OperatingModeBase() {}

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

typedef boost::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef boost::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;
} // namespace layered_hardware_epos

#endif