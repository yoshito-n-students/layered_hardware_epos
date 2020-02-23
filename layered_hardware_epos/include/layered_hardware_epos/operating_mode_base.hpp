#ifndef LAYERED_HARDWARE_EPOS_OPERATING_MODE_BASE_HPP
#define LAYERED_HARDWARE_EPOS_OPERATING_MODE_BASE_HPP

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include <layered_hardware_epos/epos_actuator_data.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for isnan()
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
  // TODO: implement common operations ??

  //
  // utility
  //

  static bool isNotNaN(const double a) { return !boost::math::isnan(a); }

  static bool areNotEqual(const double a, const double b) {
    // does !(|a - b| < EPS) instead of (|a - b| >= EPS) to return True when a and/or b is NaN
    return !(std::abs(a - b) < std::numeric_limits< double >::epsilon());
  }

protected:
  const std::string name_;
  const EposActuatorDataPtr data_;
};

typedef boost::shared_ptr< OperatingModeBase > OperatingModePtr;
typedef boost::shared_ptr< const OperatingModeBase > OperatingModeConstPtr;
} // namespace layered_hardware_epos

#endif