#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_epos/epos_actuator_layer.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(layered_hardware_epos::EposActuatorLayer,
                       layered_hardware::LayerBase);