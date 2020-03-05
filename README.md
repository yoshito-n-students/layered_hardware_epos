# layered_hardware_epos
A ros_control layer implementation for maxon EPOS motor drivers. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware) to understand the layered scheme.

## Plugins: layered_hardware_epos_plugins
### layered_hardware_epos/EposActuatorLayer
* implements state & command interfaces for maxon EPOS motor drivers
#### <u>Layer parameters (should be defined under ~<layer_name>)</u>
___device___ (string, default: 'EPOS4')
* device type of EPOS driver like 'EPOS', 'EPOS2', or 'EPOS4'.

___protocol_stack___ (string, default: 'MAXON SERIAL V2')
* protocol stack type of EPOS driver like 'MAXON SERIAL V2', or 'MAXON RS232'.

___interface___ (string, default: 'USB')
* interface type of EPOS driver like 'USB', 'RS232', or 'CANopen'.

___port___ (string, default: 'USB0')
* port name of EPOS driver

___baudrate___ (int, default: 1000000)
* baudrate of communication to EPOS drivers

___timeout___ (double, default: 0.5)
* timeout of communication to EPOS drivers in seconds

___actuators___ (struct, required)
* actuator parameters (see below)

#### <u>Actuator parameters (should be defined under ~<layer_name>/actuators/<actuator_name>)</u>
___id___ (int, required)
* id of the EPOS driver

___count_per_revolution___ (int, required)
* encoder count per revolution of the actuator, for conversion between position units

___torque_constant___ (double, required)
* torque constant for conversion between current and torque in N*m/A

___operation_mode_map___ (map<string, string>, required)
* map from ROS's controller names to EPOS driver's operation mode names
* possible operation mode names are 'clear_fault', 'current', 'disable', 'position', 'profile_position', 'profile_velocity', 'reset', & 'velocity'

#### <u>Example</u>
see [layered_hardware_epos/launch/single_epos4_example.launch](layered_hardware_epos/launch/single_epos4_example.launch)