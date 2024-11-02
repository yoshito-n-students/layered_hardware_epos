# layered_hardware_epos
A ros2_control layer implementation for Maxon EPOS actuator drivers. See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware/tree/jazzy) to understand the layered scheme.

## Plugins: layered_hardware_epos_plugins
### layered_hardware_epos/EposActuatorLayer
* sends commands to Maxon EPOS actuator drivers within `write()` function
* fetches states of actuators within `read()` function
* switches actuators' operation modes within `perform_command_mode_swtich()` function when controllers using associated interfaces activate

#### Hardware parameters
___<layer_name>___ (yaml, required)
* map of parameter names and values for this layer

___<layer_name>.device___ (string, default: 'EPOS4')
* device type of EPOS driver like 'EPOS', 'EPOS2', or 'EPOS4'.

___<layer_name>.protocol_stack___ (string, default: 'MAXON SERIAL V2')
* protocol stack type of EPOS driver like 'MAXON SERIAL V2', or 'MAXON RS232'.

___<layer_name>.interface___ (string, default: 'USB')
* interface type of EPOS driver like 'USB', 'RS232', or 'CANopen'.

___<layer_name>.port___ (string, default: 'USB0')
* port name of EPOS driver

___<layer_name>.baudrate___ (int, default: 1000000)
* baudrate of communication to EPOS drivers

___<layer_name>.timeout___ (double, default: 0.5)
* timeout of communication to EPOS drivers in seconds

___<layer_name>.actuators___ (map, required)
* map of parameters for each actuator

___<layer_name>.actuators.<actuator_name>.id___ (int, required)
* id of the EPOS driver

___<layer_name>.actuators.<actuator_name>.count_per_revolution___ (int, required)
* encoder count per revolution of the actuator, for conversion between position units

___<layer_name>.actuators.<actuator_name>.torque_constant___ (double, required)
* torque constant for conversion between current and torque in N*m/A

___<layer_name>.actuators.<actuator_name>.operation_mode_map___ (map, required)
* map to actuator's operation mode names from associated interface names (typically joint interfaces)
* possible operation mode names are 'clear_falut', 'current', 'disable', 'position', 'profile_position', 'profile_velocity', 'reset' & 'velocity'

#### Example of parameter description
```xml
<param name="example_epos_actuator_layer">
    device: EPOS4
    protocol_stack: MAXON SERIAL V2
    interface: USB
    port: USB0
    baudrate: 1000000
    actuators:
        example_epos_1:
            id: 1
            count_per_revolution: 2048
            torque_constant: 2.41
            operation_mode_map:
                example_joint_1/position: profile_position
                ...
        example_epos_2:
            id: 2
            ...
</param>
```

## Example
see [examples](examples)