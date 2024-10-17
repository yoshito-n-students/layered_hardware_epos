#include <iostream>
#include <string>

#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/exception.hpp>
#include <epos_command_library_cpp/node.hpp>
#include <epos_command_library_cpp/selection.hpp>

#include <boost/make_shared.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace bpo = boost::program_options;
namespace eclc = epos_command_library_cpp;

// ====================
// command line parser

struct Configs {
  bool do_list_rs232 = false;
  unsigned int timeout_ms = 500;
  unsigned short max_node_id = 8;
};

bool parseConfigs(const int argc, const char *const argv[], Configs *const configs) {
  try {
    // define available options
    bpo::options_description options;
    bool do_show_help;
    options.add(
        boost::make_shared<bpo::option_description>("help", bpo::bool_switch(&do_show_help)));
    options.add(boost::make_shared<bpo::option_description>(
        "rs232", bpo::bool_switch(&configs->do_list_rs232)));
    options.add(boost::make_shared<bpo::option_description>(
        "timeout-ms", bpo::value(&configs->timeout_ms)->default_value(500)));
    options.add(boost::make_shared<bpo::option_description>(
        "max-node-id", bpo::value(&configs->max_node_id)->default_value(8)));
    // parse the command line
    bpo::variables_map args;
    bpo::store(bpo::parse_command_line(argc, argv, options), args);
    bpo::notify(args);
    // show help if requested
    if (do_show_help) {
      std::cout << "Available options:\n" << options << std::endl;
      return false;
    }
  } catch (const bpo::error &error) {
    std::cerr << "Error caught on parsing configs: " << error.what() << std::endl;
    return false;
  }
  return true;
}

// ===============================
// helper functions to list nodes

struct Path {
  std::string device = "";
  std::string protocol_stack = "";
  std::string interface = "";
  std::string port = "";
  unsigned short node_id = 0;
};

void list_node(const eclc::Device &device, const Path &path, const Configs & /*configs*/,
               const std::size_t n_indent) {
  try {
    const std::string indent(n_indent, '\t');
    const eclc::Node node(device, path.node_id);

    // serial number
    const auto serial_number = *node.get_serial_number();
    std::cout << indent << "Node Id: " << std::dec << path.node_id << std::endl;
    std::cout << indent << "\tSerial number: 0x" << std::hex << serial_number << std::endl;

    // versions
    unsigned short hw_version, sw_version, app_num, app_version;
    *node.get_version(&hw_version, &sw_version, &app_num, &app_version);
    std::cout << indent << "\tHardware version: 0x" << std::hex << hw_version << std::endl;
    std::cout << indent << "\tSoftware version: 0x" << std::hex << sw_version << std::endl;
    std::cout << indent << "\tApplication number: 0x" << std::hex << app_num << std::endl;
    std::cout << indent << "\tApplication version: 0x" << std::hex << app_version << std::endl;

    // motor parameters
    const auto motor_type = *node.get_motor_type();
    if (motor_type == MT_DC_MOTOR) {
      std::cout << indent << "\tMotor type: DC (" << std::dec << motor_type << ")" << std::endl;
    } else if (motor_type == MT_EC_BLOCK_COMMUTATED_MOTOR ||
               motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) {
      std::cout << indent << "\tMotor type: EC (" << std::dec << motor_type << ")" << std::endl;
    } else {
      std::cout << indent << "\tMotor type: Unknown (" << std::dec << motor_type << ")"
                << std::endl;
    }

    // sensor parameters
    const auto sensor_type = *node.get_sensor_type();
    if (sensor_type == ST_INC_ENCODER_2CHANNEL || sensor_type == ST_INC_ENCODER_3CHANNEL ||
        sensor_type == ST_INC_ENCODER2_2CHANNEL || sensor_type == ST_INC_ENCODER2_3CHANNEL ||
        sensor_type == ST_ANALOG_INC_ENCODER_2CHANNEL ||
        sensor_type == ST_ANALOG_INC_ENCODER_3CHANNEL) {
      std::cout << indent << "\tSensor type: Incremental encoder (" << std::dec << sensor_type
                << ")" << std::endl;
    } else if (sensor_type == ST_HALL_SENSORS) {
      std::cout << indent << "\tSensor type: Hall sensors (" << std::dec << sensor_type << ")"
                << std::endl;
    } else if (sensor_type == ST_SSI_ABS_ENCODER_BINARY || sensor_type == ST_SSI_ABS_ENCODER_GREY) {
      std::cout << indent << "\tSensor type: Asbolute encoder (" << std::dec << sensor_type << ")"
                << std::endl;
    } else {
      std::cout << indent << "\tSensor type: Unknown (" << std::dec << sensor_type << ")"
                << std::endl;
    }

    // error history in node
    const auto dev_errors = *node.get_device_error_codes();
    std::cout << indent << "\tDevice errors: " << dev_errors.size() << std::endl;
    for (std::size_t i = 0; i < dev_errors.size(); ++i) {
      std::cout << indent << "\t\t[" << i + 1 << "]: 0x" << std::hex << dev_errors[i] << std::endl;
    }
  } catch (const eclc::Exception &error) {
    // catching an error indicates the node does not exist. nothing to show.
  }
}

void list_nodes_on_port(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.port << std::endl;

  // show available baudrates
  const auto baudrates =
      *eclc::get_baudrate_list(path.device, path.protocol_stack, path.interface, path.port);
  std::cout << indent << "\tBaudrates:" << std::endl;
  for (const unsigned int baudrate : baudrates) {
    std::cout << indent << "\t\t" << std::dec << baudrate << std::endl;
  }

  // open the device on the given path
  auto device = *eclc::Device::open(path.device, path.protocol_stack, path.interface, path.port);
  *device.set_timeout(configs.timeout_ms);

  // list nodes on the device
  std::cout << indent << "\tNodes (up to node id '" << configs.max_node_id << "'):" << std::endl;
  for (unsigned short node_id = 1; node_id <= configs.max_node_id; ++node_id) {
    list_node(device, Path{path.device, path.protocol_stack, path.interface, path.port, node_id},
              configs, n_indent + 2);
  }
}

void list_nodes_on_interface(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  if (path.interface != "RS232" || configs.do_list_rs232) {
    std::cout << indent << path.interface << std::endl;
  } else {
    std::cout << indent << "RS232 (use '--rs232' option to list)" << std::endl;
    return;
  }

  // list nodes on the interface
  *eclc::reset_port_name_selection(path.device, path.protocol_stack, path.interface);
  const auto ports =
      *eclc::get_port_name_selection(path.device, path.protocol_stack, path.interface);
  for (const auto &port : ports) {
    list_nodes_on_port(Path{path.device, path.protocol_stack, path.interface, port}, configs,
                       n_indent + 1);
  }
}

void list_nodes_on_protocol_stack(const Path &path, const Configs &configs,
                                  const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.protocol_stack << std::endl;

  const auto interfaces = *eclc::get_interface_name_selection(path.device, path.protocol_stack);
  for (const auto &interface : interfaces) {
    list_nodes_on_interface(Path{path.device, path.protocol_stack, interface}, configs,
                            n_indent + 1);
  }
}

void list_nodes_on_device(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.device << std::endl;

  const auto protocol_stacks = *eclc::get_protocol_stack_name_selection(path.device);
  for (const auto &protocol_stack : protocol_stacks) {
    list_nodes_on_protocol_stack(Path{path.device, protocol_stack}, configs, n_indent + 1);
  }
}

void list_all_nodes(const Configs &configs, const std::size_t n_indent = 0) {
  try {
    const auto devices = *eclc::get_device_name_selection();
    for (const auto &device : devices) {
      list_nodes_on_device(Path{device}, configs, n_indent);
    }
  } catch (const eclc::Exception &error) {
    std::cerr << "Error caught on listing nodes: " << error.what() << std::endl;
  }
}

// =====
// main

int main(int argc, char *argv[]) {
  Configs configs;
  if (parseConfigs(argc, argv, &configs)) {
    list_all_nodes(configs);
  }

  return 0;
}