#include <iostream>
#include <string>
#include <vector>

#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/exception.hpp>
#include <epos_command_library_cpp/node.hpp>
#include <epos_command_library_cpp/selection.hpp>

#include <boost/foreach.hpp>
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
  bool do_list_rs232;
  unsigned int timeout_ms;
  unsigned short max_node_id;
};

bool parseConfigs(const int argc, const char *const argv[], Configs *const configs) {
  try {
    // define available options
    bpo::options_description options;
    bool do_show_help;
    options.add(
        boost::make_shared< bpo::option_description >("help", bpo::bool_switch(&do_show_help)));
    options.add(boost::make_shared< bpo::option_description >(
        "rs232", bpo::bool_switch(&configs->do_list_rs232)));
    options.add(boost::make_shared< bpo::option_description >(
        "timeout-ms", bpo::value(&configs->timeout_ms)->default_value(500)));
    options.add(boost::make_shared< bpo::option_description >(
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
  Path(const std::string &_device = "", const std::string &_protocol_stack = "",
       const std::string &_interface = "", const std::string &_port = "",
       const unsigned short _node_id = 0)
      : device(_device), protocol_stack(_protocol_stack), interface(_interface), port(_port),
        node_id(_node_id) {}

  std::string device;
  std::string protocol_stack;
  std::string interface;
  std::string port;
  unsigned short node_id;
};

void listNode(const eclc::Device &device, const Path &path, const Configs &configs,
              const std::size_t n_indent) {
  try {
    const eclc::Node node(device, path.node_id);

    const boost::uint64_t serial_number(*node.getSerialNumber());
    unsigned short hw_version, sw_version, app_num, app_version;
    *node.getVersion(&hw_version, &sw_version, &app_num, &app_version);

    const std::string indent(n_indent, '\t');
    std::cout << indent << "Node Id: " << std::dec << path.node_id << std::endl;
    std::cout << indent << "\tSerial number: 0x" << std::hex << serial_number << std::endl;
    std::cout << indent << "\tHardware version: 0x" << std::hex << hw_version << std::endl;
    std::cout << indent << "\tSoftware version: 0x" << std::hex << sw_version << std::endl;
    std::cout << indent << "\tApplication number: 0x" << std::hex << app_num << std::endl;
    std::cout << indent << "\tApplication version: 0x" << std::hex << app_version << std::endl;

    const unsigned short motor_type(*node.getMotorType());
    std::cout << indent << "\tMotor type: " << std::dec << motor_type << std::endl;
    if (motor_type == MT_DC_MOTOR) {
      unsigned short nominal_current, max_output_current, thermal_time_constant;
      *node.getDcMotorParameter(&nominal_current, &max_output_current, &thermal_time_constant);
      std::cout << indent << "\t\tNominal current: " << nominal_current << std::endl;
      std::cout << indent << "\t\tMax output current: " << max_output_current << std::endl;
      std::cout << indent << "\t\tThermal time constant: " << thermal_time_constant << std::endl;
    } else if (motor_type == MT_EC_BLOCK_COMMUTATED_MOTOR ||
               motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) {
      unsigned short nominal_current, max_output_current, thermal_time_constant;
      unsigned char n_pole_pairs;
      *node.getEcMotorParameter(&nominal_current, &max_output_current, &thermal_time_constant,
                                &n_pole_pairs);
      std::cout << indent << "\t\tNominal current: " << nominal_current << std::endl;
      std::cout << indent << "\t\tMax output current: " << max_output_current << std::endl;
      std::cout << indent << "\t\tThermal time constant: " << thermal_time_constant << std::endl;
      std::cout << indent << "\t\tPole pairs: " << static_cast< int >(n_pole_pairs) << std::endl;
    } else {
      std::cerr << indent << "\t\tUnknown motor type!!!" << std::endl;
    }

    const unsigned short sensor_type(*node.getSensorType());
    std::cout << indent << "\tSensor type: " << std::dec << sensor_type << std::endl;
    if (sensor_type == ST_INC_ENCODER_2CHANNEL || sensor_type == ST_INC_ENCODER_3CHANNEL ||
        sensor_type == ST_INC_ENCODER2_2CHANNEL || sensor_type == ST_INC_ENCODER2_3CHANNEL) {
      unsigned int encoder_resolution;
      bool inverted_polarity;
      *node.getIncEncoderParameter(&encoder_resolution, &inverted_polarity);
      std::cout << indent << "\t\tEncoder resolution: " << encoder_resolution << std::endl;
      std::cout << indent << "\t\tInverted polarity: " << (inverted_polarity ? "Yes" : "No")
                << std::endl;
    } else {
      std::cerr << indent << "\t\tUnknown sensor type!!!" << std::endl;
    }
  } catch (const eclc::Exception &error) {
    // catching an error indicates the node does not exist. nothing to show.
  }
}

void listNodesOnPort(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.port << std::endl;

  // show available baudrates
  const std::vector< unsigned int > baudrates(
      *eclc::getBaudrateList(path.device, path.protocol_stack, path.interface, path.port));
  std::cout << indent << "\tBaudrates:" << std::endl;
  BOOST_FOREACH (const unsigned int baudrate, baudrates) {
    std::cout << indent << "\t\t" << std::dec << baudrate << std::endl;
  }

  // open the device on the given path
  eclc::Device device(
      *eclc::Device::open(path.device, path.protocol_stack, path.interface, path.port));
  *device.setTimeout(configs.timeout_ms);

  // list nodes on the device
  std::cout << indent << "\tNodes (up to node id '" << configs.max_node_id << "'):" << std::endl;
  for (unsigned short node_id = 1; node_id <= configs.max_node_id; ++node_id) {
    listNode(device, Path(path.device, path.protocol_stack, path.interface, path.port, node_id),
             configs, n_indent + 2);
  }
}

void listNodesOnInterface(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  if (path.interface != "RS232" || configs.do_list_rs232) {
    std::cout << indent << path.interface << std::endl;
  } else {
    std::cout << indent << "RS232 (use '--rs232' option to list)" << std::endl;
    return;
  }

  // list nodes on the interface
  const std::vector< std::string > ports(
      *eclc::getPortNameList(path.device, path.protocol_stack, path.interface));
  BOOST_FOREACH (const std::string &port, ports) {
    listNodesOnPort(Path(path.device, path.protocol_stack, path.interface, port), configs,
                    n_indent + 1);
  }
}

void listNodesOnProtocolStack(const Path &path, const Configs &configs,
                              const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.protocol_stack << std::endl;

  const std::vector< std::string > interfaces(
      *eclc::getInterfaceNameList(path.device, path.protocol_stack));
  BOOST_FOREACH (const std::string &interface, interfaces) {
    listNodesOnInterface(Path(path.device, path.protocol_stack, interface), configs, n_indent + 1);
  }
}

void listNodesOnDevice(const Path &path, const Configs &configs, const std::size_t n_indent) {
  const std::string indent(n_indent, '\t');
  std::cout << indent << path.device << std::endl;

  const std::vector< std::string > protocol_stacks(*eclc::getProtocolStackNameList(path.device));
  BOOST_FOREACH (const std::string &protocol_stack, protocol_stacks) {
    listNodesOnProtocolStack(Path(path.device, protocol_stack), configs, n_indent + 1);
  }
}

void listAllNodes(const Configs &configs, const std::size_t n_indent = 0) {
  try {
    const std::vector< std::string > devices(*eclc::getDeviceNameList());
    BOOST_FOREACH (const std::string &device, devices) {
      listNodesOnDevice(Path(device), configs, n_indent);
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
    listAllNodes(configs);
  }
  return 0;
}