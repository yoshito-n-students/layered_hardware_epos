#include <iostream>
#include <string>
#include <vector>

#include <epos_command_library_cpp/device.hpp>
#include <epos_command_library_cpp/node.hpp>
#include <epos_command_library_cpp/result.hpp>
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

int main(int argc, char *argv[]) {
  //
  bool list_rs232;
  unsigned int timeout_ms;
  unsigned short max_node_id;
  try {
    // define available options
    bpo::options_description options;
    bool show_help;
    options.add(
        boost::make_shared< bpo::option_description >("help", bpo::bool_switch(&show_help)));
    options.add(
        boost::make_shared< bpo::option_description >("rs232", bpo::bool_switch(&list_rs232)));
    options.add(boost::make_shared< bpo::option_description >(
        "timeout-ms", bpo::value(&timeout_ms)->default_value(500)));
    options.add(boost::make_shared< bpo::option_description >(
        "max-node-id", bpo::value(&max_node_id)->default_value(8)));
    // parse the command line
    bpo::variables_map args;
    bpo::store(bpo::parse_command_line(argc, argv, options), args);
    bpo::notify(args);
    // show help if requested
    if (show_help) {
      std::cout << "Available options:\n" << options << std::endl;
      return 0;
    }
  } catch (const bpo::error &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }

  std::cout << "Listing Nodes:" << std::endl;

  const std::vector< std::string > device_names(*eclc::getDeviceNameList());
  BOOST_FOREACH (const std::string &device_name, device_names) {
    std::cout << device_name << std::endl;

    const std::vector< std::string > protocol_stack_names(
        *eclc::getProtocolStackNameList(device_name));
    BOOST_FOREACH (const std::string &protocol_stack_name, protocol_stack_names) {
      std::cout << "\t" << protocol_stack_name << std::endl;

      const std::vector< std::string > interface_names(
          *eclc::getInterfaceNameList(device_name, protocol_stack_name));
      BOOST_FOREACH (const std::string &interface_name, interface_names) {
        std::cout << "\t\t" << interface_name << std::endl;
        if (!list_rs232 && interface_name == "RS232") {
          std::cout << "\t\t\tSkipping RS232" << std::endl;
          continue;
        }

        const std::vector< std::string > port_names(
            *eclc::getPortNameList(device_name, protocol_stack_name, interface_name));
        BOOST_FOREACH (const std::string &port_name, port_names) {
          std::cout << "\t\t\t" << port_name << std::endl;

          const std::vector< unsigned int > baudrates(
              *eclc::getBaudrateList(device_name, protocol_stack_name, interface_name, port_name));
          std::cout << "\t\t\t\tBaudrates:" << std::endl;
          BOOST_FOREACH (unsigned int baudrate, baudrates) {
            std::cout << "\t\t\t\t\t" << std::dec << baudrate << std::endl;
          }

          eclc::Device device(
              *eclc::Device::open(device_name, protocol_stack_name, interface_name, port_name));
          *device.setTimeout(timeout_ms);

          std::cout << "\t\t\t\tNodes (up to Node Id " << max_node_id << "):" << std::endl;
          for (std::size_t node_id = 1; node_id <= max_node_id; ++node_id) {
            const eclc::Node node(device, node_id);
            try {
              boost::uint64_t serial_number(*node.getSerialNumber());
              const unsigned short hw_version(*node.getHardwareVersion()),
                  sw_version(*node.getSoftwareVersion());

              std::cout << "\t\t\t\t\tNode Id: " << std::dec << node_id << std::endl;
              std::cout << "\t\t\t\t\t\tSerial Number: 0x" << std::hex << serial_number
                        << std::endl;
              std::cout << "\t\t\t\t\t\tHardware Version: 0x" << std::hex << hw_version
                        << std::endl;
              std::cout << "\t\t\t\t\t\tSoftware Version: 0x" << std::hex << sw_version
                        << std::endl;
            } catch (const std::runtime_error &error) {
              continue;
            }
          }
        }
      }
    }
  }

  return 0;
}