#ifndef LAYERED_HARDWARE_EPOS_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_EPOS_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LHE_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware_epos"), __VA_ARGS__)
#define LHE_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware_epos"), __VA_ARGS__)
#define LHE_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware_epos"), __VA_ARGS__)
#define LHE_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware_epos"), __VA_ARGS__)
#define LHE_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware_epos"), __VA_ARGS__)

#endif