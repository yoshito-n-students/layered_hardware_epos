#ifndef LAYERED_HARDWARE_EPOS_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_EPOS_LOGGING_UTILS_HPP

#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware_epos/common_namespaces.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace layered_hardware_epos {

// returns reference to the common logger without construction overhead
static inline rclcpp::Logger &get_lhe_logger() {
  static rclcpp::Logger logger = rclcpp::get_logger("layered_hardware_epos");
  return logger;
}

// logging functions which supports cpp-string arguments

template <typename... Args> static inline void lhe_debug(const char *format, Args &&...args) {
  RCLCPP_DEBUG(get_lhe_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhe_info(const char *format, Args &&...args) {
  RCLCPP_INFO(get_lhe_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhe_warn(const char *format, Args &&...args) {
  RCLCPP_WARN(get_lhe_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhe_error(const char *format, Args &&...args) {
  RCLCPP_ERROR(get_lhe_logger(), format, lh::to_format_arg(args)...);
}

template <typename... Args> static inline void lhe_fatal(const char *format, Args &&...args) {
  RCLCPP_FATAL(get_lhe_logger(), format, lh::to_format_arg(args)...);
}

} // namespace layered_hardware_epos

#endif