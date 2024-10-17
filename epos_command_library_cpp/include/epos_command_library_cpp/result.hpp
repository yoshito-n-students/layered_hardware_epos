#ifndef EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP
#define EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP

#include <optional>
#include <sstream>
#include <string>
#include <variant>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/exception.hpp>

namespace epos_command_library_cpp {

// ==========================================
// utility class holding value or error code

template <typename T> class Result {
public:
  using Value = T;

  // ==========
  // observers

  bool is_success() const { return error_or_value_.index() == 2; }

  bool is_error() const { return error_or_value_.index() == 1; }

  // ===================
  // accessors to value

  Value &value() {
    if (is_error()) {
      throw NoValueException(error_info());
    }
    return std::get<1>(error_or_value_);
  }

  const Value &value() const {
    if (is_error()) {
      throw NoValueException(error_info());
    }
    return std::get<1>(error_or_value_);
  }

  Value &operator*() { return value(); }

  const Value &operator*() const { return value(); }

  Value *operator->() { return &value(); }

  const Value *operator->() const { return &value(); }

  // ========================
  // accessors to error code

  unsigned int error_code() const {
    if (is_success()) {
      throw NoErrorCodeException("No error code");
    }
    return std::get<0>(error_or_value_);
  }

  std::string error_info() const {
    const unsigned int code = error_code();
    char info[256];
    if (VCS_GetErrorInfo(code, info, 256) == 0) {
      std::ostringstream msg;
      msg << "No info for the error code (" << code << ")";
      throw NoErrorInfoException(msg.str());
    }
    return info;
  }

  // ========================================================================
  // factory methods (only way to create instance as constructor is private)

  static Result<T> success(const T &value) {
    Result<T> result;
    result.error_or_value_.template emplace<1>(value);
    return result;
  }

  static Result<T> success(T &&value) {
    Result<T> result;
    result.error_or_value_.template emplace<1>(value);
    return result;
  }

  static Result<T> error(const unsigned int error_code) {
    Result<T> result;
    result.error_or_value_.template emplace<0>(error_code);
    return result;
  }

private:
  Result() = default;

private:
  std::variant<unsigned int, Value> error_or_value_;
};

// ============================
// specialization for T = void

template <> class Result<void> {
public:
  // ==========
  // observers

  bool is_success() const { return !error_code_.has_value(); }

  bool is_error() const { return error_code_.has_value(); }

  // ==========
  // unwrapper

  void operator*() const {
    if (is_error()) {
      throw NoValueException(error_info());
    }
  }

  // ========================
  // accessors to error code

  unsigned int error_code() const {
    if (is_success()) {
      throw NoErrorCodeException("No error code");
    }
    return error_code_.value();
  }

  std::string error_info() const {
    const unsigned int code = error_code();
    char info[256];
    if (VCS_GetErrorInfo(code, info, 256) == 0) {
      std::ostringstream msg;
      msg << "No info for the error code (" << code << ")";
      throw NoErrorInfoException(msg.str());
    }
    return info;
  }

  // ========================================================================
  // factory methods (only way to create instance as constructor is private)

  static Result<void> success() {
    Result<void> result;
    result.error_code_ = std::nullopt;
    return result;
  }

  static Result<void> error(const unsigned int error_code) {
    Result<void> result;
    result.error_code_.emplace(error_code);
    return result;
  }

private:
  Result() = default;

private:
  std::optional<unsigned int> error_code_;
};
} // namespace epos_command_library_cpp

#endif