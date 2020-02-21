#ifndef EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP
#define EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP

#include <stdexcept>
#include <string>

#include <epos_command_library/Definitions.h>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

namespace epos_command_library_cpp {

template < typename T > class Result {
public:
  virtual ~Result() {}

  bool isError() const { return value_ == boost::none; }

  bool isSuccess() const { return value_ != boost::none; }

  unsigned int errorCode() const {
    if (isSuccess()) {
      throw std::runtime_error("No error code");
    }
    return error_code_;
  }

  std::string errorInfo() const {
    char info[255];
    if (VCS_GetErrorInfo(errorCode(), info, 255) == 0) {
      throw std::runtime_error("VCS_GetErrorInfo() failed with error code '" +
                               boost::lexical_cast< std::string >(error_code_) + "'");
    }
    return info;
  }

  const T &unwrap() const {
    if (isError()) {
      throw std::runtime_error(errorInfo());
    }
    return *value_;
  }

  const T &operator*() const { return unwrap(); }

  // ================
  // factory methods

  static Result< T > success(const T &value) {
    Result< T > result;
    result.value_ = value;
    return result;
  }

  static Result< T > error(const unsigned int error_code) {
    Result< T > result;
    result.error_code_ = error_code;
    return result;
  }

private:
  Result() {}

private:
  boost::optional< T > value_;
  unsigned int error_code_;
};
} // namespace epos_command_library_cpp

#endif