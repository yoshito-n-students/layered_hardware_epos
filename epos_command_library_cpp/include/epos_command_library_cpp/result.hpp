#ifndef EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP
#define EPOS_COMMAND_LIBRARY_CPP_RESULT_HPP

#include <string>

#include <epos_command_library/Definitions.h>
#include <epos_command_library_cpp/exception.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

namespace epos_command_library_cpp {

// ==================================================
// base class wrapping epos command lib's error code

class ResultBase {
public:
  virtual ~ResultBase() {}

  virtual bool isError() const = 0;

  virtual bool isSuccess() const = 0;

  unsigned int errorCode() const {
    if (isSuccess()) {
      throw NoErrorCodeException("No error code");
    }
    return error_code_;
  }

  std::string errorInfo() const {
    char info[256];
    if (VCS_GetErrorInfo(errorCode(), info, 256) == 0) {
      throw NoErrorInfoException("No info for the error code '" +
                                 boost::lexical_cast< std::string >(error_code_) + "'");
    }
    return info;
  }

protected:
  // allow to construct only to child classes
  ResultBase() {}

protected:
  unsigned int error_code_;
};

// ==========================================
// utility class holding value or error code

template < typename T > class Result : public ResultBase {
public:
  typedef T Value;

  virtual ~Result() {}

  bool isError() const override { return value_ == boost::none; }

  bool isSuccess() const override { return value_ != boost::none; }

  Value &unwrap() {
    if (isError()) {
      throw NoValueException(errorInfo());
    }
    return *value_;
  }

  const Value &unwrap() const {
    if (isError()) {
      throw NoValueException(errorInfo());
    }
    return *value_;
  }

  Value &operator*() { return unwrap(); }

  const Value &operator*() const { return unwrap(); }

  Value *operator->() { return &unwrap(); }

  const Value *operator->() const { return &unwrap(); }

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
};

// void specialization
template <> class Result< void > : public ResultBase {
public:
  typedef void Value;

  virtual ~Result() {}

  bool isError() const override { return is_error_; }

  bool isSuccess() const override { return !is_error_; }

  void unwrap() const {
    if (isError()) {
      throw NoValueException(errorInfo());
    }
  }

  void operator*() const { unwrap(); }

  // ================
  // factory methods

  static Result< void > success() {
    Result< void > result;
    result.is_error_ = false;
    return result;
  }

  static Result< void > error(const unsigned int error_code) {
    Result< void > result;
    result.is_error_ = true;
    result.error_code_ = error_code;
    return result;
  }

private:
  Result() {}

private:
  bool is_error_;
};
} // namespace epos_command_library_cpp

#endif