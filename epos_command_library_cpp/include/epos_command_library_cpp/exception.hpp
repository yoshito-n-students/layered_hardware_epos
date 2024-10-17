#ifndef EPOS_COMMAND_LIBRARY_CPP_EXCEPTION_HPP
#define EPOS_COMMAND_LIBRARY_CPP_EXCEPTION_HPP

#include <stdexcept>

namespace epos_command_library_cpp {

// ======================
// common base exception

class Exception : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// ====================
// specific exceptions

class NoValueException : public Exception {
public:
  using Exception::Exception;
};

class NoErrorCodeException : public Exception {
public:
  using Exception::Exception;
};

class NoErrorInfoException : public Exception {
public:
  using Exception::Exception;
};

} // namespace epos_command_library_cpp

#endif