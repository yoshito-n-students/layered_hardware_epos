#ifndef EPOS_COMMAND_LIBRARY_CPP_EXCEPTION_HPP
#define EPOS_COMMAND_LIBRARY_CPP_EXCEPTION_HPP

#include <stdexcept>
#include <string>

namespace epos_command_library_cpp {

// ======================
// common base exception

class Exception : public std::runtime_error {
public:
  Exception(const std::string &what_arg) : std::runtime_error(what_arg) {}
};

// ====================
// specific exceptions

class NoValueException : public Exception {
public:
  NoValueException(const std::string &what_arg) : Exception(what_arg) {}
};

class NoErrorCodeException : public Exception {
public:
  NoErrorCodeException(const std::string &what_arg) : Exception(what_arg) {}
};

class NoErrorInfoException : public Exception {
public:
  NoErrorInfoException(const std::string &what_arg) : Exception(what_arg) {}
};

} // namespace epos_command_library_cpp

#endif