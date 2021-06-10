#ifndef SHIFT_CORE_EXCEPTION_
#define SHIFT_CORE_EXCEPTION_
#include <string>

namespace shift {
namespace core {

class Exception : public std::exception {

private:
  std::string message_;
  std::string backtrace_;

  static const std::string _backtrace(int skip=1);

public:
  explicit Exception(const std::string& message);
  explicit Exception(const std::string& message, const char* file, int line);
  
  const char* what() const noexcept override {
    return message_.c_str();
  }
  
  inline const std::string backtrace() const {
    return backtrace_;
  }
};


} // namespace core
} // namespace shift

#endif
