#ifndef SHIFT_BASE_H_
#define SHIFT_BASE_H_

#include <any>
#include <map>
#include <string>
#include <vector>

static int id__ = 0;

namespace shift {
namespace core {

/*!
@brief Abstract base class for Shift C++ objects
*/
class Base {

public:
  Base(std::string name)
    : name_(name)
    , id_(id__++) {}
  virtual ~Base() {}

  const std::string& get_name() const;
  void set_name(const std::string);
  int get_id() const;
  bool verify();

protected:
  std::string name_ = "";
  int id_;
};

}
} // namespace shift

#endif
