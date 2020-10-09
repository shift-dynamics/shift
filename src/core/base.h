#ifndef SHIFT_BASE_H_
#define SHIFT_BASE_H_

#include <any>
#include <map>
#include <string>
#include <vector>

static int id__ = 0;

namespace shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class Base {

public:
  Base(std::string name)
    : name_(name)
    , id_(id__++) {}
  virtual ~Base() {}

  const std::string& name() const;
  void setName(const std::string);
  int id() const;
  bool verify();

protected:
  std::string name_ = "";
  int id_;
};

} // namespace shift

#endif
