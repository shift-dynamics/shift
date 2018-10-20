#ifndef _SHIFT_BASE_H_
#define _SHIFT_BASE_H_

#include <string>

static int id = 0;

namespace Shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class Base {

public:
  Base(std::string name) {
    name_ = name;
    id_ = id++;
  }
  ~Base() {}

  std::string getName();
  void setName(std::string);
  int getId();

private:
  std::string name_;
  int id_;
};

} // namespace Shift

#endif
