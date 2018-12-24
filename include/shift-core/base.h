#ifndef _SHIFT_BASE_H_
#define _SHIFT_BASE_H_

#include <string>
#include <vector>

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

  const std::string& getName();
  bool isActive();
  void setName(const std::string&);
  int getId();
  void setFullyQualifiedName(const std::vector<std::string>&);
  const std::vector<std::string>& getFullyQualifiedName();
  void setActive(bool);

protected:
  std::string name_;
  std::vector<std::string> fullyQualifiedName_;
  int id_;
  bool active_ = true;
};

} // namespace Shift

#endif
