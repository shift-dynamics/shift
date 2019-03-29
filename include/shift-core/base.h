#ifndef SHIFT_BASE_H_
#define SHIFT_BASE_H_

#include <string>
#include <vector>

static int id__ = 0;

namespace Shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class Base {

public:
  Base(std::string name) {
    name_ = name;
    id_ = id__++;
  }
  virtual ~Base() {}

  const std::string& name() const;
  bool isActive() const;
  void setName(const std::string&);
  int id() const;
  void setFullyQualifiedName(const std::vector<std::string>&);
  const std::vector<std::string>& fullyQualifiedName() const;
  void setActive(bool);

protected:
  std::string name_;
  std::vector<std::string> fullyQualifiedName_;
  int id_;
  bool active_ = true;
};

} // namespace Shift

#endif
