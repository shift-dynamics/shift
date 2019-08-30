#ifndef SHIFT_BASE_H_
#define SHIFT_BASE_H_

#include <any>
#include <map>
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
  bool verify();

protected:
  std::string name_;
  std::vector<std::string> fully_qualified_name_;
  int id_;
  bool active_ = true;
  std::map<std::string, std::any> map_;
};

} // namespace Shift

#endif
