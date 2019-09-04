#ifndef SHIFT_GEOMETRY_BASE_H_
#define SHIFT_GEOMETRY_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

namespace Shift {

class Frame;
class Context;

namespace Geometry {

/*!
@brief Abstract Geometry base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name) {}
  Base()
    : Shift::Base("") {}
  virtual ~Base() {
    std::cout << "virtual ~Geometry::Base() override: " << name_ << std::endl;
  }

  const std::string& getType() {
    return type_;
  }

  virtual void load();
  virtual void connect();
  virtual void setActiveObjects(bool) override;

  static const std::string arrayKey() {
    return "geometry";
  }
  static const std::string defaultType() {
    return "";
  }

protected:
  virtual void addParameters() override;
  std::string type_; //!< descriptive name of Geometry type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Geometry

} // namespace Shift

#endif
