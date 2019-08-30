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
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }
  void setContext(Context* context) {
    context_ = context;
  }

  virtual void load();
  virtual void connect();

  static const std::string arrayKey() {
    return "geometry";
  }
  static const std::string defaultType() {
    return "";
  }

protected:
  Context* context_;
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
