#ifndef SHIFT_LIGHT_BASE_H_
#define SHIFT_LIGHT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;
class Context;

namespace Light {

/*!
@brief Abstract Shift::Light base class
*/
class Base
  : public Shift::Base
  , public Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name) {}
  Base()
    : Shift::Base("") {}
  virtual ~Base() override {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "lights";
  }
  static const std::string defaultType() {
    return "";
  }
  virtual void load();
  virtual void connect();
  void setActiveObjects(bool parent_is_active) override;

protected:
  virtual void addParameters() override;
  std::vector<std::string> parent_;
  std::string type_; //!< descriptive name of Light type
  ksl_vec3_t Ka_ = ksl_vec3(0.2, 0.2, 0.2);
  ksl_vec3_t Kd_ = ksl_vec3(1.0, 1.0, 1.0);
  ksl_vec3_t Ks_ = ksl_vec3(1.0, 1.0, 1.0);
};

class Modules {
public:
  static bool initialized();
};

} // namespace Light

} // namespace Shift

#endif
