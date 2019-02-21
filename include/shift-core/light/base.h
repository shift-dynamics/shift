#ifndef SHIFT_LIGHT_BASE_H_
#define SHIFT_LIGHT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

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
  , public Serialize {

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

  virtual void readJSON(Json::Value&);
  virtual void print();
  virtual void load();
  virtual void connect();

protected:
  Context* context_;
  void addKeys();
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
