#ifndef _SHIFT_LIGHT_H_
#define _SHIFT_LIGHT_H_

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

namespace Light {

/*!
@brief Abstract Shift::Light base class
*/
class LightBase
  : public Base
  , public Serialize {

public:
  LightBase(std::string name)
    : Base(name) {}
  LightBase()
    : Base("") {}
  ~LightBase() {}

  const std::string& getType() {
    return type_;
  }

  virtual void readJSON(Json::Value&);
  virtual void print();

protected:
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
