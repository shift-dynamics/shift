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
    : Base(name) {
    addKeys();
  }
  LightBase()
    : Base("") {
    addKeys();
  }
  ~LightBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Light type
};

} // namespace Light

} // namespace Shift

#endif
