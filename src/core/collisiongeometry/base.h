#ifndef SHIFT_COLLISIONGEOMETRY_BASE_H_
#define SHIFT_COLLISIONGEOMETRY_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class Base;
}
class Frame;
class Context;

namespace collisiongeometry {

/*!
@brief Abstract CollisionGeometry base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    addParameters();
  }
  Base()
    : shift::core::Base("") {
    addParameters();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  virtual void setActiveObjects(bool parent_is_active) override;

  virtual void load();
  virtual void connect();
  static const std::string arrayKey() {
    return "collision_geometry";
  }

  static const std::string defaultType() {
    return "";
  }

protected:
  virtual void addParameters() override;

  std::string type_; //!< descriptive name of CollisionGeometry type
};

class Modules {
public:
  static bool initialized();
};

} // namespace collisiongeometry
}
} // namespace shift

#endif
