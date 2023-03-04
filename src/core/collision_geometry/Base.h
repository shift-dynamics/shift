#ifndef SHIFT_COLLISIONGEOMETRY_BASE_H_
#define SHIFT_COLLISIONGEOMETRY_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace body {
class Base;
}
class Frame;
class Context;

namespace collision_geometry {

/*!
@brief Abstract CollisionGeometry base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "collision_geometry";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  virtual void ActivateObjects(bool parent_is_active) override;

  virtual void Load();
  virtual void Connect();

protected:
  virtual void DeclareParameters() override;

  std::string type_; //!< descriptive name of CollisionGeometry type
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace collision_geometry
}
} // namespace shift

#endif
