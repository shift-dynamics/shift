#ifndef _SHIFT_COLLISIONGEOMETRY_H_
#define _SHIFT_COLLISIONGEOMETRY_H_

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

namespace CollisionGeometry {

/*!
@brief Abstract CollisionGeometry base class
*/
class CollisionGeometryBase
  : public Base
  , public Serialize {

public:
  CollisionGeometryBase(std::string name)
    : Base(name) {
    addKeys();
  }
  CollisionGeometryBase()
    : Base("") {
    addKeys();
  }
  ~CollisionGeometryBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of CollisionGeometry type
};

class Modules {
public:
  static bool initialized();
};

} // namespace CollisionGeometry

} // namespace Shift

#endif
