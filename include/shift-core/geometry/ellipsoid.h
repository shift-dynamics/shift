#ifndef _SHIFT_ELLIPSOIDGEOMETRY_H_
#define _SHIFT_ELLIPSOIDGEOMETRY_H_

#include "shift-core/factory.h"
#include "shift-core/geometry/geometry.h"

namespace Shift {

namespace Geometry {

class Ellipsoid : public GeometryBase {
public:
  Ellipsoid()
    : GeometryBase() {
    addKeys();
  }
  Ellipsoid(std::string name)
    : GeometryBase(name) {
    addKeys();
  }
  ~Ellipsoid() {}
  static Ellipsoid* create();
  void addKeys();
  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Geometry

} // namespace Shift

#endif
