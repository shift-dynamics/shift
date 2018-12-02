#ifndef _SHIFT_ELLIPSOIDGEOMETRY_H_
#define _SHIFT_ELLIPSOIDGEOMETRY_H_

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

private:
};

} // namespace Geometry

} // namespace Shift

#endif
