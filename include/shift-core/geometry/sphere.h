#ifndef _SHIFT_SPHEREGEOMETRY_H_
#define _SHIFT_SPHEREGEOMETRY_H_

#include "shift-core/geometry/geometry.h"

namespace Shift {

namespace Geometry {

class Sphere : public GeometryBase {
public:
  Sphere()
    : GeometryBase() {
    addKeys();
  }
  Sphere(std::string name)
    : GeometryBase(name) {
    addKeys();
  }
  ~Sphere() {}
  static std::unique_ptr<Sphere> create();
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
