#ifndef SHIFT_GEOMETRY_SPHERE_H_
#define SHIFT_GEOMETRY_SPHERE_H_

#include "shift-core/geometry/base.h"

namespace Shift {

namespace Geometry {

class Sphere : public Base {
public:
  Sphere()
    : Base() {
    addKeys();
  }
  Sphere(std::string name)
    : Base(name) {
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
