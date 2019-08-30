#ifndef SHIFT_GEOMETRY_SPHERE_H_
#define SHIFT_GEOMETRY_SPHERE_H_

#include "shift-core/geometry/base.h"

namespace Shift {

namespace Geometry {

class Sphere : public Base {
public:
  Sphere()
    : Base() {
    addParameters();
  }
  Sphere(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Sphere() override {}
  static std::unique_ptr<Sphere> create();
  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Geometry

} // namespace Shift

#endif
