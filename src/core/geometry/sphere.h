#ifndef SHIFT_GEOMETRY_SPHERE_H_
#define SHIFT_GEOMETRY_SPHERE_H_

#include "shift/core/geometry/base.h"

namespace shift {

namespace geometry {

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

  static bool isRegistered() {
    return registered_;
  }

private:
  void addParameters() override;
  static bool registered_;
};

} // namespace geometry

} // namespace shift

#endif
