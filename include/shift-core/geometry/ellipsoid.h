#ifndef SHIFT_GEOMETRY_ELLIPSOID_H_
#define SHIFT_GEOMETRY_ELLIPSOID_H_

#include "shift-core/factory.h"
#include "shift-core/geometry/base.h"

namespace Shift {

namespace Geometry {

class Ellipsoid : public Base {
public:
  Ellipsoid()
    : Base() {
    addKeys();
  }
  Ellipsoid(std::string name)
    : Base(name) {
    addKeys();
  }
  ~Ellipsoid() {}
  static std::unique_ptr<Ellipsoid> create();
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
