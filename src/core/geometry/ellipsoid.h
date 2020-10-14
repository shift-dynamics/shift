#ifndef SHIFT_GEOMETRY_ELLIPSOID_H_
#define SHIFT_GEOMETRY_ELLIPSOID_H_

#include "shift/core/factory.h"
#include "shift/core/geometry/base.h"

namespace shift {
namespace core {
namespace geometry {

class Ellipsoid : public Base {
public:
  Ellipsoid()
    : Base() {
    addParameters();
  }
  Ellipsoid(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Ellipsoid() override {}
  static std::unique_ptr<Ellipsoid> create();
  static bool isRegistered() {
    return registered_;
  }

private:
  void addParameters() override;
  static bool registered_;
};

} // namespace geometry
}
} // namespace shift

#endif
