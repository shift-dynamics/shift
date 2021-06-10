#ifndef SHIFT_GEOMETRY_ELLIPSOID_H_
#define SHIFT_GEOMETRY_ELLIPSOID_H_

#include "shift/core/Factory.h"
#include "shift/core/geometry/Base.h"

namespace shift {
namespace core {
namespace geometry {

class Ellipsoid : public Base {
public:
  Ellipsoid()
    : Base() {
    DeclareParameters();
  }
  Ellipsoid(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Ellipsoid() override {}
  static std::unique_ptr<Ellipsoid> create();
  static bool is_registered() {
    return registered_;
  }

private:
  void DeclareParameters() override;
  static bool registered_;
};

} // namespace geometry
}
} // namespace shift

#endif
