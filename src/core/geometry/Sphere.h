#ifndef SHIFT_GEOMETRY_SPHERE_H_
#define SHIFT_GEOMETRY_SPHERE_H_

#include "shift/core/geometry/Base.h"

namespace shift {
namespace core {
namespace geometry {

class Sphere : public Base {
public:
  Sphere()
    : Base() {
    DeclareParameters();
  }
  Sphere(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Sphere() override {}
  static std::unique_ptr<Sphere> create();

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
