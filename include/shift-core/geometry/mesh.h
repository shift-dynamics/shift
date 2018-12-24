#ifndef _SHIFT_MESHGEOMETRY_H_
#define _SHIFT_MESHGEOMETRY_H_

#include "shift-core/geometry/geometry.h"

namespace Shift {

namespace Geometry {

class Mesh : public GeometryBase {
public:
  Mesh()
    : GeometryBase() {
    addKeys();
  }
  Mesh(std::string name)
    : GeometryBase(name) {
    addKeys();
  }
  ~Mesh() {}
  static std::unique_ptr<Mesh> create();
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
