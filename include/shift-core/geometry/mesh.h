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
  static Mesh* create();
  void addKeys();

private:
};

} // namespace Geometry

} // namespace Shift

#endif
