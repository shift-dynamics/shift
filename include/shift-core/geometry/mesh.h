#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift-core/geometry/base.h"

namespace Shift {

namespace Geometry {

class Mesh : public Base {
public:
  Mesh()
    : Base() {
    addKeys();
  }
  Mesh(std::string name)
    : Base(name) {
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
