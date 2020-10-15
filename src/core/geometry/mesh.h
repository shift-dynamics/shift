#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift/core/geometry/base.h"

namespace shift {
namespace core {
namespace geometry {

class Mesh : public Base {
public:
  Mesh()
    : Base() {
    addParameters();
  }
  Mesh(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Mesh() override {}
  static std::unique_ptr<Mesh> create();

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
