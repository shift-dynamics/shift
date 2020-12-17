#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift/core/geometry/Base.h"
#include "shift/gui/MeshData.h"

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
  virtual void load() override;

private:
  void addParameters() override;
  static bool registered_;
  std::vector<std::string> filenames_;
  shift::gui::ShadingType shading_type_;
  std::vector<shift::gui::MeshData> mesh_data_;
  std::vector<shift::gui::MaterialData> material_data_;
};

} // namespace geometry
} // namespace core
} // namespace shift

#endif
