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
    DeclareParameters();
  }
  Mesh(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Mesh() override {}

  static std::unique_ptr<Mesh> Create();

  static bool is_registered() {
    return registered_;
  }

  virtual void Load() override;

private:
  void DeclareParameters() override;
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
