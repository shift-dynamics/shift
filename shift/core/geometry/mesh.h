#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift/core/geometry/base.h"
#include "shift/gui/mesh_data.h"

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

  const std::vector<shift::gui::MeshData>& get_mesh_data() const {
    return mesh_data_;
  }

  const std::vector<std::string>& get_filenames() const {
    return filenames_;
  }

  const std::vector<shift::gui::MaterialData>& get_material_data() const {
    return material_data_;
  }

private:
  void DeclareParameters() override;
  static bool registered_;
  std::vector<std::string> filenames_;
  std::string color_space_ = "linear";
  std::string texture_mapping_ = "uv";
  double texture_scale_ = 1.0;
  shift::gui::ShadingType shading_type_;
  std::vector<shift::gui::MeshData> mesh_data_;
  std::vector<shift::gui::MaterialData> material_data_;
};

} // namespace geometry
} // namespace core
} // namespace shift

#endif
