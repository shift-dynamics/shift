#ifndef SHIFT_GEOMETRY_MESH_H_
#define SHIFT_GEOMETRY_MESH_H_

#include "shift/core/geometry/base.h"
#include "shift/gui/mesh_data.h"

#include <memory>
#include <string>
#include <vector>

namespace shift {
namespace core {
namespace geometry {

/*! \brief imported triangle meshes and their materials, shared by all
 * meshes that load the same files with the same options */
struct MeshAsset {
  std::vector<shift::gui::MeshData> mesh_data;
  /*! materials of all files; MeshData::material_index indexes them */
  std::vector<shift::gui::MaterialData> material_data;
};

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
    return asset_->mesh_data;
  }

  const std::vector<std::string>& get_filenames() const {
    return filenames_;
  }

  const std::vector<shift::gui::MaterialData>& get_material_data() const {
    return asset_->material_data;
  }

  /*! \brief the imported data; meshes that loaded the same files with the
   * same options return the same asset, so it identifies instances of one
   * mesh */
  const MeshAsset* get_asset() const {
    return asset_.get();
  }

private:
  void DeclareParameters() override;
  /*! \brief import one file, appending its meshes and materials */
  void Import(const std::string& file, MeshAsset& asset) const;
  static bool registered_;
  std::vector<std::string> filenames_;
  std::string color_space_ = "linear";
  std::string texture_mapping_ = "uv";
  double texture_scale_ = 1.0;
  shift::gui::ShadingType shading_type_;
  /*! never null: empty until loaded */
  std::shared_ptr<const MeshAsset> asset_ = std::make_shared<MeshAsset>();
};

} // namespace geometry
} // namespace core
} // namespace shift

#endif
