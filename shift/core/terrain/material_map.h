#ifndef SHIFT_TERRAIN_MATERIAL_MAP_H_
#define SHIFT_TERRAIN_MATERIAL_MAP_H_

#include <cstdint>
#include <string>
#include <vector>

#include "shift/core/force_module/soil.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace environment {
class SurfaceMaterial;
}
namespace terrain {

/*!
@brief a material of a material map: the surface material (by name) and the
value of the samples of the map that select it

JSON:
\code
{"value": 1, "material": "wet_sand"}
\endcode
*/
class MappedMaterial : public parameter::List {
public:
  MappedMaterial() {
    DeclareParameters();
  }
  virtual ~MappedMaterial() override {}

  static const std::string get_array_key() {
    return "materials";
  }

  int get_value() const {
    return value_;
  }
  const std::string& get_material() const {
    return material_;
  }

protected:
  void DeclareParameters() override;

private:
  int value_ = 0;
  std::string material_;
};

/*!
@brief the surface materials of a terrain by area (see
environment::SurfaceMaterial), from an integer TIFF image

Each sample of the map is an integer that selects the surface material of
materials with that value. The map is laid out as a soil map (SoilMap): a
single channel 8 or 16 bit integer TIFF, its first sample at the origin of
the terrain frame and the samples x_scale by y_scale apart, as those of the
elevation map; beyond the map, the material is that of the nearest sample of
its edge.

The soil at a point is the soil of the material of the nearest sample, or
the terrain's own where that material has none: so a material map takes the
place of a soil map, and a terrain has one or the other. The viewer blends
the looks of the materials between samples, over blend_width (m) on either
side of a boundary, broken up by noise so that it does not follow the grid.

Loading throws if a value of the map has no material (listing every such
value), if two materials have the same value, or if a material is not among
the surface materials.

JSON (SI units):
\code
"material_map": {"filename": "beach_materials.tiff",
                 "x_scale": 0.5, "y_scale": 0.5, "blend_width": 0.6,
                 "materials": [{"value": 0, "material": "dry_sand"},
                               {"value": 1, "material": "wet_sand"}]}
\endcode
*/
class MaterialMap : public parameter::List {
public:
  MaterialMap() {
    DeclareParameters();
  }
  virtual ~MaterialMap() {}

  /*! \brief whether a map was given */
  bool is_given() const {
    return !filename_.empty();
  }
  const std::string& get_filename() const {
    return filename_;
  }

  /*!
  @brief read the map at path and resolve its materials and their soils
  @param [in] path the image file
  @param [in] sae the terrain's coordinate_type is "sae" (image columns
  along x), else "iso" (image rows along x)
  @param [in] materials the surface materials of the model
  @param [in] base_soil index in soils of the terrain's soil, -1 if it has
  none
  @param [in] bekker whether the soils need Bekker's moduli
  @param [in] owner name of the terrain, for errors
  @param [in,out] soils the soils of the terrain; those of the materials
  are appended
  */
  void Load(const std::string& path, bool sae,
            const std::vector<const environment::SurfaceMaterial*>& materials,
            int base_soil, bool bekker, const std::string& owner,
            std::vector<force_module::SoilProperties>& soils);

  /*! \brief index in get_materials() of the material at x, y of the
   * terrain frame (that of the nearest sample), -1 if the map is not loaded */
  int MaterialIndex(double x, double y) const;

  /*! \brief index in the terrain's soils of the soil at x, y of the terrain
   * frame (that of the nearest sample), -1 if none */
  int SoilIndex(double x, double y) const;

  /*! \brief the materials in use, in the order of the indices of samples */
  const std::vector<const environment::SurfaceMaterial*>& get_materials()
    const {
    return used_;
  }
  /*! \brief index in get_materials of each sample, x fastest */
  const std::vector<uint8_t>& get_indices() const {
    return material_;
  }
  long get_x_samples() const {
    return nx_;
  }
  long get_y_samples() const {
    return ny_;
  }
  double get_x_scale() const {
    return x_scale_;
  }
  double get_y_scale() const {
    return y_scale_;
  }
  double get_blend_width() const {
    return blend_width_;
  }

  /*! \brief the most materials a map can hold */
  static constexpr int kMaxMaterials = 16;

protected:
  void DeclareParameters() override;

private:
  std::string filename_;
  double x_scale_ = 1.0;
  double y_scale_ = 1.0;
  double blend_width_ = 0.5;
  parameter::Array<MappedMaterial> materials_;

  long nx_ = 0;
  long ny_ = 0;
  std::vector<const environment::SurfaceMaterial*> used_;
  std::vector<uint8_t> material_; //!< index in used_ of each sample
  std::vector<int> soil_;         //!< soil of each material in used_
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif
