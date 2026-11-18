#ifndef SHIFT_ENVIRONMENT_SURFACE_MATERIAL_H_
#define SHIFT_ENVIRONMENT_SURFACE_MATERIAL_H_

#include <string>

#include "shift/core/force_module/soil.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace environment {

/*!
@brief A kind of ground ("surface_materials" at the root of the model file):
how it looks, what it is, and its soil

A terrain's material map (terrain::MaterialMap) lays surface materials over
it by area. The look is a texture drawn by triplanar projection, tiled every
texture_scale meters, with an optional normal map and roughness map; color
multiplies the texture and roughness the roughness map (or is the roughness
without one). An optional height map (displacement, light high) decides
where two materials meet: the higher shows, so that stones stand out of the
sand around them instead of fading into it. The label names what the ground is ("sand", "road", "grass",
...), for the labels a camera gives its pixels. The soil (see
force_module::Soil, optional) is the soil of the terrain where the material
lies, for the models that act on soil; where it has none, the terrain's own.

JSON (SI units):
\code
"surface_materials": [
  {"name": "wet_sand", "label": "sand",
   "texture_filename": "qrc:/textures/dry_sand.jpg",
   "normal_map_filename": "qrc:/textures/dry_sand_normal.jpg",
   "texture_scale": 2.0, "color": [0.62, 0.55, 0.45], "roughness": 0.45,
   "soil": {"name": "dry_sand", "cohesion": 2000.0}}
]
\endcode
*/
class SurfaceMaterial : public parameter::List {
public:
  SurfaceMaterial() {
    DeclareParameters();
  }
  virtual ~SurfaceMaterial() override {}

  static const std::string get_array_key() {
    return "surface_materials";
  }

  /*! \brief throws if a parameter is out of range */
  void Check() const;

  const std::string& get_name() const {
    return name_;
  }
  /*! \brief the label, or the name if none is given */
  const std::string& get_label() const {
    return label_.empty() ? name_ : label_;
  }
  const std::string& get_texture_filename() const {
    return texture_filename_;
  }
  const std::string& get_normal_map_filename() const {
    return normal_map_filename_;
  }
  const std::string& get_roughness_map_filename() const {
    return roughness_map_filename_;
  }
  const std::string& get_height_map_filename() const {
    return height_map_filename_;
  }
  double get_texture_scale() const {
    return texture_scale_;
  }
  double get_normal_scale() const {
    return normal_scale_;
  }
  /*! \brief m, the depth of the relief of the height map */
  double get_height_scale() const {
    return height_scale_;
  }
  const ksl_vec3_t& get_color() const {
    return color_;
  }
  double get_roughness() const {
    return roughness_;
  }
  const force_module::Soil& get_soil() const {
    return soil_;
  }

  /*! \brief paths of the images, found by the context (empty if none or
   * not found) */
  void set_paths(const std::string& texture, const std::string& normal_map,
                 const std::string& roughness_map,
                 const std::string& height_map) {
    texture_path_ = texture;
    normal_map_path_ = normal_map;
    roughness_map_path_ = roughness_map;
    height_map_path_ = height_map;
  }
  const std::string& get_texture_path() const {
    return texture_path_;
  }
  const std::string& get_normal_map_path() const {
    return normal_map_path_;
  }
  const std::string& get_roughness_map_path() const {
    return roughness_map_path_;
  }
  const std::string& get_height_map_path() const {
    return height_map_path_;
  }

protected:
  void DeclareParameters() override;

private:
  std::string name_;
  std::string label_;
  std::string texture_filename_;
  std::string normal_map_filename_;
  std::string roughness_map_filename_;
  std::string height_map_filename_;
  double texture_scale_ = 1.0;
  double normal_scale_ = 1.0;
  double height_scale_ = 0.0;
  ksl_vec3_t color_ = ksl_vec3(1.0, 1.0, 1.0);
  double roughness_ = 0.9;
  force_module::Soil soil_;
  std::string texture_path_;
  std::string normal_map_path_;
  std::string roughness_map_path_;
  std::string height_map_path_;
};

} // namespace environment
} // namespace core
} // namespace shift

#endif
