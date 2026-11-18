#include "shift/core/environment/surface_material.h"

#include "shift/core/exception.h"

void shift::core::environment::SurfaceMaterial::DeclareParameters() {
  DeclareParameter("name", &name_,
                   "name of the material, by which material maps refer to it",
                   true);
  DeclareParameter("label", &label_,
                   "what the ground is, e.g. \"sand\", \"road\" or "
                   "\"grass\": the label cameras give its pixels; defaults to "
                   "the name",
                   false);
  DeclareParameter("texture_filename", &texture_filename_,
                   "image of the ground: a file next to the model or in an "
                   "include directory, or a built-in texture, e.g. "
                   "qrc:/textures/dry_sand.jpg; optional",
                   false);
  DeclareParameter("normal_map_filename", &normal_map_filename_,
                   "tangent space normal map of the texture, tiled like it; "
                   "optional",
                   false);
  DeclareParameter("roughness_map_filename", &roughness_map_filename_,
                   "roughness of the texture (its green channel, 0 to 1), "
                   "tiled like it; optional",
                   false);
  DeclareParameter("height_map_filename", &height_map_filename_,
                   "height of the texture (gray, light high), tiled like it: "
                   "where materials meet, the higher shows, as stones above "
                   "sand; optional",
                   false);
  DeclareParameter("height_scale", &height_scale_,
                   "depth of the relief of the height map (m), from its "
                   "lightest to its darkest: the viewer and the ray tracer "
                   "draw the texture sunk into the surface by up to this "
                   "much (parallax occlusion mapping), so that stones stand "
                   "out of the ground and hide what is behind them; defaults "
                   "to 0, flat",
                   false);
  DeclareParameter("texture_scale", &texture_scale_,
                   "size of one tile of the texture (m), defaults to 1",
                   false);
  DeclareParameter("normal_scale", &normal_scale_,
                   "strength of the normal map: its slopes are multiplied by "
                   "this factor; defaults to 1",
                   false);
  DeclareParameter("color", &color_,
                   "linear RGB factor of the texture (the color itself "
                   "without one); defaults to [1, 1, 1]",
                   false);
  DeclareParameter("roughness", &roughness_,
                   "factor of the roughness map, or the roughness without "
                   "one; defaults to 0.9",
                   false);
  DeclareParameter("soil", &soil_,
                   "the soil where the material lies (see force_module::Soil) "
                   "for the models that act on soil; optional, else the "
                   "terrain's",
                   false);
}

void shift::core::environment::SurfaceMaterial::Check() const {
  const std::string prefix = "surface material " + name_ + ": ";
  if(!(texture_scale_ > 0.0)) {
    throw shift::core::Exception(prefix + "texture_scale must be positive");
  }
  if(!(height_scale_ >= 0.0)) {
    throw shift::core::Exception(prefix + "height_scale must not be "
                                 "negative");
  }
  if(!(roughness_ >= 0.0 && roughness_ <= 1.0)) {
    throw shift::core::Exception(prefix + "roughness must be from 0 to 1");
  }
  if(!(color_.x >= 0.0 && color_.y >= 0.0 && color_.z >= 0.0)) {
    throw shift::core::Exception(prefix + "color must not be negative");
  }
}
