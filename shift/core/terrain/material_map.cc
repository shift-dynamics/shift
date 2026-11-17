#include "shift/core/terrain/material_map.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "shift/core/environment/surface_material.h"
#include "shift/core/exception.h"
#include "shift/core/terrain/tiff.h"

void shift::core::terrain::MappedMaterial::DeclareParameters() {
  DeclareParameter("value", &value_,
                   "the value of the samples of the material map that select "
                   "this material",
                   true);
  DeclareParameter("material", &material_,
                   "the name of a surface material (\"surface_materials\" at "
                   "the root of the model file)",
                   true);
}

void shift::core::terrain::MaterialMap::DeclareParameters() {
  DeclareParameter("filename", &filename_,
                   "single channel 8 or 16 bit integer TIFF whose samples "
                   "select the surface material of each area: a file next to "
                   "the model or in an include directory",
                   true);
  DeclareParameter("x_scale", &x_scale_,
                   "spacing of the samples along x (m), defaults to 1 m",
                   false);
  DeclareParameter("y_scale", &y_scale_,
                   "spacing of the samples along y (m), defaults to 1 m",
                   false);
  DeclareParameter("blend_width", &blend_width_,
                   "width (m) over which the viewer blends neighboring "
                   "materials, on either side of a boundary; defaults to 0.5",
                   false);
  DeclareParameter("materials", &materials_,
                   "the surface material of each value of the map, as "
                   "{\"value\": 1, \"material\": \"wet_sand\"}",
                   true);
}

void shift::core::terrain::MaterialMap::Load(
  const std::string& path, bool sae,
  const std::vector<const environment::SurfaceMaterial*>& materials,
  int base_soil, bool bekker, const std::string& owner,
  std::vector<force_module::SoilProperties>& soils) {
  const std::string prefix = owner + ": material_map: ";
  if(!(x_scale_ > 0.0) || !(y_scale_ > 0.0) || !(blend_width_ >= 0.0)) {
    throw shift::core::Exception(prefix + "x_scale and y_scale must be "
                                          "positive, blend_width not negative");
  }
  TiffImage image;
  std::string error;
  if(!ReadTiff(path, image, error)) {
    throw shift::core::Exception(prefix + path + ": " + error);
  }
  if(image.is_float || image.bits > 16) {
    throw shift::core::Exception(prefix + path + ": the map must be 8 or 16 "
                                 "bit integer samples, each a material's "
                                 "value");
  }
  /* the materials by value, each once in used_ with its soil */
  used_.clear();
  soil_.clear();
  std::map<long, int> by_value;
  std::map<const environment::SurfaceMaterial*, int> index_of;
  for(auto& mapped : materials_.get_data()) {
    const long value = mapped->get_value();
    if(by_value.count(value)) {
      throw shift::core::Exception(prefix + "two materials have the value " +
                                   std::to_string(value));
    }
    const environment::SurfaceMaterial* material = nullptr;
    for(auto m : materials) {
      if(m->get_name() == mapped->get_material()) {
        material = m;
      }
    }
    if(!material) {
      throw shift::core::Exception(prefix + "no surface material named '" +
                                   mapped->get_material() + "'");
    }
    if(!index_of.count(material)) {
      if(int(used_.size()) == kMaxMaterials) {
        throw shift::core::Exception(prefix + "at most " +
                                     std::to_string(kMaxMaterials) +
                                     " materials");
      }
      index_of[material] = int(used_.size());
      used_.push_back(material);
      int soil = base_soil;
      if(material->get_soil().is_given()) {
        soils.push_back(material->get_soil().Resolve(
          prefix + "material " + material->get_name(), bekker));
        soil = int(soils.size()) - 1;
      }
      soil_.push_back(soil);
    }
    by_value[value] = index_of[material];
  }
  /* the material of each sample, x fastest */
  nx_ = sae ? image.width : image.height;
  ny_ = sae ? image.height : image.width;
  material_.assign(size_t(nx_) * ny_, 0);
  std::set<long> missing;
  for(long row = 0; row < image.height; row++) {
    for(long column = 0; column < image.width; column++) {
      const long value =
        std::lround(image.samples[size_t(row) * image.width + column]);
      const auto it = by_value.find(value);
      if(it == by_value.end()) {
        missing.insert(value);
        continue;
      }
      const long i = sae ? column : row; // along x
      const long j = sae ? row : column; // along y
      material_[size_t(j) * nx_ + i] = uint8_t(it->second);
    }
  }
  if(!missing.empty()) {
    std::string values;
    for(long value : missing) {
      values += (values.empty() ? "" : ", ") + std::to_string(value);
    }
    throw shift::core::Exception(prefix + path +
                                 " has samples of values with no material: " +
                                 values);
  }
}

int shift::core::terrain::MaterialMap::MaterialIndex(double x,
                                                     double y) const {
  if(material_.empty()) {
    return -1;
  }
  /* clamped before rounding, so that a far point cannot overflow a long */
  const long i = std::lround(std::clamp(x / x_scale_, 0.0, double(nx_ - 1)));
  const long j = std::lround(std::clamp(y / y_scale_, 0.0, double(ny_ - 1)));
  return material_[size_t(j) * nx_ + i];
}

int shift::core::terrain::MaterialMap::SoilIndex(double x, double y) const {
  const int material = MaterialIndex(x, y);
  return material < 0 ? -1 : soil_[material];
}
