#include "shift/core/terrain/soil_map.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "shift/core/exception.h"
#include "shift/core/terrain/tiff.h"

void shift::core::terrain::MappedSoil::DeclareParameters() {
  shift::core::force_module::Soil::DeclareParameters();
  DeclareParameter("value", &value_,
                   "the value of the samples of the soil map that select "
                   "this soil",
                   true);
}

void shift::core::terrain::SoilMap::DeclareParameters() {
  DeclareParameter("filename", &filename_,
                   "single channel 8 or 16 bit integer TIFF whose samples "
                   "select the soil of each area: a file next to the model "
                   "or in an include directory",
                   true);
  DeclareParameter("x_scale", &x_scale_,
                   "spacing of the samples along x (m), defaults to 1 m",
                   false);
  DeclareParameter("y_scale", &y_scale_,
                   "spacing of the samples along y (m), defaults to 1 m",
                   false);
  DeclareParameter("soils", &soils_,
                   "the soil of each value of the map (see force_module::Soil), "
                   "with its \"value\"; samples of 0 with no soil of their own "
                   "have the soil of the terrain",
                   true);
}

void shift::core::terrain::SoilMap::Load(
  const std::string& path, bool sae, int base_soil, const std::string& owner,
  std::vector<shift::core::force_module::SoilProperties>& soils) {
  const std::string prefix = owner + ": soil_map: ";
  if(!(x_scale_ > 0.0) || !(y_scale_ > 0.0)) {
    throw shift::core::Exception(prefix + "x_scale and y_scale must be "
                                          "positive");
  }
  TiffImage image;
  std::string error;
  if(!ReadTiff(path, image, error)) {
    throw shift::core::Exception(prefix + path + ": " + error);
  }
  if(image.is_float || image.bits > 16) {
    throw shift::core::Exception(prefix + path + ": the map must be 8 or 16 "
                                 "bit integer samples, each a soil's value");
  }
  /* the soils by value */
  std::map<long, int> by_value;
  for(auto& soil : soils_.get_data()) {
    const long value = soil->get_value();
    if(by_value.count(value)) {
      throw shift::core::Exception(prefix + "two soils have the value " +
                                   std::to_string(value));
    }
    soils.push_back(
      soil->Resolve(prefix + "value " + std::to_string(value)));
    by_value[value] = static_cast<int>(soils.size()) - 1;
  }
  if(!by_value.count(0) && base_soil >= 0) {
    by_value[0] = base_soil;
  }
  /* the index of each sample, x fastest */
  nx_ = sae ? image.width : image.height;
  ny_ = sae ? image.height : image.width;
  index_.assign(size_t(nx_) * ny_, -1);
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
      index_[size_t(j) * nx_ + i] = it->second;
    }
  }
  if(!missing.empty()) {
    std::string values;
    for(long value : missing) {
      values += (values.empty() ? "" : ", ") + std::to_string(value);
    }
    throw shift::core::Exception(
      prefix + path + " has samples of values with no soil: " + values +
      (missing.count(0) ? " (0 takes the terrain's soil, if it has one)" : ""));
  }
}

int shift::core::terrain::SoilMap::Index(double x, double y) const {
  if(index_.empty()) {
    return -1;
  }
  /* clamped before rounding, so that a far point cannot overflow a long */
  const long i = std::lround(std::clamp(x / x_scale_, 0.0, double(nx_ - 1)));
  const long j = std::lround(std::clamp(y / y_scale_, 0.0, double(ny_ - 1)));
  return index_[size_t(j) * nx_ + i];
}
