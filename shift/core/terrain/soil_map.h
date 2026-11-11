#ifndef SHIFT_TERRAIN_SOIL_MAP_H_
#define SHIFT_TERRAIN_SOIL_MAP_H_

#include <string>
#include <vector>

#include "shift/core/force_module/soil.h"
#include "shift/core/parameter/array.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace terrain {

/*!
@brief a soil of a soil map: a soil (see force_module::Soil) and the value of
the samples of the map that select it

JSON:
\code
{"value": 2, "name": "clayey_soil_wet", "shear_modulus": 0.02}
\endcode
*/
class MappedSoil : public force_module::Soil {
public:
  MappedSoil() {
    DeclareParameters();
  }
  virtual ~MappedSoil() override {}

  static const std::string get_array_key() {
    return "soils";
  }

  int get_value() const {
    return value_;
  }

protected:
  void DeclareParameters() override;

private:
  int value_ = 0;
};

/*!
@brief the soils of a terrain by area, from an integer TIFF image

Each sample of the map is an integer that selects a soil: the soil of
soils with that value, or for 0 without such an entry, the soil of the
terrain. The map is a single channel 8 or 16 bit integer TIFF; its samples
are read as they are, not normalized, and a float map is refused. The soil
at a point of the terrain frame is that of the nearest sample, never
interpolated, so the boundary between two soils stays sharp. The first sample
is at the origin of the terrain frame and the samples are x_scale by y_scale
apart, laid out as those of the elevation map (the terrain's coordinate_type).
Beyond the map, the soil is that of the nearest sample of its edge.

Loading throws if a value of the map has no soil (listing every such value),
if two soils have the same value, or if a soil is invalid.

JSON (SI units):
\code
"soil_map": {"filename": "soils.tiff", "x_scale": 0.25, "y_scale": 0.25,
             "soils": [{"value": 1, "name": "sandy_loam"},
                       {"value": 2, "name": "clayey_soil_wet",
                        "shear_modulus": 0.02}]}
\endcode
*/
class SoilMap : public parameter::List {
public:
  SoilMap() {
    DeclareParameters();
  }
  virtual ~SoilMap() {}

  /*! \brief whether a map was given */
  bool is_given() const {
    return !filename_.empty();
  }

  const std::string& get_filename() const {
    return filename_;
  }

  /*!
  @brief read the map at path and resolve its soils

  @param [in] path the image file
  @param [in] sae the terrain's coordinate_type is "sae" (image columns
  along x), else "iso" (image rows along x)
  @param [in] base_soil index in soils of the terrain's soil, -1 if it has
  none
  @param [in] owner name of the terrain, for errors
  @param [in,out] soils the soils of the terrain; those of the map are
  appended
  */
  void Load(const std::string& path, bool sae, int base_soil,
            const std::string& owner,
            std::vector<force_module::SoilProperties>& soils);

  /*! \brief index in the terrain's soils of the soil at x, y of the terrain
   * frame: that of the nearest sample */
  int Index(double x, double y) const;

protected:
  void DeclareParameters() override;

private:
  std::string filename_;
  double x_scale_ = 1.0;
  double y_scale_ = 1.0;
  parameter::Array<MappedSoil> soils_;

  /* index in the terrain's soils of each sample, nx_ along x by ny_ along y,
  x fastest */
  std::vector<int> index_;
  long nx_ = 0;
  long ny_ = 0;
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif
