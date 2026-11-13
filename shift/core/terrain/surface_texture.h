#ifndef SHIFT_TERRAIN_SURFACE_TEXTURE_H_
#define SHIFT_TERRAIN_SURFACE_TEXTURE_H_

#include <string>

#include "shift/core/parameter/list.h"

namespace shift {
namespace core {
namespace terrain {

/*!
@brief A surface drawn over part of a terrain, in patches: dirt or grass
among gravel, say

The viewer lays the surface's texture over the terrain's own where a noise
of the position on the terrain, with features about patch_size across, is
above a level chosen so that the surface covers about the fraction coverage
of the ground. The edges of the patches follow the surface's texture, so that
stones, say, stand out of the dirt around them. Later surfaces of a terrain
lie over earlier ones. The surface changes only the look of the terrain, not
its shape or its soil.

JSON (SI units):
\code
"surfaces": [{"texture_filename": "qrc:/textures/dry_sand.jpg",
              "texture_scale": 2.0, "coverage": 0.3, "patch_size": 12.0}]
\endcode
*/
class SurfaceTexture : public parameter::List {
public:
  SurfaceTexture() {
    DeclareParameters();
  }
  virtual ~SurfaceTexture() override {}

  static const std::string get_array_key() {
    return "surfaces";
  }

  /*! \brief throws if a parameter is out of range */
  void Check(const std::string& owner) const;

  const std::string& get_texture_filename() const {
    return texture_filename_;
  }
  const std::string& get_normal_map_filename() const {
    return normal_map_filename_;
  }
  /*! \brief paths of the texture and the normal map, found by the terrain
   * (empty if none or not found) */
  void set_paths(const std::string& texture, const std::string& normal_map) {
    texture_path_ = texture;
    normal_map_path_ = normal_map;
  }
  const std::string& get_texture_path() const {
    return texture_path_;
  }
  const std::string& get_normal_map_path() const {
    return normal_map_path_;
  }
  /*! \brief size of one tile of the texture (m) */
  double get_texture_scale() const {
    return texture_scale_;
  }
  /*! \brief factor of the slopes of the normal map */
  double get_normal_scale() const {
    return normal_scale_;
  }
  /*! \brief fraction of the terrain that the surface covers, 0 to 1 */
  double get_coverage() const {
    return coverage_;
  }
  /*! \brief size of the patches of the surface (m) */
  double get_patch_size() const {
    return patch_size_;
  }

protected:
  void DeclareParameters() override;

private:
  std::string texture_filename_;
  std::string normal_map_filename_;
  double texture_scale_ = 1.0;
  double normal_scale_ = 1.0;
  double coverage_ = 0.3;
  double patch_size_ = 10.0;
  std::string texture_path_;
  std::string normal_map_path_;
};

} // namespace terrain
} // namespace core
} // namespace shift

#endif
