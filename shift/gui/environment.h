#ifndef SHIFT_GUI_ENVIRONMENT_H_
#define SHIFT_GUI_ENVIRONMENT_H_

#include <QVector3D>

#include <vector>

namespace shift {
namespace gui {

/*!
@brief Sky colors; the radiance of the sky without the sun depends only on the
altitude of a direction, i.e. on its z component (+z down)
*/
struct Sky {
  QVector3D zenith;
  QVector3D horizon;
  QVector3D ground = QVector3D(0.12f, 0.11f, 0.10f);

  /*! \brief radiance from direction d, without the sun; keep in sync with
   * Sky() in sky.frag and raytrace.comp */
  QVector3D Radiance(const QVector3D& d) const;
};

/*!
@brief Image based lighting tables for the sky, in the split sum
approximation (Karis, "Real Shading in Unreal Engine 4", 2013)

The sky is symmetric about the z axis, so the tables are indexed by the z
component of a direction instead of being cube maps. Values are stored at
texel centers, as RGBA rows, to be sampled with linear filtering:
- specular: prefiltered radiance for the reflection direction, columns z from
  -1 to 1, rows roughness from 0 to 1
- irradiance: cosine weighted mean radiance about the normal, z from -1 to 1
- brdf: scale and bias applied to F0, columns n.v from 0 to 1, rows roughness
  from 0 to 1
*/
struct EnvironmentTables {
  static constexpr int kZCount = 64;
  static constexpr int kRoughnessCount = 16;
  static constexpr int kBrdfCount = 32;

  std::vector<float> specular;
  std::vector<float> irradiance;
  std::vector<float> brdf;
};

EnvironmentTables ComputeEnvironmentTables(const Sky& sky);

} // namespace gui
} // namespace shift

#endif
