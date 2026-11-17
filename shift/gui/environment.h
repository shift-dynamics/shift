#ifndef SHIFT_GUI_ENVIRONMENT_H_
#define SHIFT_GUI_ENVIRONMENT_H_

#include <QVector3D>

#include <string>
#include <vector>

namespace shift {
namespace gui {

/*!
@brief The sky, sun and fog of a scene (from shift::core::environment), or
the viewer's own when the model gives none

Directions are in shift's world frame, +z down: the sky is at -z.
*/
struct SceneEnvironment {
  enum SkyType { kGradient = 0, kPhysical = 1, kImage = 2 };
  /*! whether the model gives an environment: its sun then shines along
   * with the model's lights, instead of giving way to them */
  bool given = false;

  SkyType sky_type = kGradient;
  QVector3D zenith{0.22f, 0.36f, 0.62f};
  QVector3D horizon{0.62f, 0.68f, 0.74f};
  QVector3D ground{0.12f, 0.11f, 0.10f};
  float turbidity = 3.f;
  std::string image_path; /*!< kImage: an OpenEXR file or resource */
  float image_rotation = 0.f; /*!< kImage: about the vertical (rad) */
  float sky_intensity = 1.f;

  /*! unit vector toward the sun */
  QVector3D sun_direction = QVector3D(0.45f, 0.3f, -0.84f).normalized();
  float sun_intensity = 3.f;
  QVector3D sun_color{1.f, 0.96f, 0.9f};
  /*! the model gives the sun's color; else a physical sky colors it */
  bool sun_color_given = false;

  float fog_density = 0.f; /*!< 1/m at fog_height, 0 for none */
  float fog_height_falloff = 0.f; /*!< 1/m, upward */
  float fog_height = 0.f;         /*!< m, upward (-z) */
  QVector3D fog_color;
  /*! the model gives the fog's color; else the sky's at the horizon */
  bool fog_color_given = false;

  bool water = false;        /*!< whether there is water */
  float water_height = 0.f;  /*!< m, upward (-z) */
  QVector3D water_color{0.012f, 0.045f, 0.055f};
  QVector3D water_extinction{0.45f, 0.12f, 0.09f}; /*!< 1/m */
  float wave_height = 0.15f;  /*!< m, crest to trough */
  float wavelength = 5.f;     /*!< m, of the longest waves */
  float wave_direction = 0.f; /*!< rad, azimuth of travel */
  float wind_speed = 0.f;     /*!< m/s, 10 m above the water */
};

/*!
@brief Radiance of the sky in every direction, in the equirectangular
projection: column u from azimuth -pi (u = 0) to pi about the vertical, from
+x toward +y; row v from the zenith (v = 0, -z) to the nadir. Texel centers
are at (i + 0.5) / width, (j + 0.5) / height. The sun's disk is not in it:
the sun is a light of its own.
*/
struct SkyMap {
  int width = 0;
  int height = 0;
  std::vector<QVector3D> texels; /*!< rows from the zenith */

  /*! \brief bilinear radiance toward d (a unit vector), wrapping in
   * azimuth */
  QVector3D Sample(const QVector3D& d) const;
  /*! \brief the map averaged over boxes of factor x factor texels */
  SkyMap Downsampled(int factor) const;
};

/*! \brief unit direction of the point (u, v) of the equirectangular
 * projection, and back */
QVector3D EquirectDirection(float u, float v);
void EquirectCoordinates(const QVector3D& d, float& u, float& v);

/*! \brief radiance of the clear sky of Preetham, Shirley and Smits ("A
 * practical analytic model for daylight", 1999) toward d, for the sun toward
 * sun and the haze turbidity; linear RGB, scaled to the viewer's units (the
 * default gradient sky's zenith for a sun 57 degrees high). Above the
 * horizon only. */
QVector3D PreethamRadiance(const QVector3D& d, const QVector3D& sun,
                           float turbidity);

/*! \brief color of sunlight through the air for the sun toward sun and the
 * haze turbidity: the transmittance of Rayleigh and aerosol scattering
 * along the air mass of its elevation, normalized to the default sun color
 * overhead */
QVector3D SunlightColor(const QVector3D& sun, float turbidity);

/*! \brief Angstrom's turbidity coefficient of the haze turbidity, the
 * optical depth of the aerosols overhead at 1 um (Preetham et al. 1999): the
 * aerosols' optical depth at wavelength l (um) is beta l^-1.3. The
 * shaders' AirTransmittance takes it. */
float AngstromBeta(float turbidity);

/*! \brief the sky map of an environment; an image that can't be read gives
 * the gradient sky, and error says why */
SkyMap BuildSkyMap(const SceneEnvironment& environment, std::string* error);

/*! \brief reads an OpenEXR image into a sky map, turned by rotation (rad)
 * about the vertical; false and error if it can't */
bool LoadSkyImage(const std::string& path, float rotation, SkyMap& map,
                  std::string& error);

/*!
@brief Image based lighting tables of a sky map, in the split sum
approximation (Karis, "Real Shading in Unreal Engine 4", 2013)

Values are RGBA rows at texel centers, for linear filtering:
- radiance: the sky map itself, kRadianceWidth x kRadianceHeight
- specular: radiance prefiltered for the reflection direction (with the
  usual n = v = r assumption), kSpecularWidth x kSpecularHeight
  equirectangular per roughness level, the levels stacked in rows (level j
  for roughness j / (kRoughnessCount - 1))
- irradiance: cosine weighted mean radiance about the normal,
  kIrradianceWidth x kIrradianceHeight equirectangular
- brdf: scale and bias applied to F0, columns n.v from 0 to 1, rows
  roughness from 0 to 1
- zenith, horizon: radiance overhead and the mean at the horizon, for the
  floor's ambient light and its fade into the distance
*/
struct EnvironmentTables {
  static constexpr int kRadianceWidth = 1024;
  static constexpr int kRadianceHeight = 512;
  static constexpr int kSpecularWidth = 64;
  static constexpr int kSpecularHeight = 32;
  static constexpr int kRoughnessCount = 8;
  static constexpr int kIrradianceWidth = 32;
  static constexpr int kIrradianceHeight = 16;
  static constexpr int kBrdfCount = 32;

  std::vector<float> radiance;
  std::vector<float> specular;
  std::vector<float> irradiance;
  std::vector<float> brdf;
  QVector3D zenith;
  QVector3D horizon;
};

/*! \brief the tables of a sky map of kRadianceWidth x kRadianceHeight */
EnvironmentTables ComputeEnvironmentTables(const SkyMap& sky);

} // namespace gui
} // namespace shift

#endif
