/* the viewer's sky (shift/gui/environment.h): the equirectangular map of
directions, the clear sky of Preetham et al., the color of sunlight through
the air, and a sky from an OpenEXR image */

#include <cmath>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#ifdef SHIFT_HAVE_OPENEXR
#include <ImfArray.h>
#include <ImfRgbaFile.h>
#endif

#include "ksl/vec3.h"
#include "shift/gui/environment.h"

/* the length of v */
static float Length(const ksl_vec3f_t& v) { return ksl_vec3f_l2norm(&v); }

/* the length of a - b */
static float Distance(const ksl_vec3f_t& a, const ksl_vec3f_t& b) {
  ksl_vec3f_t d;
  ksl_subtract_vvf(&a, &b, &d);
  return ksl_vec3f_l2norm(&d);
}

/* a direction above the horizon at azimuth (from +x toward +y) and
elevation, radians; z is down */
static ksl_vec3f_t Direction(float azimuth, float elevation) {
  return ksl_vec3f(std::cos(elevation) * std::cos(azimuth),
                   std::cos(elevation) * std::sin(azimuth),
                   -std::sin(elevation));
}

/* u from azimuth -pi to pi, v from the zenith to the nadir, and back */
TEST(Sky, EquirectangularMap) {
  float u, v;
  shift::gui::EquirectCoordinates(ksl_vec3f(0.f, 0.f, -1.f), u, v);
  EXPECT_NEAR(v, 0.f, 1e-6f);
  shift::gui::EquirectCoordinates(ksl_vec3f(1.f, 0.f, 0.f), u, v);
  EXPECT_NEAR(u, 0.5f, 1e-6f);
  EXPECT_NEAR(v, 0.5f, 1e-6f);
  shift::gui::EquirectCoordinates(ksl_vec3f(0.f, 1.f, 0.f), u, v);
  EXPECT_NEAR(u, 0.75f, 1e-6f);
  for(float a : {-2.5f, -0.3f, 0.f, 1.2f, 3.f}) {
    for(float e : {-1.2f, -0.2f, 0.f, 0.4f, 1.4f}) {
      const ksl_vec3f_t d = Direction(a, e);
      shift::gui::EquirectCoordinates(d, u, v);
      const ksl_vec3f_t back = shift::gui::EquirectDirection(u, v);
      EXPECT_NEAR(Distance(back, d), 0.f, 1e-5f) << a << " " << e;
    }
  }
}

/* the clear sky is brightest about the sun, darkest away from it, and
bluer overhead than at the horizon; scaled so that the default sun's zenith
is about the default gradient sky's */
TEST(Sky, Preetham) {
  const shift::gui::SceneEnvironment defaults;
  const ksl_vec3f_t sun = defaults.sun_direction;
  const ksl_vec3f_t zenith =
    shift::gui::PreethamRadiance(ksl_vec3f(0.f, 0.f, -1.f), sun, 3.f);
  const float scale = Length(zenith) / Length(defaults.zenith);
  EXPECT_GT(scale, 0.7f);
  EXPECT_LT(scale, 1.4f);
  const float sun_azimuth = std::atan2(sun.y, sun.x);
  const ksl_vec3f_t near_sun =
    shift::gui::PreethamRadiance(Direction(sun_azimuth, 0.85f), sun, 3.f);
  const ksl_vec3f_t opposite = shift::gui::PreethamRadiance(
    Direction(sun_azimuth + float(M_PI), 0.85f), sun, 3.f);
  EXPECT_GT(Length(near_sun), 2.f * Length(opposite));
  const ksl_vec3f_t horizon =
    shift::gui::PreethamRadiance(Direction(sun_azimuth + 1.5f, 0.02f), sun,
                                 3.f);
  EXPECT_GT(zenith.z / zenith.x, horizon.z / horizon.x);
  /* hazier: a brighter, whiter horizon */
  const ksl_vec3f_t hazy =
    shift::gui::PreethamRadiance(Direction(sun_azimuth + 1.5f, 0.02f), sun,
                                 8.f);
  EXPECT_LT(hazy.z / hazy.x, horizon.z / horizon.x);
}

/* sunlight reddens as the sun sets and as the haze thickens */
TEST(Sky, SunlightColor) {
  auto redness = [](const ksl_vec3f_t& c) { return c.x / c.z; };
  const ksl_vec3f_t high = shift::gui::SunlightColor(Direction(0.f, 1.4f), 3.f);
  const ksl_vec3f_t low = shift::gui::SunlightColor(Direction(0.f, 0.05f), 3.f);
  EXPECT_GT(redness(low), 1.5f * redness(high));
  EXPECT_LT(Length(low), Length(high));
  const ksl_vec3f_t hazy = shift::gui::SunlightColor(Direction(0.f, 0.3f), 8.f);
  const ksl_vec3f_t clear = shift::gui::SunlightColor(Direction(0.f, 0.3f), 2.f);
  EXPECT_GT(redness(hazy), redness(clear));
}

#ifdef SHIFT_HAVE_OPENEXR
/* an image of a bright sky above the horizon, a dark ground below it and a
red quarter about +y; turned by the rotation */
TEST(Sky, OpenExrImage) {
  const int width = 64, height = 32;
  Imf::Array2D<Imf::Rgba> pixels(height, width);
  for(int j = 0; j < height; j++) {
    for(int i = 0; i < width; i++) {
      const float u = (i + 0.5f) / width;
      const bool sky = j < height / 2;
      const bool red = u > 0.625f && u < 0.875f; // azimuth 45 to 135 deg
      pixels[j][i] = sky ? (red ? Imf::Rgba(4.f, 0.f, 0.f, 1.f)
                                : Imf::Rgba(1.f, 2.f, 3.f, 1.f))
                         : Imf::Rgba(0.1f, 0.1f, 0.1f, 1.f);
    }
  }
  const std::string path = testing::TempDir() + "/sky_test.exr";
  {
    Imf::RgbaOutputFile file(path.c_str(), width, height, Imf::WRITE_RGBA);
    file.setFrameBuffer(&pixels[0][0], 1, width);
    file.writePixels(height);
  }
  shift::gui::SkyMap map;
  std::string error;
  ASSERT_TRUE(shift::gui::LoadSkyImage(path, 0.f, map, error)) << error;
  EXPECT_EQ(map.width, shift::gui::EnvironmentTables::kRadianceWidth);
  const ksl_vec3f_t up = map.Sample(Direction(0.f, 0.6f));
  EXPECT_NEAR(up.x, 1.f, 0.01f);
  EXPECT_NEAR(up.z, 3.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(1.57f, 0.6f)).x, 4.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(0.f, -0.6f)).x, 0.1f, 0.01f);
  /* turned 90 degrees: the red quarter about +y moves to about -x */
  ASSERT_TRUE(shift::gui::LoadSkyImage(path, float(M_PI / 2), map, error));
  EXPECT_NEAR(map.Sample(Direction(float(M_PI), 0.6f)).x, 4.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(1.57f, 0.6f)).x, 1.f, 0.01f);
  /* a file that is not an image */
  {
    std::FILE* f = std::fopen(path.c_str(), "w");
    std::fputs("not an image", f);
    std::fclose(f);
  }
  EXPECT_FALSE(shift::gui::LoadSkyImage(path, 0.f, map, error));
  EXPECT_FALSE(error.empty());
  std::remove(path.c_str());
}
#endif
