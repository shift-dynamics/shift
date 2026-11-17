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

#include "shift/gui/environment.h"

/* a direction above the horizon at azimuth (from +x toward +y) and
elevation, radians; z is down */
static QVector3D Direction(float azimuth, float elevation) {
  return QVector3D(std::cos(elevation) * std::cos(azimuth),
                   std::cos(elevation) * std::sin(azimuth),
                   -std::sin(elevation));
}

/* u from azimuth -pi to pi, v from the zenith to the nadir, and back */
TEST(Sky, EquirectangularMap) {
  float u, v;
  shift::gui::EquirectCoordinates(QVector3D(0.f, 0.f, -1.f), u, v);
  EXPECT_NEAR(v, 0.f, 1e-6f);
  shift::gui::EquirectCoordinates(QVector3D(1.f, 0.f, 0.f), u, v);
  EXPECT_NEAR(u, 0.5f, 1e-6f);
  EXPECT_NEAR(v, 0.5f, 1e-6f);
  shift::gui::EquirectCoordinates(QVector3D(0.f, 1.f, 0.f), u, v);
  EXPECT_NEAR(u, 0.75f, 1e-6f);
  for(float a : {-2.5f, -0.3f, 0.f, 1.2f, 3.f}) {
    for(float e : {-1.2f, -0.2f, 0.f, 0.4f, 1.4f}) {
      const QVector3D d = Direction(a, e);
      shift::gui::EquirectCoordinates(d, u, v);
      const QVector3D back = shift::gui::EquirectDirection(u, v);
      EXPECT_NEAR((back - d).length(), 0.f, 1e-5f) << a << " " << e;
    }
  }
}

/* the clear sky is brightest about the sun, darkest away from it, and
bluer overhead than at the horizon; scaled so that the default sun's zenith
is about the default gradient sky's */
TEST(Sky, Preetham) {
  const shift::gui::SceneEnvironment defaults;
  const QVector3D sun = defaults.sun_direction;
  const QVector3D zenith =
    shift::gui::PreethamRadiance(QVector3D(0.f, 0.f, -1.f), sun, 3.f);
  const float scale = zenith.length() / defaults.zenith.length();
  EXPECT_GT(scale, 0.7f);
  EXPECT_LT(scale, 1.4f);
  const float sun_azimuth = std::atan2(sun.y(), sun.x());
  const QVector3D near_sun =
    shift::gui::PreethamRadiance(Direction(sun_azimuth, 0.85f), sun, 3.f);
  const QVector3D opposite = shift::gui::PreethamRadiance(
    Direction(sun_azimuth + float(M_PI), 0.85f), sun, 3.f);
  EXPECT_GT(near_sun.length(), 2.f * opposite.length());
  const QVector3D horizon =
    shift::gui::PreethamRadiance(Direction(sun_azimuth + 1.5f, 0.02f), sun,
                                 3.f);
  EXPECT_GT(zenith.z() / zenith.x(), horizon.z() / horizon.x());
  /* hazier: a brighter, whiter horizon */
  const QVector3D hazy =
    shift::gui::PreethamRadiance(Direction(sun_azimuth + 1.5f, 0.02f), sun,
                                 8.f);
  EXPECT_LT(hazy.z() / hazy.x(), horizon.z() / horizon.x());
}

/* sunlight reddens as the sun sets and as the haze thickens */
TEST(Sky, SunlightColor) {
  auto redness = [](const QVector3D& c) { return c.x() / c.z(); };
  const QVector3D high = shift::gui::SunlightColor(Direction(0.f, 1.4f), 3.f);
  const QVector3D low = shift::gui::SunlightColor(Direction(0.f, 0.05f), 3.f);
  EXPECT_GT(redness(low), 1.5f * redness(high));
  EXPECT_LT(low.length(), high.length());
  const QVector3D hazy = shift::gui::SunlightColor(Direction(0.f, 0.3f), 8.f);
  const QVector3D clear = shift::gui::SunlightColor(Direction(0.f, 0.3f), 2.f);
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
  const QVector3D up = map.Sample(Direction(0.f, 0.6f));
  EXPECT_NEAR(up.x(), 1.f, 0.01f);
  EXPECT_NEAR(up.z(), 3.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(1.57f, 0.6f)).x(), 4.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(0.f, -0.6f)).x(), 0.1f, 0.01f);
  /* turned 90 degrees: the red quarter about +y moves to about -x */
  ASSERT_TRUE(shift::gui::LoadSkyImage(path, float(M_PI / 2), map, error));
  EXPECT_NEAR(map.Sample(Direction(float(M_PI), 0.6f)).x(), 4.f, 0.01f);
  EXPECT_NEAR(map.Sample(Direction(1.57f, 0.6f)).x(), 1.f, 0.01f);
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
