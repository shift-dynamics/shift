/* Terrain contact force module (force_module::TerrainContact): rest on four
points, Hunt and Crossley's force, friction, and a slope of an elevation map */

#include <tiffio.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/terrain_contact.h"

static const double kWeight = 2.0 * 9.81;

/* a level, zero elevation map of 0.25 m cells, 20 m a side */
static void WriteFlatMap(const std::string& path) {
  const int n = 81;
  TIFF* tiff = TIFFOpen(path.c_str(), "w");
  ASSERT_NE(tiff, nullptr);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, n);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, n);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, 1);
  std::vector<float> row(n, 0.0f);
  for(int r = 0; r < n; r++) {
    TIFFWriteScanline(tiff, row.data(), r, 0);
  }
  TIFFClose(tiff);
}

/* a body on a float joint at the height z, sliding along x at speed */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      double z, double speed, const std::string& keys,
                      double time_span, const std::string& frames = "",
                      const std::string& terrain = "") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
      "include_directories": [")" << testing::TempDir() << R"("],
      "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.01, "relative_error": 1e-7,
                             "absolute_error": 1e-9}]},
      "frames": [)" << frames << R"(], "terrain": [)" << terrain << R"(],
      "assemblies": [{"name": "test", "bodies": [{"name": "body",
        "center_of_mass": [0, 0, 0],
        "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                    "Iyz": 0, "Izx": 0}}],
      "joints": [{"name": "j", "type": "float",
        "parent": [["fixed"], "origin"], "child": [["body"], "origin"],
        "initial_displacement": {"T": [0, 0, )" << z << R"(],
          "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "initial_velocity": [)" << speed << R"(, 0, 0, 0, 0, 0]}],
      "force_modules": [{"type": "terrain_contact", "name": "c",
        "body": "body",
        "points": [0.2, 0.2, 0, 0.2, -0.2, 0, -0.2, 0.2, 0, -0.2, -0.2, 0],
        )" << keys << R"(}]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::force_module::TerrainContact* Find(
  shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto m =
         dynamic_cast<shift::core::force_module::TerrainContact*>(module)) {
      return m;
    }
  }
  return nullptr;
}

static ksl_vec3_t RunToEnd(shift::core::Context& context) {
  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  std::copy(last.begin(), last.end(), context.get_q().begin());
  context.UpdateDHVA();
  auto body = context.FindActiveBody("body", nullptr);
  return context.get_body_origin_pose(body, context.get_dhva()).t;
}

TEST(TerrainContact, ABodyRestsOnItsPoints) {
  // four points of stiffness k share the weight: each sinks m g / (4 k)
  shift::core::Context context;
  LoadModel(context, "tc_rest", 0.0, 0.0,
            R"("stiffness": 2000, "damping": 20)", 5.0);
  EXPECT_NEAR(RunToEnd(context).z, kWeight / 8000.0, 1e-5);
}

TEST(TerrainContact, HuntAndCrossleyHasNoJumpAsTheContactStarts) {
  // falling onto the ground the force starts from zero, and sinking it
  // grows: with the exponent 1.5 the rest depth is (m g / (4 k))^(2 / 3)
  shift::core::Context context;
  LoadModel(context, "tc_exponent", -0.01, 0.0,
            R"("stiffness": 50000, "exponent": 1.5, "damping": 20)", 5.0);
  EXPECT_NEAR(RunToEnd(context).z, std::pow(kWeight / 200000.0, 2.0 / 3.0),
              1e-5);
}

TEST(TerrainContact, NoForceInTheAir) {
  shift::core::Context context;
  LoadModel(context, "tc_air", -0.5, 0.0, R"("stiffness": 2000)", 0.1);
  RunToEnd(context);
  EXPECT_EQ(Find(context)->get_normal_force(), 0.0);
}

TEST(TerrainContact, FrictionStopsASlide) {
  // from 2 m/s at mu 0.4: 2^2 / (2 mu g) = 0.51 m
  shift::core::Context context;
  LoadModel(context, "tc_slide", kWeight / 4e5, 2.0,
            R"("stiffness": 1e5, "damping": 5, "friction": 0.4,
               "slip_speed": 1e-3)",
            1.5);
  EXPECT_NEAR(RunToEnd(context).x, 4.0 / (2.0 * 0.4 * 9.81), 0.02);
}

TEST(TerrainContact, ABodySlidesDownAnElevationMap) {
  // a level map on a frame tilted by a about y, through the origin, downhill
  // towards -x: the acceleration along the slope is g (sin a - mu cos a)
  const std::string map = testing::TempDir() + "/tc_flat.tiff";
  WriteFlatMap(map);
  const double a = 0.2, mu = 0.1, c = std::cos(a), s = std::sin(a);
  std::ostringstream frame;
  frame.precision(17);
  frame << R"({"name": "slope", "T": [)" << -10.0 * c << ", -10, "
        << 10.0 * s << R"(], "R": [[)" << c << ", 0, " << s
        << R"(], [0, 1, 0], [)" << -s << ", 0, " << c << "]]}";
  shift::core::Context context;
  LoadModel(context, "tc_slope", 0.0, 0.0,
            R"("stiffness": 1e5, "damping": 5, "friction": 0.1,
               "slip_speed": 1e-3)",
            2.0, frame.str(),
            R"({"type": "basic", "name": "map", "filename": "tc_flat.tiff",
                "parent": [["fixed"], "slope"], "x_scale": 0.25,
                "y_scale": 0.25})");
  const ksl_vec3_t end = RunToEnd(context);
  std::remove(map.c_str());
  const double along = 0.5 * 9.81 * (s - mu * c) * 4.0;
  EXPECT_NEAR(end.x, -along * c, 0.05 * along);
  // it stays on the surface as it goes
  EXPECT_NEAR(end.z, -end.x * s / c, 0.01);
}

TEST(TerrainContact, RefusesANegativeStiffness) {
  shift::core::Context context;
  EXPECT_THROW(LoadModel(context, "tc_bad", 0.0, 0.0,
                         R"("stiffness": -1)", 0.0),
               shift::core::Exception);
}
