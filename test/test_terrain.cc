/* Elevation map terrain (terrain::Basic): surface queries, and a tire
resting on it */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <tiffio.h>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/terrain/base.h"
#include "shift/core/terrain/basic.h"

/* a single channel 32 bit float TIFF, sample(row, column) */
static void WriteTiff(const std::string& path, int width, int height,
                      const std::function<float(int, int)>& sample) {
  TIFF* tiff = TIFFOpen(path.c_str(), "w");
  ASSERT_NE(tiff, nullptr);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, 1);
  std::vector<float> row(width);
  for(int r = 0; r < height; r++) {
    for(int c = 0; c < width; c++) {
      row[c] = sample(r, c);
    }
    TIFFWriteScanline(tiff, row.data(), r, 0);
  }
  TIFFClose(tiff);
}

/* a single channel 16 bit (half) float TIFF of the halves sample(row, column) */
static void WriteHalfTiff(const std::string& path, int width, int height,
                          const std::function<uint16_t(int, int)>& sample) {
  TIFF* tiff = TIFFOpen(path.c_str(), "w");
  ASSERT_NE(tiff, nullptr);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, 16);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, height);
  std::vector<uint16_t> row(width);
  for(int r = 0; r < height; r++) {
    for(int c = 0; c < width; c++) {
      row[c] = sample(r, c);
    }
    TIFFWriteScanline(tiff, row.data(), r, 0);
  }
  TIFFClose(tiff);
}

/* an upright wheel on a vertical prismatic joint, with a terrain if terrain
is not empty. Without forward speed the transient slip of the Fiala model is
undamped, so the wheel neither moves forward nor spins. */
static void LoadWheel(shift::core::Context& context, const std::string& name,
                      const std::string& terrain, const std::string& frame,
                      double z0, double time_span) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")" << testing::TempDir() << R"("],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )" << time_span
        << R"(, "time_step": 0.01, "relative_error": 1e-6,
                     "absolute_error": 1e-6}]
  },
  "frames": [)" << frame << R"(],
  "terrain": [)" << terrain << R"(],
  "assemblies": [{"name": "tire",
    "force_modules": [{"type": "tire", "name": "tire", "tire_type": "fiala",
                       "wheel_body": "wheel"}],
    "bodies": [{"name": "wheel", "geometry": [], "center_of_mass": [0, 0, 0],
                "inertia": {"mass": 20.0, "Ixx": 0.578, "Iyy": 0.931,
                            "Izz": 0.578, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "C", "type": "prismatic", "parent": [["fixed"], "origin"],
       "child": [["wheel"], "origin"], "axis": [0, 0, 1],
       "initial_displacement": )" << z0 << R"(}]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static ksl_vec3_t WheelCenter(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "wheel") {
      return context.get_body_origin_pose(body).t;
    }
  }
  ADD_FAILURE() << "no wheel";
  return {{0, 0, 0}};
}

static std::string Terrain(const std::string& file, const std::string& extra) {
  return R"({"type": "basic", "name": "map", "filename": ")" + file +
         R"(", "parent": [["fixed"], "terrain"], "x_scale": 0.5,
            "y_scale": 0.25)" + extra + "}";
}

/* a terrain frame turned 30 degrees about z and moved */
static const char* kTerrainFrame =
  R"({"name": "terrain", "T": [-3.0, -1.0, 0.2],
      "R": [[0.8660254037844387, -0.5, 0.0], [0.5, 0.8660254037844387, 0.0],
            [0.0, 0.0, 1.0]]})";

/* a plane z = a x + b y + c of the terrain frame is reproduced exactly by
both interpolations, including the cells at the edges of the map */
TEST(Terrain, PlaneQuery) {
  const double a = 0.05, b = -0.08, c = 0.1; // dz/dx, dz/dy, z(0, 0)
  const std::string file = "terrain_plane.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 20, 16, [&](int row, int column) {
    return float(a * 0.5 * column + b * 0.25 * row + c); // sae: x by column
  });
  for(const char* interpolation : {"bilinear", "bicubic"}) {
    shift::core::Context context;
    LoadWheel(context, std::string("plane_") + interpolation,
              Terrain(file, std::string(R"(, "interpolation_type": ")") +
                              interpolation + "\""),
              kTerrainFrame, -0.345, 0.0);
    ASSERT_EQ(context.get_active_terrain().size(), 1u);
    const auto* terrain = context.get_active_terrain()[0];
    const double cos30 = std::sqrt(3.0) / 2.0, sin30 = 0.5;
    for(const auto& local : std::vector<std::pair<double, double>>{
          {1.3, 0.9}, {4.71, 2.02}, {7.5, 3.0}, {2.0, 1.5}, {0.1, 0.05},
          {9.4, 3.7}}) {
      const double x = local.first, y = local.second;
      /* a point 1 m above the surface, in the inertial frame */
      const double z = a * x + b * y + c;
      const ksl_vec3_t point = {{-3.0 + cos30 * x - sin30 * y,
                                 -1.0 + sin30 * x + cos30 * y, 0.2 + z - 1.0}};
      ksl_vec3_t surface, normal;
      ASSERT_TRUE(terrain->Query(point, surface, normal));
      EXPECT_NEAR(surface.x, point.x, 1e-12) << interpolation;
      EXPECT_NEAR(surface.y, point.y, 1e-12) << interpolation;
      EXPECT_NEAR(surface.z, 0.2 + z, 1e-6) << interpolation << " " << x;
      /* into the terrain: (-a, -b, 1) normalized, turned about z */
      const double length = std::sqrt(a * a + b * b + 1.0);
      EXPECT_NEAR(normal.x, (-a * cos30 + b * sin30) / length, 1e-6);
      EXPECT_NEAR(normal.y, (-a * sin30 - b * cos30) / length, 1e-6);
      EXPECT_NEAR(normal.z, 1.0 / length, 1e-6);
    }
  }
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* the bicubic surface passes through every sample of the map, including
those of its edges and corners, and is C2 continuous across the boundaries of
the cells: its slope and second derivative are the same on both sides */
TEST(Terrain, BicubicInterpolatesTheSamples) {
  const int columns = 9, rows = 7;
  const double dx = 0.5, dy = 0.25;
  /* a fixed pattern of samples, from a constant seed */
  std::vector<float> samples(columns * rows);
  unsigned state = 12345u;
  for(auto& sample : samples) {
    state = state * 1664525u + 1013904223u;
    sample = float(0.3 * (state >> 8) / double(1u << 24));
  }
  const std::string file = "terrain_samples.tiff";
  WriteTiff(testing::TempDir() + "/" + file, columns, rows,
            [&](int row, int column) { return samples[row * columns + column]; });
  shift::core::Context context;
  LoadWheel(context, "samples", Terrain(file, ""),
            R"({"name": "terrain", "T": [0.0, 0.0, 0.0]})", -0.345, 0.0);
  ASSERT_EQ(context.get_active_terrain().size(), 1u);
  const auto* terrain = dynamic_cast<const shift::core::terrain::Basic*>(
    context.get_active_terrain()[0]);
  ASSERT_NE(terrain, nullptr);
  ASSERT_TRUE(terrain->is_bicubic());
  double z_x, z_y;
  for(int row = 0; row < rows; row++) {
    for(int column = 0; column < columns; column++) {
      EXPECT_NEAR(terrain->LocalZ(column * dx, row * dy, z_x, z_y),
                  samples[row * columns + column], 1e-12)
        << row << " " << column;
    }
  }
  /* across the boundary x = 2 dx, along the line y = 1.3 dy: the slope on
  each side, and the second derivative from the slopes on each side */
  const double x = 2.0 * dx, y = 1.3 * dy, h = 1e-6;
  auto slope = [&](double at) {
    terrain->LocalZ(at, y, z_x, z_y);
    return z_x;
  };
  EXPECT_NEAR(slope(x - h), slope(x + h), 1e-5);
  const double left = (slope(x - h) - slope(x - 3.0 * h)) / (2.0 * h);
  const double right = (slope(x + 3.0 * h) - slope(x + h)) / (2.0 * h);
  EXPECT_NEAR(left, right, 1e-4 * (1.0 + std::fabs(left)));
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* iso: rows along x, columns along y, samples are heights (up) */
TEST(Terrain, IsoConvention) {
  const std::string file = "terrain_iso.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 12, 10, [](int row, int column) {
    return float(0.02 * row + 0.03 * column);
  });
  shift::core::Context context;
  LoadWheel(context, "iso",
            Terrain(file, R"(, "coordinate_type": "iso", "interpolation_type": "bilinear")"),
            R"({"name": "terrain", "T": [0, 0, 0]})", -0.345, 0.0);
  const auto* terrain = context.get_active_terrain()[0];
  ksl_vec3_t surface, normal;
  /* sample row 3 at x = 1.5, column 4 at y = 1.0 */
  ASSERT_TRUE(terrain->Query({{1.5, 1.0, -5.0}}, surface, normal));
  EXPECT_NEAR(surface.z, -(0.02 * 3 + 0.03 * 4), 1e-6);
  EXPECT_LT(normal.z, 0.0 + 1.0); // into the terrain, mostly +z
  EXPECT_GT(normal.z, 0.9);
}

/* 16 bit float maps (e.g. examples/excavator) are read exactly */
TEST(Terrain, HalfFloat) {
  const std::string file = "terrain_half.tiff";
  /* halves 0x3800 = 0.5, 0xc000 = -2, 0x2e66 = 0.0999755859375 and the
  subnormal 0x0001 = 2^-24, in the four quadrants of the map */
  WriteHalfTiff(testing::TempDir() + "/" + file, 8, 8, [](int row, int column) {
    const uint16_t halves[4] = {0x3800, 0xc000, 0x2e66, 0x0001};
    return halves[2 * (row >= 4) + (column >= 4)];
  });
  shift::core::Context context;
  LoadWheel(context, "half",
            Terrain(file, R"(, "interpolation_type": "bilinear")"),
            R"({"name": "terrain", "T": [0, 0, 0]})", -0.345, 0.0);
  const auto* terrain = context.get_active_terrain()[0];
  ksl_vec3_t surface, normal;
  /* sae: x = 0.5 column, y = 0.25 row */
  const double expected[4] = {0.5, -2.0, 0.0999755859375, std::ldexp(1.0, -24)};
  const double x[2] = {0.5, 3.0}, y[2] = {0.25, 1.5};
  for(int quadrant = 0; quadrant < 4; quadrant++) {
    ASSERT_TRUE(terrain->Query({{x[quadrant % 2], y[quadrant / 2], -5.0}},
                               surface, normal));
    EXPECT_DOUBLE_EQ(surface.z, expected[quadrant]) << quadrant;
  }
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* a wheel comes to rest as far above a flat terrain 0.3 m above the ground
as it does above the ground without a terrain */
TEST(Terrain, TireRestsOnTerrain) {
  shift::core::Context flat;
  LoadWheel(flat, "flat", "", R"({"name": "terrain", "T": [0, 0, 0]})", -0.345,
            1.5);
  flat.Simulate();
  const ksl_vec3_t on_ground = WheelCenter(flat);

  const std::string file = "terrain_raised.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 8, 8,
            [](int, int) { return -0.3f; }); // sae: 0.3 m up
  shift::core::Context raised;
  LoadWheel(raised, "raised", Terrain(file, ""),
            R"({"name": "terrain", "T": [-2.0, -1.0, 0]})", -0.645, 1.5);
  raised.Simulate();
  const ksl_vec3_t on_terrain = WheelCenter(raised);
  std::remove((testing::TempDir() + "/" + file).c_str());

  EXPECT_LT(on_ground.z, -0.3); // resting, the tire is compressed
  EXPECT_NEAR(on_terrain.z, on_ground.z - 0.3, 1e-6);
}

/* the surfaces drawn over a terrain load in order, with their values, and
the texture variation with them */
TEST(Terrain, SurfacesLoadInOrder) {
  const std::string file = "terrain_surfaces.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 4, 4,
            [](int, int) { return 0.f; });
  shift::core::Context context;
  LoadWheel(context, "surfaces",
            Terrain(file, R"(, "texture_variation": true, "surfaces": [
              {"texture_filename": "qrc:/textures/dry_sand.jpg",
               "texture_scale": 2.0, "coverage": 0.35, "patch_size": 8.0},
              {"texture_filename": "qrc:/textures/grass.jpg",
               "normal_map_filename": "qrc:/textures/grass_normal.jpg",
               "normal_scale": 0.5, "coverage": 0.15}])"),
            R"({"name": "terrain", "T": [0.0, 0.0, 0.0]})", -0.345, 0.0);
  const auto* terrain = dynamic_cast<const shift::core::terrain::Basic*>(
    context.get_active_terrain()[0]);
  ASSERT_NE(terrain, nullptr);
  EXPECT_TRUE(terrain->get_texture_variation());
  const auto& surfaces = terrain->get_surfaces();
  ASSERT_EQ(surfaces.size(), 2u);
  EXPECT_EQ(surfaces[0]->get_texture_path(), "qrc:/textures/dry_sand.jpg");
  EXPECT_TRUE(surfaces[0]->get_normal_map_path().empty());
  EXPECT_DOUBLE_EQ(surfaces[0]->get_texture_scale(), 2.0);
  EXPECT_DOUBLE_EQ(surfaces[0]->get_coverage(), 0.35);
  EXPECT_DOUBLE_EQ(surfaces[0]->get_patch_size(), 8.0);
  EXPECT_EQ(surfaces[1]->get_normal_map_path(),
            "qrc:/textures/grass_normal.jpg");
  EXPECT_DOUBLE_EQ(surfaces[1]->get_normal_scale(), 0.5);
  EXPECT_DOUBLE_EQ(surfaces[1]->get_texture_scale(), 1.0); // the defaults
  EXPECT_DOUBLE_EQ(surfaces[1]->get_patch_size(), 10.0);
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* more surfaces than can be drawn, or a coverage out of range, do not load */
TEST(Terrain, SurfacesAreChecked) {
  const std::string file = "terrain_surfaces_checked.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 4, 4,
            [](int, int) { return 0.f; });
  const std::string sand =
    R"({"texture_filename": "qrc:/textures/dry_sand.jpg"})";
  for(const std::string& surfaces : std::vector<std::string>{
         "[" + sand + ", " + sand + ", " + sand + "]",
       R"([{"texture_filename": "qrc:/textures/dry_sand.jpg",
            "coverage": 1.5}])",
       R"([{"texture_filename": "qrc:/textures/dry_sand.jpg",
            "patch_size": 0.0}])"}) {
    shift::core::Context context;
    EXPECT_ANY_THROW(LoadWheel(context, "surfaces_checked",
                               Terrain(file, ", \"surfaces\": " + surfaces),
                               R"({"name": "terrain", "T": [0.0, 0.0, 0.0]})",
                               -0.345, 0.0))
      << surfaces;
  }
  std::remove((testing::TempDir() + "/" + file).c_str());
}
