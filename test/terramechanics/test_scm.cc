/* deformable SCM terrain (terrain::Scm): its grid of nodes, the deformed
surface, nodes that change only when a step is accepted; and the soils of a
terrain, its own and those of a soil map (terrain::SoilMap) */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <tiffio.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/soil.h"
#include "shift/core/terrain/scm.h"

/* a single channel TIFF of bits per sample, unsigned integer or float,
sample(row, column) */
static void WriteTiff(const std::string& path, int width, int height,
                      int bits, bool is_float,
                      const std::function<double(int, int)>& sample) {
  TIFF* tiff = TIFFOpen(path.c_str(), "w");
  ASSERT_NE(tiff, nullptr);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, bits);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT,
               is_float ? SAMPLEFORMAT_IEEEFP : SAMPLEFORMAT_UINT);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, 1);
  std::vector<unsigned char> row(size_t(width) * bits / 8);
  for(int r = 0; r < height; r++) {
    for(int c = 0; c < width; c++) {
      const double value = sample(r, c);
      unsigned char* p = row.data() + size_t(c) * bits / 8;
      if(is_float) {
        const float f = float(value);
        std::memcpy(p, &f, 4);
      } else if(bits == 8) {
        *p = static_cast<unsigned char>(value);
      } else {
        const uint16_t u = static_cast<uint16_t>(value);
        std::memcpy(p, &u, 2);
      }
    }
    TIFFWriteScanline(tiff, row.data(), r, 0);
  }
  TIFFClose(tiff);
}

/* a model with one terrain, on the frame "terrain" of the fixed body */
static void LoadTerrain(shift::core::Context& context, const std::string& name,
                        const std::string& terrain, const std::string& frame) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  std::ostringstream model;
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")" << testing::TempDir() << R"("],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]
  },
  "frames": [)" << frame << R"(],
  "terrain": [)" << terrain << R"(],
  "assemblies": [{"name": "a",
    "bodies": [{"name": "block", "center_of_mass": [0, 0, 0],
                "inertia": {"mass": 1.0, "Ixx": 1, "Iyy": 1, "Izz": 1,
                            "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["block"], "origin"], "axis": [0, 0, 1]}]
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

/* a terrain frame turned 30 degrees about z and moved */
static const char* kTerrainFrame =
  R"({"name": "terrain", "T": [-3.0, -1.0, 0.2],
      "R": [[0.8660254037844387, -0.5, 0.0], [0.5, 0.8660254037844387, 0.0],
            [0.0, 0.0, 1.0]]})";
static const char* kLevelFrame = R"({"name": "terrain", "T": [0, 0, 0]})";

/* a point of the terrain frame of kTerrainFrame in the inertial frame */
static ksl_vec3_t Inertial(double x, double y, double z) {
  const double c = std::sqrt(3.0) / 2.0, s = 0.5;
  return {{-3.0 + c * x - s * y, -1.0 + s * x + c * y, 0.2 + z}};
}

static const char* kGrid =
  R"({"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
      "x": [-1, 1], "y": [-0.5, 0.5], "spacing": 0.1,
      "soil": {"name": "sandy_loam"}})";

static shift::core::terrain::Scm* GetScm(shift::core::Context& context) {
  if(context.get_active_terrain().size() != 1u) {
    ADD_FAILURE() << "no terrain";
    return nullptr;
  }
  return dynamic_cast<shift::core::terrain::Scm*>(
    context.get_active_terrain()[0]);
}

/* without an elevation map the grid lies in the xy plane of the terrain
frame; its nodes are spacing apart from the start of each range */
TEST(Scm, FlatGrid) {
  shift::core::Context context;
  LoadTerrain(context, "scm_flat", kGrid, kTerrainFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  EXPECT_EQ(scm->get_node_count_x(), 21);
  EXPECT_EQ(scm->get_node_count_y(), 11);
  EXPECT_DOUBLE_EQ(scm->NodeX(20), 1.0);
  EXPECT_DOUBLE_EQ(scm->NodeY(0), -0.5);
  long i, j;
  ASSERT_TRUE(scm->NearestNode(0.04, 0.26, i, j));
  EXPECT_EQ(i, 10);
  EXPECT_EQ(j, 8);
  EXPECT_TRUE(scm->NearestNode(1.04, 0.0, i, j));
  EXPECT_FALSE(scm->NearestNode(1.06, 0.0, i, j));
  EXPECT_FALSE(scm->NearestNode(0.0, -0.56, i, j));
  /* the surface and its normal, into the terrain (+z of the frame) */
  ksl_vec3_t surface, normal;
  const ksl_vec3_t above = Inertial(0.37, -0.21, -2.0);
  ASSERT_TRUE(scm->Query(above, surface, normal));
  const ksl_vec3_t expected = Inertial(0.37, -0.21, 0.0);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(surface.at[k], expected.at[k], 1e-12);
  }
  EXPECT_NEAR(normal.z, 1.0, 1e-12);
  /* beyond the grid the terrain goes on, undisturbed */
  ASSERT_TRUE(scm->Query(Inertial(5.0, 4.0, -1.0), surface, normal));
  EXPECT_NEAR(surface.z, 0.2, 1e-12);
  EXPECT_EQ(scm->get_changed_node_count(), 0u);
}

/* a range that is not a whole number of spacings ends at the last node
inside it */
TEST(Scm, PartialRange) {
  shift::core::Context context;
  LoadTerrain(context, "scm_partial",
              R"({"type": "scm", "name": "site",
                  "parent": [["fixed"], "terrain"], "x": [0, 1],
                  "y": [0, 0.3], "spacing": 0.3,
                  "soil": {"name": "sandy_loam"}})",
              kLevelFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  EXPECT_EQ(scm->get_node_count_x(), 4); // 0, 0.3, 0.6, 0.9
  EXPECT_EQ(scm->get_node_count_y(), 2); // 0, 0.3
}

/* a trial changes nothing until the step is accepted; then the surface is
lowered at the node, bilinearly between it and its neighbours */
TEST(Scm, TrialsTakeEffectWhenAccepted) {
  shift::core::Context context;
  LoadTerrain(context, "scm_trials", kGrid, kTerrainFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  const double x = scm->NodeX(10), y = scm->NodeY(5); // (0, 0)
  scm->Trial(10, 5).level = 0.04;
  scm->Trial(10, 5).plastic_sinkage = 0.05;
  double z_x, z_y;
  EXPECT_EQ(scm->SurfaceZ(x, y, z_x, z_y), 0.0);
  EXPECT_EQ(scm->FindNode(10, 5), nullptr);
  ASSERT_NE(scm->FindTrial(10, 5), nullptr);
  const long revision = scm->get_revision();
  context.AcceptStep({});
  EXPECT_EQ(scm->get_revision(), revision + 1);
  ASSERT_NE(scm->FindNode(10, 5), nullptr);
  EXPECT_EQ(scm->FindNode(10, 5)->level, 0.04);
  EXPECT_EQ(scm->FindNode(10, 5)->plastic_sinkage, 0.05);
  EXPECT_EQ(scm->FindTrial(10, 5), nullptr);
  EXPECT_NEAR(scm->NodeZ(10, 5), 0.04, 1e-15);
  EXPECT_NEAR(scm->SurfaceZ(x, y, z_x, z_y), 0.04, 1e-15);
  /* halfway to the next node along x: half the level, sloping up to it */
  EXPECT_NEAR(scm->SurfaceZ(x + 0.05, y, z_x, z_y), 0.02, 1e-15);
  EXPECT_NEAR(z_x, -0.4, 1e-12);
  /* Query sees the rut, through the terrain frame, inside the cell (on a
  line of the grid the surface has a kink): a quarter of a spacing along y,
  0.04 x 0.5 x 0.75, sloping up along x and y */
  ksl_vec3_t surface, normal;
  ASSERT_TRUE(scm->Query(Inertial(x + 0.05, y + 0.025, -1.0), surface,
                         normal));
  const ksl_vec3_t expected = Inertial(x + 0.05, y + 0.025, 0.015);
  for(int k = 0; k < 3; k++) {
    EXPECT_NEAR(surface.at[k], expected.at[k], 1e-12);
  }
  /* into the terrain: (-z_x, -z_y, 1) = (0.3, 0.2, 1), turned with the
  frame */
  const double length = std::sqrt(0.09 + 0.04 + 1.0);
  const double c = std::sqrt(3.0) / 2.0, s = 0.5;
  EXPECT_NEAR(normal.x, (c * 0.3 - s * 0.2) / length, 1e-12);
  EXPECT_NEAR(normal.y, (s * 0.3 + c * 0.2) / length, 1e-12);
  EXPECT_NEAR(normal.z, 1.0 / length, 1e-12);

  /* a trial starts from the accepted node */
  EXPECT_EQ(scm->Trial(10, 5).level, 0.04);
  /* trials dropped at the start of an evaluation are never accepted */
  scm->Trial(3, 3).level = 1.0;
  scm->ClearTrials();
  context.AcceptStep({});
  EXPECT_EQ(scm->FindNode(3, 3), nullptr);
  EXPECT_EQ(scm->get_revision(), revision + 1);
  /* a node back to undisturbed is no longer stored */
  scm->Trial(10, 5) = shift::core::terrain::Scm::Node();
  context.AcceptStep({});
  EXPECT_EQ(scm->FindNode(10, 5), nullptr);
  EXPECT_EQ(scm->get_changed_node_count(), 0u);
  EXPECT_EQ(scm->SurfaceZ(x, y, z_x, z_y), 0.0);
  EXPECT_THROW(scm->Trial(21, 0), shift::core::Exception);
  EXPECT_THROW(scm->Trial(0, -1), shift::core::Exception);
}

/* the surface is continuous at the edge of the grid: a lowered corner node
runs out to the undisturbed surface one spacing beyond it */
TEST(Scm, EdgeOfTheGrid) {
  shift::core::Context context;
  LoadTerrain(context, "scm_edge", kGrid, kLevelFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  scm->Trial(0, 0).level = 0.05;
  context.AcceptStep({});
  double z_x, z_y;
  EXPECT_NEAR(scm->SurfaceZ(-1.0, -0.5, z_x, z_y), 0.05, 1e-15);
  EXPECT_NEAR(scm->SurfaceZ(-1.05, -0.5, z_x, z_y), 0.025, 1e-15);
  EXPECT_NEAR(z_x, 0.5, 1e-12);
  EXPECT_NEAR(scm->SurfaceZ(-1.0, -0.55, z_x, z_y), 0.025, 1e-15);
  EXPECT_NEAR(scm->SurfaceZ(-1.1, -0.5, z_x, z_y), 0.0, 1e-15);
  EXPECT_EQ(scm->SurfaceZ(-1.2, -0.5, z_x, z_y), 0.0);
}

/* on an elevation map, the nodes start on the map's surface */
TEST(Scm, ElevationMap) {
  const double a = 0.05, b = -0.08, c = 0.1; // z = a x + b y + c
  const std::string file = "scm_plane.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 20, 16, 32, true,
            [&](int row, int column) {
              return a * 0.5 * column + b * 0.25 * row + c;
            });
  shift::core::Context context;
  LoadTerrain(context, "scm_map",
              R"({"type": "scm", "name": "site", "filename": ")" + file +
                R"(", "parent": [["fixed"], "terrain"], "x_scale": 0.5,
                   "y_scale": 0.25, "x": [1, 3], "y": [0.5, 1.5],
                   "spacing": 0.25, "soil": {"name": "sandy_loam"}})",
              kLevelFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  for(long i : {0L, 3L, 8L}) {
    for(long j : {0L, 2L, 4L}) {
      const double plane = a * scm->NodeX(i) + b * scm->NodeY(j) + c;
      EXPECT_NEAR(scm->UndisturbedZ(i, j), plane, 1e-6);
    }
  }
  scm->Trial(3, 2).level = 0.01;
  context.AcceptStep({});
  const double plane = a * scm->NodeX(3) + b * scm->NodeY(2) + c;
  EXPECT_NEAR(scm->NodeZ(3, 2), plane + 0.01, 1e-6);
  double z_x, z_y;
  EXPECT_NEAR(scm->SurfaceZ(scm->NodeX(3), scm->NodeY(2), z_x, z_y),
              plane + 0.01, 1e-6);
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* what a terrain needs: an scm terrain a soil, and well formed ranges; a
basic terrain an elevation map */
TEST(Scm, Checks) {
  auto load = [](const std::string& terrain) {
    shift::core::Context context;
    LoadTerrain(context, "scm_checks", terrain, kLevelFrame);
  };
  EXPECT_THROW(load(R"({"type": "scm", "name": "s", "parent": [["fixed"],
                        "terrain"], "x": [0, 1], "y": [0, 1],
                        "spacing": 0.1})"),
               shift::core::Exception);
  EXPECT_THROW(load(R"({"type": "scm", "name": "s", "parent": [["fixed"],
                        "terrain"], "x": [1, 0], "y": [0, 1], "spacing": 0.1,
                        "soil": {"name": "sandy_loam"}})"),
               shift::core::Exception);
  EXPECT_THROW(load(R"({"type": "scm", "name": "s", "parent": [["fixed"],
                        "terrain"], "x": [0, 1], "y": [0, 1], "spacing": 0,
                        "soil": {"name": "sandy_loam"}})"),
               shift::core::Exception);
  EXPECT_THROW(load(R"({"type": "basic", "name": "b", "parent": [["fixed"],
                        "terrain"]})"),
               shift::core::Exception);
}

/* soils:
  row 0:  1  1  2  2
  row 1:  1  0  2  2
  row 2:  3  3  3  3
sae: columns along x (0.5 m apart), rows along y (0.25 m apart) */
static const int kSoilValues[3][4] = {{1, 1, 2, 2}, {1, 0, 2, 2}, {3, 3, 3, 3}};

static std::string SoilTerrain(const std::string& map,
                               const std::string& extra,
                               const std::string& soils) {
  return R"({"type": "scm", "name": "site", "parent": [["fixed"], "terrain"],
             "x": [0, 2], "y": [0, 1], "spacing": 0.25)" + extra +
         R"(, "soil_map": {"filename": ")" + map +
         R"(", "x_scale": 0.5, "y_scale": 0.25, "soils": [)" + soils + "]}}";
}

static const char* kSoils =
  R"({"value": 1, "name": "sandy_loam"},
     {"value": 2, "name": "clayey_soil_wet", "shear_modulus": 0.02},
     {"value": 3, "sinkage_exponent": 0.9, "cohesive_modulus": 3e4,
      "frictional_modulus": 1.2e6, "cohesion": 2500, "friction_angle": 0.45,
      "shear_modulus": 0.012, "unit_weight": 17500})";

static shift::core::force_module::SoilProperties Table(const char* name) {
  shift::core::force_module::SoilProperties soil;
  EXPECT_TRUE(shift::core::force_module::LookupSoil(name, soil)) << name;
  return soil;
}

/* the soil of the nearest sample, 0 the terrain's own */
TEST(SoilMap, NearestSample) {
  const std::string file = "soil_map.tiff";
  for(int bits : {8, 16}) {
    WriteTiff(testing::TempDir() + "/" + file, 4, 3, bits, false,
              [](int row, int column) { return kSoilValues[row][column]; });
    shift::core::Context context;
    LoadTerrain(context, "soil_map",
                SoilTerrain(file, R"(, "soil": {"name": "dry_sand"})", kSoils),
                kTerrainFrame);
    auto* scm = GetScm(context);
    ASSERT_NE(scm, nullptr);
    ASSERT_EQ(scm->get_soils().size(), 4u); // the terrain's, then the map's
    auto k_phi = [&](double x, double y) {
      const auto* soil = scm->LocalSoil(x, y);
      return soil ? soil->k_phi : -1.0;
    };
    EXPECT_EQ(k_phi(0.0, 0.0), Table("sandy_loam").k_phi);
    EXPECT_EQ(k_phi(0.74, 0.0), Table("sandy_loam").k_phi); // column 1
    EXPECT_EQ(k_phi(0.76, 0.0), Table("clayey_soil_wet").k_phi);
    EXPECT_EQ(k_phi(0.5, 0.25), Table("dry_sand").k_phi); // value 0
    EXPECT_EQ(k_phi(0.5, 0.37), Table("dry_sand").k_phi); // row 1
    EXPECT_EQ(k_phi(0.5, 0.38), 1.2e6);                   // row 2
    /* beyond the map, the nearest sample of its edge */
    EXPECT_EQ(k_phi(-3.0, 9.0), 1.2e6);
    EXPECT_EQ(k_phi(9.0, -3.0), Table("clayey_soil_wet").k_phi);
    /* overrides of a table soil */
    EXPECT_EQ(scm->LocalSoil(1.5, 0.0)->shear_modulus, 0.02);
    /* through the terrain frame, and at the nodes */
    const auto* soil = scm->SoilAt(Inertial(0.5, 0.25, -4.0));
    ASSERT_NE(soil, nullptr);
    EXPECT_EQ(soil->k_phi, Table("dry_sand").k_phi);
    EXPECT_EQ(scm->NodeSoil(2, 1)->k_phi, Table("dry_sand").k_phi);
    EXPECT_EQ(scm->NodeSoil(8, 4)->k_phi, 1.2e6);
  }
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* iso: image rows along x, columns along y */
TEST(SoilMap, Iso) {
  const std::string file = "soil_map_iso.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 4, 3, 8, false,
            [](int row, int column) { return kSoilValues[row][column]; });
  shift::core::Context context;
  LoadTerrain(context, "soil_map_iso",
              SoilTerrain(file,
                          R"(, "soil": {"name": "dry_sand"},
                              "coordinate_type": "iso")",
                          kSoils),
              kLevelFrame);
  auto* scm = GetScm(context);
  ASSERT_NE(scm, nullptr);
  /* row 2 at x = 1.0, column 1 at y = 0.25 */
  EXPECT_EQ(scm->LocalSoil(1.0, 0.25)->k_phi, 1.2e6);
  /* row 1 at x = 0.5, column 1 at y = 0.25: value 0 */
  EXPECT_EQ(scm->LocalSoil(0.5, 0.25)->k_phi, Table("dry_sand").k_phi);
  std::remove((testing::TempDir() + "/" + file).c_str());
}

/* the map must give every one of its values a soil, as integers */
TEST(SoilMap, Checks) {
  const std::string file = "soil_map_checks.tiff";
  const std::string path = testing::TempDir() + "/" + file;
  auto message = [&](const std::string& terrain) {
    try {
      shift::core::Context context;
      LoadTerrain(context, "soil_map_checks", terrain, kLevelFrame);
    } catch(const shift::core::Exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  /* values 4 and 5 have no soil */
  WriteTiff(path, 4, 3, 8, false, [](int row, int column) {
    return row == 0 && column == 0 ? 5 : (row == 2 ? 4 : 1);
  });
  std::string error = message(SoilTerrain(file, "", kSoils));
  EXPECT_NE(error.find("4, 5"), std::string::npos) << error;
  /* 0 without a soil of the terrain */
  WriteTiff(path, 4, 3, 8, false,
            [](int row, int column) { return kSoilValues[row][column]; });
  error = message(SoilTerrain(file, "", kSoils));
  EXPECT_NE(error.find("values with no soil: 0"), std::string::npos) << error;
  /* two soils of one value */
  error = message(SoilTerrain(file, R"(, "soil": {"name": "dry_sand"})",
                              std::string(kSoils) +
                                R"(, {"value": 2, "name": "snow"})"));
  EXPECT_NE(error.find("two soils have the value 2"), std::string::npos)
    << error;
  /* a float map */
  WriteTiff(path, 4, 3, 32, true, [](int, int) { return 1.0; });
  error = message(SoilTerrain(file, R"(, "soil": {"name": "dry_sand"})",
                              kSoils));
  EXPECT_NE(error.find("integer"), std::string::npos) << error;
  std::remove(path.c_str());
}

/* a basic terrain's soil, and none without one */
TEST(SoilMap, BasicTerrainSoil) {
  const std::string file = "soil_basic.tiff";
  WriteTiff(testing::TempDir() + "/" + file, 4, 4, 32, true,
            [](int, int) { return 0.0; });
  const std::string terrain =
    R"({"type": "basic", "name": "b", "filename": ")" + file +
    R"(", "parent": [["fixed"], "terrain"])";
  {
    shift::core::Context context;
    LoadTerrain(context, "soil_basic", terrain + "}", kLevelFrame);
    EXPECT_EQ(context.get_active_terrain()[0]->SoilAt({{1.0, 1.0, 0.0}}),
              nullptr);
  }
  {
    shift::core::Context context;
    LoadTerrain(context, "soil_basic",
                terrain + R"(, "soil": {"name": "snow"}})", kLevelFrame);
    const auto* soil =
      context.get_active_terrain()[0]->SoilAt({{1.0, 1.0, 0.0}});
    ASSERT_NE(soil, nullptr);
    EXPECT_EQ(soil->k_phi, Table("snow").k_phi);
  }
  std::remove((testing::TempDir() + "/" + file).c_str());
}
