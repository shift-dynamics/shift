/* Scenery strewn over a terrain (environment::Scenery): where the items are
placed, the same for a seed every time */

#include <gtest/gtest.h>
#include <tiffio.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <ksl/vec3.h>

#include "shift/core/body/fixed.h"
#include "shift/core/context.h"
#include "shift/core/environment/scenery.h"

/* a single channel TIFF of 8 bit integer or 32 bit float samples,
sample(row, column) */
static void WriteTiff(const std::string& path, int width, int height,
                      bool is_float,
                      const std::function<double(int, int)>& sample) {
  TIFF* tiff = TIFFOpen(path.c_str(), "w");
  ASSERT_NE(tiff, nullptr);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, is_float ? 32 : 8);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT,
               is_float ? SAMPLEFORMAT_IEEEFP : SAMPLEFORMAT_UINT);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, 1);
  std::vector<float> floats(width);
  std::vector<uint8_t> bytes(width);
  for(int r = 0; r < height; r++) {
    for(int c = 0; c < width; c++) {
      floats[c] = static_cast<float>(sample(r, c));
      bytes[c] = static_cast<uint8_t>(sample(r, c));
    }
    TIFFWriteScanline(tiff, is_float ? static_cast<void*>(floats.data())
                                     : static_cast<void*>(bytes.data()),
                      r, 0);
  }
  TIFFClose(tiff);
}

/* a terrain 40 m square on 1 m samples, sloping down along x by slope (z of
a sample, down, is -slope x), and a material map on 0.5 m samples: "moss"
for x < 20, "gravel" beyond */
static void WriteMaps(double slope) {
  WriteTiff(testing::TempDir() + "/scenery_elevation.tiff", 41, 41, true,
            [slope](int, int column) { return -slope * column; });
  WriteTiff(testing::TempDir() + "/scenery_materials.tiff", 81, 81, false,
            [](int, int column) { return column * 0.5 < 20.0 ? 0 : 1; });
}

static std::string Model(const std::string& scenery) {
  const std::string dir = testing::TempDir() + "/";
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "rk4", "time_span": 0.0, "time_step": 0.01}]},
 "surface_materials": [{"name": "moss"}, {"name": "gravel"}],
 "terrain": [{"type": "basic", "name": "ground",
   "filename": ")" + dir + R"(scenery_elevation.tiff",
   "x_scale": 1.0, "y_scale": 1.0, "z_scale": 1.0, "coordinate_type": "sae",
   "material_map": {"filename": ")" + dir + R"(scenery_materials.tiff",
     "x_scale": 0.5, "y_scale": 0.5,
     "materials": [{"value": 0, "material": "moss"},
                   {"value": 1, "material": "gravel"}]}}],
 "scenery": [)" + scenery + R"(],
 "assemblies": [{"name": "a",
  "bodies": [{"name": "ball", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.01, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
              "child": [["ball"], "origin"],
              "initial_displacement": {"T": [0, 0, -5],
                "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}}]}]})";
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/scenery_test.json";
  {
    std::ofstream file(path);
    file << json;
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static std::vector<std::unique_ptr<shift::core::environment::Scenery>>&
GetScenery(shift::core::Context& context) {
  return dynamic_cast<shift::core::body::Fixed*>(context.get_fixed_body())
    ->get_scenery();
}

static std::string Scenery(const std::string& name, const std::string& extra) {
  return R"({"name": ")" + name + R"(", "terrain": "ground",
             "geometry": [{"type": "sphere"}, {"type": "sphere"}],)" +
         extra + "}";
}

static double Distance(const ksl_vec3_t& a, const ksl_vec3_t& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

/* the items of a seed are the same every time; no two nearer than spacing;
on the materials given, and margin from the others */
TEST(Scenery, Placement) {
  WriteMaps(0.0);
  const std::string json = Model(Scenery(
    "bushes", R"("materials": ["moss"], "margin": 2.0, "density": 0.5,
                 "spacing": 1.5, "scale": [0.5, 2.0], "seed": 7)"));
  shift::core::Context first, second;
  Load(first, json);
  Load(second, json);
  const auto& items = GetScenery(first)[0]->get_items();
  const auto& again = GetScenery(second)[0]->get_items();
  ASSERT_GT(items.size(), 50u);
  ASSERT_EQ(items.size(), again.size());
  bool both_variants[2] = {false, false};
  for(size_t i = 0; i < items.size(); i++) {
    EXPECT_EQ(items[i].variant, again[i].variant);
    EXPECT_EQ(items[i].scale, again[i].scale);
    EXPECT_EQ(items[i].pose.t.x, again[i].pose.t.x);
    EXPECT_EQ(items[i].pose.t.y, again[i].pose.t.y);
    const ksl_vec3_t& p = items[i].pose.t;
    EXPECT_LT(p.x, 20.0 - 2.0 + 0.5) << "on the gravel or within margin";
    EXPECT_GE(p.x, 0.0);
    EXPECT_LE(p.y, 40.0);
    EXPECT_GE(items[i].scale, 0.5);
    EXPECT_LE(items[i].scale, 2.0);
    both_variants[items[i].variant] = true;
    for(size_t j = 0; j < i; j++) {
      EXPECT_GE(Distance(p, items[j].pose.t), 1.5);
    }
  }
  EXPECT_TRUE(both_variants[0] && both_variants[1]);

  /* another seed, other items */
  shift::core::Context other;
  Load(other, Model(Scenery("bushes", R"("materials": ["moss"],
                                          "density": 0.5, "seed": 8)")));
  const auto& others = GetScenery(other)[0]->get_items();
  ASSERT_FALSE(others.empty());
  EXPECT_NE(others[0].pose.t.x, items[0].pose.t.x);
}

/* the items of a scenery keep spacing from those it avoids */
TEST(Scenery, Avoid) {
  WriteMaps(0.0);
  shift::core::Context context;
  Load(context,
       Model(Scenery("trees", R"("density": 0.05, "spacing": 4, "seed": 1)") +
             ", " +
             Scenery("grass", R"("density": 2, "spacing": 3,
                                  "avoid": ["trees"], "seed": 2)")));
  const auto& scenery = GetScenery(context);
  ASSERT_EQ(scenery.size(), 2u);
  const auto& trees = scenery[0]->get_items();
  const auto& grass = scenery[1]->get_items();
  ASSERT_FALSE(trees.empty());
  ASSERT_FALSE(grass.empty());
  for(const auto& blade : grass) {
    for(const auto& tree : trees) {
      EXPECT_GE(Distance(blade.pose.t, tree.pose.t), 3.0);
    }
  }
}

/* on a slope: the origin depth below the surface, upright with tilt 0, on
the slope with tilt 1 */
TEST(Scenery, StandsOnTheSlope) {
  const double slope = 0.2;
  WriteMaps(slope);
  shift::core::Context context;
  Load(context,
       Model(Scenery("trees", R"("density": 0.02, "depth": 0.3, "seed": 3)") +
             ", " +
             Scenery("stones", R"("density": 0.02, "tilt": 1, "seed": 4)")));
  const auto& scenery = GetScenery(context);
  /* the downward normal of the surface z = -slope x */
  ksl_vec3_t normal = ksl_vec3(slope, 0.0, 1.0);
  ksl_vec3_normalize(&normal);
  ASSERT_FALSE(scenery[0]->get_items().empty());
  for(const auto& tree : scenery[0]->get_items()) {
    const ksl_vec3_t& p = tree.pose.t;
    EXPECT_NEAR(p.z, -slope * p.x + 0.3, 1e-6);
    EXPECT_NEAR(tree.pose.R.v2.z, 1.0, 1e-12); // upright
  }
  ASSERT_FALSE(scenery[1]->get_items().empty());
  for(const auto& stone : scenery[1]->get_items()) {
    EXPECT_NEAR(stone.pose.t.z, -slope * stone.pose.t.x, 1e-6);
    EXPECT_NEAR(ksl_dot_vv(&stone.pose.R.v2, &normal), 1.0, 1e-9);
  }
}

static std::string LoadError(const std::string& scenery) {
  WriteMaps(0.0);
  shift::core::Context context;
  try {
    Load(context, Model(scenery));
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(Scenery, Checks) {
  EXPECT_NE(LoadError(R"({"name": "s", "terrain": "nowhere",
                          "geometry": [{"type": "sphere"}],
                          "density": 1})")
              .find("no active terrain named"),
            std::string::npos);
  EXPECT_NE(LoadError(Scenery("s", R"("density": 1,
                                       "materials": ["sand"])"))
              .find("no surface material named"),
            std::string::npos);
  EXPECT_NE(LoadError(Scenery("s", R"("density": 1, "avoid": ["t"])") +
                      ", " + Scenery("t", R"("density": 1)"))
              .find("not a scenery given before it"),
            std::string::npos);
  EXPECT_NE(LoadError(Scenery("s", R"("density": 1, "scale": [2, 1])"))
              .find("scale"),
            std::string::npos);
  EXPECT_NE(LoadError(Scenery("s", R"("density": 1, "tilt": 2)"))
              .find("tilt"),
            std::string::npos);
}
