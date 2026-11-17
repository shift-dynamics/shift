/* the surroundings of a model as the viewer and cameras see them, at the
root of the model file: "environment" (sky, sun, fog and water) and
"surface_materials" */

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/environment/environment.h"
#include "shift/core/environment/surface_material.h"
#include "shift/core/exception.h"

/* a pendulum, with root holding more keys of the root of the model file,
each followed by a comma */
static void Load(shift::core::Context& context, const std::string& name,
                 const std::string& root) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]
  },
  )" << root << R"(
  "assemblies": [{"name": "a",
    "bodies": [{"name": "bob", "center_of_mass": [0.0, 0.0, 1.0],
                "inertia": {"mass": 1.0, "Ixx": 0.1, "Iyy": 0.1, "Izz": 0.1,
                            "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}],
    "joints": [{"name": "pin", "type": "revolute",
                "parent": [["fixed"], "origin"], "child": [["bob"], "origin"],
                "initial_displacement": 0.0}]}]
})";
  }
  try {
    context.LoadJSON(path);
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  std::remove(path.c_str());
}

/* the message of the exception loading root throws, empty if none */
static std::string Error(const std::string& root) {
  try {
    shift::core::Context context;
    Load(context, "environment_error", root);
  } catch(const shift::core::Exception& e) {
    return e.what();
  }
  return "";
}

TEST(Environment, NoneGiven) {
  shift::core::Context context;
  Load(context, "environment_none", "");
  EXPECT_EQ(context.get_environment(), nullptr);
  EXPECT_TRUE(context.get_surface_materials().empty());
}

TEST(Environment, Values) {
  shift::core::Context context;
  Load(context, "environment_values", R"("environment": {
    "sky": {"type": "physical", "turbidity": 4.0},
    "sun": {"azimuth": 90.0, "elevation": 30.0},
    "fog": {"density": 0.01},
    "water": {"height": -0.5, "wave_height": 0.4, "wave_direction": 45.0}},)");
  const auto* environment = context.get_environment();
  ASSERT_NE(environment, nullptr);
  EXPECT_EQ(environment->get_sky().get_type(), "physical");
  EXPECT_EQ(environment->get_sky().get_turbidity(), 4.0);
  /* toward the sun, z down: azimuth 90 is +y */
  const ksl_vec3_t d = environment->get_sun().Direction();
  EXPECT_NEAR(d.x, 0.0, 1e-12);
  EXPECT_NEAR(d.y, std::cos(M_PI / 6.0), 1e-12);
  EXPECT_NEAR(d.z, -0.5, 1e-12);
  EXPECT_EQ(environment->get_fog().get_density(), 0.01);
  EXPECT_FALSE(environment->get_fog().has_color());
  const auto* water = environment->get_water();
  ASSERT_NE(water, nullptr);
  EXPECT_EQ(water->get_height(), -0.5);
  EXPECT_EQ(water->get_wave_height(), 0.4);
  EXPECT_EQ(water->get_wave_direction(), 45.0);
  EXPECT_EQ(water->get_wavelength(), 5.0); // the default
}

/* water only when given */
TEST(Environment, NoWater) {
  shift::core::Context context;
  Load(context, "environment_no_water",
       R"("environment": {"sky": {"type": "gradient"}},)");
  ASSERT_NE(context.get_environment(), nullptr);
  EXPECT_EQ(context.get_environment()->get_water(), nullptr);
}

TEST(Environment, Checks) {
  EXPECT_NE(Error(R"("environment": {"sky": {"type": "physical",
                                             "turbidity": 12.0}},)")
              .find("turbidity"),
            std::string::npos);
  EXPECT_NE(Error(R"("environment": {"water": {"wave_height": -1.0}},)")
              .find("wave_height"),
            std::string::npos);
  EXPECT_NE(Error(R"("environment": {"fog": {"density": -1.0}},)")
              .find("density"),
            std::string::npos);
  /* a missing sky image only changes the look */
  EXPECT_EQ(Error(R"("environment": {"sky": {"type": "hdri",
                                             "filename": "missing.exr"}},)"),
            "");
}

TEST(SurfaceMaterials, Values) {
  shift::core::Context context;
  Load(context, "surface_materials", R"("surface_materials": [
    {"name": "wet_sand", "label": "sand", "roughness": 0.4,
     "texture_filename": "qrc:/textures/dry_sand.jpg"},
    {"name": "rock", "texture_filename": "missing.jpg"}],)");
  const auto materials = context.get_surface_materials();
  ASSERT_EQ(materials.size(), 2u);
  EXPECT_EQ(materials[0]->get_label(), "sand");
  EXPECT_EQ(materials[0]->get_roughness(), 0.4);
  /* built-in textures pass through; a missing file is drawn without */
  EXPECT_EQ(materials[0]->get_texture_path(), "qrc:/textures/dry_sand.jpg");
  EXPECT_EQ(materials[1]->get_label(), "rock");
  EXPECT_EQ(materials[1]->get_texture_path(), "");
}

TEST(SurfaceMaterials, Checks) {
  EXPECT_FALSE(Error(R"("surface_materials": [{"name": "a"},
                                              {"name": "a"}],)")
                 .empty());
  EXPECT_NE(Error(R"("surface_materials": [{"name": "a",
                                            "roughness": 2.0}],)")
              .find("roughness"),
            std::string::npos);
}
