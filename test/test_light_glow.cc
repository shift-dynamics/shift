/* Geometries that glow with a light (light::Base "glow"): the viewer's scene
marks their meshes with the light, whose intensity at the time shown scales
their glow */

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "shift/core/context.h"
#include "shift/core/light/base.h"
#include "shift/gui/simulation.h"

/* a body with two meshes, "lens" and "housing", and a lamp that glows with
the geometries of glow */
static std::string WriteModel(const std::string& glow) {
  const std::string directory = testing::TempDir();
  for(const char* name : {"lens", "housing"}) {
    std::ofstream obj(directory + "/glow_" + name + ".obj");
    obj << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
  }
  const std::string path = directory + "/glow_test.json";
  std::ofstream file(path);
  file << R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "car", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                "Iyz": 0, "Izx": 0},
    "geometry": [
      {"type": "mesh", "name": "housing", "filename": ["glow_housing.obj"]},
      {"type": "mesh", "name": "lens", "filename": ["glow_lens.obj"]}],
    "lights": [{"name": "lamp", "type": "point", "parent": ["car", "origin"],
                "Kd": [1, 0, 0], "intensity": 0.5, "glow": )"
       << glow << R"(}]}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
    "child": [["car"], "origin"]}]}]})";
  return path;
}

TEST(LightGlow, LoadsTheNamesOfTheGeometries) {
  shift::core::Context context;
  const std::string path = WriteModel(R"(["lens"])");
  context.LoadJSON(path);
  std::remove(path.c_str());
  const auto* lamp = context["assemblies"]["a"]["bodies"]["car"]["lights"]
                       ["lamp"].get_data<shift::core::light::Base>();
  ASSERT_NE(lamp, nullptr);
  EXPECT_EQ(lamp->get_glow(), std::vector<std::string>{"lens"});
}

TEST(LightGlow, MarksOnlyTheMeshesThatGlow) {
  shift::gui::Simulation simulation;
  simulation.Load(WriteModel(R"(["lens"])"));
  const shift::gui::Scene& scene = simulation.get_scene();
  ASSERT_EQ(scene.lights.size(), 1u);
  ASSERT_EQ(scene.meshes.size(), 2u);
  // in the order of the geometries of the body
  EXPECT_EQ(scene.meshes[0].glow_light, -1);
  EXPECT_EQ(scene.meshes[1].glow_light, 0);
  EXPECT_FLOAT_EQ(scene.lights[0].intensity, 0.5f);
}

TEST(LightGlow, NothingGlowsWithoutTheKey) {
  shift::gui::Simulation simulation;
  simulation.Load(WriteModel("[]"));
  for(const auto& mesh : simulation.get_scene().meshes) {
    EXPECT_EQ(mesh.glow_light, -1);
  }
}
