/* The map controller (controller::Map): it reads an output of another
component and sets its targets to the value of a curve at it */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/controller/map.h"
#include "shift/core/light/base.h"

/* one body on a float joint with two point lights, a schedule "pedal" that goes
from 0 to 1 over a second (setting the light \"spare\"), and the maps of the argument */
static std::string Model(const std::string& maps,
                         const std::string& assemblies = "") {
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "car", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 30, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                "Iyz": 0, "Izx": 0},
    "lights": [{"name": "lamp", "type": "point", "parent": ["car", "origin"],
                "Ka": [0, 0, 0], "Kd": [1, 0, 0], "Ks": [1, 0, 0],
                "intensity": 0.0},
               {"name": "spare", "type": "point", "parent": ["car", "origin"],
                "Ka": [0, 0, 0], "Kd": [1, 0, 0], "Ks": [1, 0, 0]}]}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
    "child": [["car"], "origin"],
    "initial_displacement": {"T": [0, 0, 0],
                             "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}}],
  "controllers": [{"type": "schedule", "name": "pedal",
                   "points": [0, 0, 1, 1],
                   "targets": ["bodies/car/lights/spare/intensity"]})" +
         maps + "]}" + assemblies + "]}";
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/map_test.json";
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

static std::string Map(const std::string& extra,
                       const std::string& component = "pedal",
                       const std::string& output = "value") {
  return R"(, {"type": "map", "name": "m", "component": ")" + component +
         R"(", "output": ")" + output +
         R"(", "targets": ["bodies/car/lights/lamp/intensity"])" + extra + "}";
}

static std::string LoadError(const std::string& maps) {
  shift::core::Context context;
  try {
    Load(context, Model(maps));
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

static shift::core::controller::Map* FindMap(shift::core::Context& context) {
  return dynamic_cast<shift::core::controller::Map*>(
    context["assemblies"]["a"]["controllers"]["m"]
      .get_data<shift::core::controller::Base>());
}

static const shift::core::light::Base* Lamp(shift::core::Context& context) {
  return context["assemblies"]["a"]["bodies"]["car"]["lights"]["lamp"]
    .get_data<shift::core::light::Base>();
}

/* evaluates the forces of the model at each time, calling check(t) after */
template <typename Check>
static void Evaluate(shift::core::Context& context,
                     const std::vector<double>& times, Check check) {
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  for(double t : times) {
    SCOPED_TRACE(t);
    context.SetForces(t, y, dy);
    check(t);
  }
}

TEST(Map, SetsALightFromAScheduleThroughItsCurve) {
  shift::core::Context context;
  // full brightness 8 once the pedal passes 0.05, as the brake lamps of the
  // polo_r_wrc example
  Load(context, Model(Map(R"(, "points": [0, 0, 0.05, 8])")));
  shift::core::controller::Map* map = FindMap(context);
  ASSERT_NE(map, nullptr);
  ASSERT_NE(Lamp(context), nullptr);
  EXPECT_DOUBLE_EQ(map->Curve(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(map->Curve(0.025), 4.0);
  EXPECT_DOUBLE_EQ(map->Curve(0.5), 8.0);
  Evaluate(context, {0.0, 0.01, 0.025, 0.5, 2.0}, [&](double t) {
    const double pedal = std::min(t, 1.0);
    EXPECT_DOUBLE_EQ(map->get_input(), pedal);
    EXPECT_DOUBLE_EQ(Lamp(context)->get_intensity(), map->Curve(pedal));
    EXPECT_DOUBLE_EQ(
      context["assemblies"]["a"]["controllers"]["m"]["value"].get<double>(),
      map->Curve(pedal));
  });
}

TEST(Map, WithoutPointsPassesTheOutputOn) {
  shift::core::Context context;
  Load(context, Model(Map("")));
  Evaluate(context, {0.0, 0.3, 0.7}, [&](double t) {
    EXPECT_DOUBLE_EQ(Lamp(context)->get_intensity(), t);
  });
}

TEST(Map, FindsAComponentOfAnotherAssemblyByItsFullyQualifiedName) {
  // the pedals of assembly "b" and of "inner", nested in it, go up twice and
  // three times as fast as the one of "a"
  const std::string others = R"(,
   {"name": "b",
    "controllers": [{"type": "schedule", "name": "pedal",
                     "points": [0, 0, 1, 2],
                     "targets": ["bodies/car/lights/spare/intensity"]}],
    "bodies": [{"name": "car", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                  "Iyz": 0, "Izx": 0},
      "lights": [{"name": "spare", "type": "point",
                  "parent": ["car", "origin"]}]}],
    "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
      "child": [["car"], "origin"]}],
    "assemblies": [{"name": "inner",
      "controllers": [{"type": "schedule", "name": "pedal",
                       "points": [0, 0, 1, 3],
                       "targets": ["bodies/car/lights/spare/intensity"]}],
      "bodies": [{"name": "car", "center_of_mass": [0, 0, 0],
        "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                    "Iyz": 0, "Izx": 0},
        "lights": [{"name": "spare", "type": "point",
                    "parent": ["car", "origin"]}]}],
      "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
        "child": [["car"], "origin"]}]}]}]})";
  for(const auto& [component, rate] :
      std::vector<std::pair<std::string, double>>{
        {"pedal", 1.0}, {"b__pedal", 2.0}, {"controllers/b__pedal", 2.0},
        {"a__pedal", 1.0}, {"b__inner__pedal", 3.0}}) {
    SCOPED_TRACE(component);
    shift::core::Context context;
    Load(context, Model(Map("", component), others));
    Evaluate(context, {0.0, 0.25}, [&](double t) {
      EXPECT_DOUBLE_EQ(Lamp(context)->get_intensity(), rate * t);
    });
  }
}

TEST(Map, RefusesWhatItCannotRead) {
  auto fails = [&](const std::string& maps, const std::string& message) {
    const std::string error = LoadError(maps);
    EXPECT_NE(error.find(message), std::string::npos)
      << "wanted '" << message << "' in '" << error << "'";
  };
  fails(Map("", "nothing"), "'nothing'");
  fails(Map("", "c__pedal"), "fully qualified name");
  fails(Map("", "a__nothing"), "fully qualified name");
  fails(Map("", "pedal", "nothing"), "nothing");
  fails(Map("", "m"), "its own output");
  fails(Map(R"(, "points": [0, 0, 1])"), "pairs");
  fails(Map(R"(, "points": [1, 0, 0, 1])"), "must increase");
}
