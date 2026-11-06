#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/bushing.h"

static const char* kInertia =
  R"("center_of_mass": [0, 0, 0], "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1,
     "Izz": 1, "Ixy": 0, "Iyz": 0, "Izx": 0})";

/* a body on a joint, with the given keys added to the body and the assembly */
static std::string Parts(const std::string& body_keys,
                         const std::string& assembly_keys) {
  return std::string(R"("bodies": [{"name": "load", )") + kInertia + body_keys +
         R"(}], "joints": [{"name": "slide", "type": "prismatic",
             "parent": [["fixed"], "origin"], "child": [["load"], "origin"],
             "axis": [0, 0, 1]}])" +
         assembly_keys;
}

/* load a model of the assemblies; the file is removed even if that throws */
static void Load(shift::core::Context& context, const std::string& assemblies) {
  const std::string path = testing::TempDir() + "/unique_names.json";
  std::ofstream(path) << R"({"model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "rk4", "time_span": 0.01, "time_step": 0.01}]},
    "assemblies": [)" << assemblies
                      << "]}";
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

/* the message of the error loading the assemblies, empty if they load */
static std::string LoadError(const std::string& assemblies) {
  try {
    shift::core::Context context;
    Load(context, assemblies);
  } catch(const shift::core::Exception& e) {
    return e.what();
  }
  return "";
}

static const std::string kBody =
  std::string(R"({"name": "load", )") + kInertia + "}";
static const char* kFrame =
  R"({"name": "tip", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "T": [0, 0, 1]})";
static const char* kSensor =
  R"({"type": "joint", "name": "s", "joint": "slide", "quantity": "position"})";

TEST(UniqueNames, DuplicateBody) {
  const std::string error = LoadError(
    R"({"name": "a", "bodies": [)" + kBody + ", " + kBody + "]}");
  EXPECT_NE(error.find("Body name 'a__load' is not unique"), std::string::npos)
    << error;
}

TEST(UniqueNames, DuplicateJoint) {
  const std::string joint =
    R"({"name": "slide", "type": "prismatic", "parent": [["fixed"], "origin"],
        "child": [["load"], "origin"], "axis": [0, 0, 1]})";
  const std::string error =
    LoadError(R"({"name": "a", "bodies": [)" + kBody + R"(], "joints": [)" +
              joint + ", " + joint + "]}");
  EXPECT_NE(error.find("Joint name 'a__slide' is not unique"),
            std::string::npos)
    << error;
}

TEST(UniqueNames, DuplicateSensor) {
  const std::string error =
    LoadError(R"({"name": "a", )" +
              Parts("", std::string(R"(, "sensors": [)") + kSensor + ", " +
                          kSensor + "]") +
              "}");
  EXPECT_NE(error.find("Sensor name 'a__s' is not unique"), std::string::npos)
    << error;
}

TEST(UniqueNames, DuplicateFrameOfABody) {
  const std::string error =
    LoadError(R"({"name": "a", )" +
              Parts(std::string(R"(, "frames": [)") + kFrame + ", " + kFrame +
                      "]",
                    "") +
              "}");
  EXPECT_NE(error.find("Frame name 'a__load__tip' is not unique"),
            std::string::npos)
    << error;
}

TEST(UniqueNames, DuplicateAssembly) {
  const std::string assembly = R"({"name": "a"})";
  const std::string error = LoadError(assembly + ", " + assembly);
  EXPECT_NE(error.find("Assembly name 'a' is not unique"), std::string::npos)
    << error;
}

TEST(UniqueNames, DuplicateNestedAssembly) {
  const std::string error = LoadError(
    R"({"name": "a", "assemblies": [{"name": "b"}, {"name": "b"}]})");
  EXPECT_NE(error.find("Assembly name 'a__b' is not unique"),
            std::string::npos)
    << error;
}

/* a repeated name is an error even in an assembly that is not active */
TEST(UniqueNames, InactiveObjectsAreChecked) {
  const std::string error =
    LoadError(R"({"name": "a", "active": false, "bodies": [)" + kBody + ", " +
              kBody + "]}");
  EXPECT_NE(error.find("Body name 'a__load' is not unique"), std::string::npos)
    << error;
}

/* the same frame name on two bodies, and the same names in an assembly and in
the assembly nested in it, or named as it is, are different objects */
TEST(UniqueNames, OtherLevelsMayRepeatAName) {
  const std::string frames = std::string(R"(, "frames": [)") + kFrame + "]";
  EXPECT_EQ(LoadError(R"({"name": "a", )" + Parts(frames, "") +
                      R"(, "assemblies": [{"name": "b", )" + Parts(frames, "") +
                      R"(}, {"name": "a", )" + Parts(frames, "") + "}]}"),
            "");
}

/* a subassembly with a body load on a slide at speed v, with force_modules */
static std::string Sub(const std::string& name, double v,
                       const std::string& force_modules = "") {
  return R"({"name": ")" + name + R"(", "bodies": [)" + kBody +
         R"(], "joints": [{"name": "slide", "type": "prismatic",
           "parent": [["fixed"], "origin"], "child": [["load"], "origin"],
           "axis": [0, 0, 1], "initial_velocity": )" +
         std::to_string(v) + R"(}], "force_modules": [)" + force_modules + "]}";
}

/* a bushing between the fixed origin and the origin of body */
static std::string Bushing(const std::string& body) {
  std::string matrix;
  for(int i = 0; i < 36; i++) {
    matrix += (i ? ", " : "") + std::string(i % 7 == 0 ? "1.0" : "0.0");
  }
  return R"({"type": "bushing", "name": "b",
             "frames": [[")" + body + R"(", "origin"], ["fixed", "origin"]],
             "stiffness": [)" + matrix + R"(], "damping": [)" + matrix + "]}";
}

/* the speed the bushing of that fully qualified name measures: a bushing is at
rest at the start, so its displacement does not tell the bodies apart */
static double BushingSpeed(shift::core::Context& context,
                                  const std::string& name) {
  for(auto module : context.get_active_force_modules()) {
    if(module->FullyQualifiedName(module->get_name()) == name) {
      auto bushing = dynamic_cast<shift::core::force_module::Bushing*>(module);
      double largest = 0.0;
      for(double x : bushing->get_velocities()) {
        largest = std::max(largest, std::fabs(x));
      }
      return largest;
    }
  }
  ADD_FAILURE() << name << " not found";
  return 0.0;
}

/* each bushing names body load: it is the one of its own assembly */
TEST(FindActiveBody, OwnAssemblyFirst) {
  shift::core::Context context;
  Load(context, R"({"name": "m", "assemblies": [)" +
                  Sub("left", 1.0, Bushing("load")) + ", " +
                  Sub("right", 3.0, Bushing("load")) + "]}");
  context.Simulate();
  EXPECT_NEAR(BushingSpeed(context, "m__left__b"), 1.0, 0.1);
  EXPECT_NEAR(BushingSpeed(context, "m__right__b"), 3.0, 0.1);
}

/* a fully qualified name reaches a body of another assembly */
TEST(FindActiveBody, FullyQualifiedNameReachesAnotherAssembly) {
  shift::core::Context context;
  Load(context, R"({"name": "m", "assemblies": [)" +
                  Sub("left", 1.0, Bushing("m__right__load")) + ", " +
                  Sub("right", 3.0) + "]}");
  context.Simulate();
  EXPECT_NEAR(BushingSpeed(context, "m__left__b"), 3.0, 0.1);
}

/* the name is in two other assemblies and not in the module's own */
TEST(FindActiveBody, AmbiguousNameIsAnError) {
  const std::string error = LoadError(
    R"({"name": "m", "force_modules": [)" + Bushing("load") +
    R"(], "assemblies": [)" + Sub("left", 1.0) + ", " + Sub("right", 3.0) +
    "]}");
  EXPECT_NE(error.find("Body name 'load' is ambiguous"), std::string::npos)
    << error;
  EXPECT_NE(error.find("'m__left__load'"), std::string::npos) << error;
  EXPECT_NE(error.find("'m__right__load'"), std::string::npos) << error;
}

/* a name found once in the model needs no qualification */
TEST(FindActiveBody, UniqueNameIsFoundAnywhere) {
  shift::core::Context context;
  Load(context, R"({"name": "m", "force_modules": [)" + Bushing("load") +
                  R"(], "assemblies": [)" + Sub("left", 2.0) + "]}");
  context.Simulate();
  EXPECT_NEAR(BushingSpeed(context, "m__b"), 2.0, 0.1);
}

TEST(FindActiveBody, MissingBody) {
  shift::core::Context context;
  Load(context, R"({"name": "m", "assemblies": [)" + Sub("left", 2.0) + "]}");
  EXPECT_EQ(context.FindActiveBody("nowhere"), nullptr);
  ASSERT_NE(context.FindActiveBody("load"), nullptr);
  EXPECT_EQ(context.FindActiveBody("m__left__load"),
            context.FindActiveBody("load"));
}
