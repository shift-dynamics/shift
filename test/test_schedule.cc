/* The schedule controller (controller::Schedule): interpolation between its
control points, and setting the parameters of other components that are
declared changeable at run time */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/controller/schedule.h"
#include "shift/core/controller/pid.h"
#include "shift/core/force_module/track_soil.h"

using shift::core::controller::Schedule;

/* one body on a float joint, with a track soil module whose ground and shoe
center are changeable at run time (shoe_length is not), and the controllers
of the argument */
static std::string Model(const std::string& controllers) {
  return R"({"model_parameters": {"version": "0.2",
  "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
            "time": "seconds"},
  "gravity": [0, 0, 9.81], "solvers": [{"type": "rnca"}],
  "integrators": [{"type": "ode113", "time_span": 0.0, "time_step": 0.01}]},
 "assemblies": [{"name": "a",
  "bodies": [{"name": "shoe", "center_of_mass": [0, 0, 0],
    "inertia": {"mass": 30, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                "Iyz": 0, "Izx": 0}}],
  "joints": [{"name": "j", "type": "float", "parent": [["fixed"], "origin"],
    "child": [["shoe"], "origin"],
    "initial_displacement": {"T": [0, 0, -0.1],
                             "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}}],
  "force_modules": [{"type": "track_soil", "name": "soil",
    "soil": {"name": "lete_sand"}, "shoes": ["shoe"],
    "shoe_center": [0, 0, 0], "shoe_length": 0.2, "shoe_width": 0.5}],
  "controllers": [)" +
         controllers + R"(]}]})";
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/schedule_test.json";
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

/* the schedule of a given name */
static Schedule* Find(shift::core::Context& context, const std::string& name) {
  auto base = context["assemblies"]["a"]["controllers"][name]
                .get_data<shift::core::controller::Base>();
  return dynamic_cast<Schedule*>(base);
}

static std::string Controller(const std::string& interpolation,
                              const std::string& extra = "",
                              const std::string& points =
                                "[0, 0, 1, 2, 3, 2, 4, 10]",
                              const std::string& targets =
                                "[\"force_modules/soil/ground\"]") {
  return R"({"type": "schedule", "name": "s", "interpolation": ")" +
         interpolation + R"(", "points": )" + points + R"(, "targets": )" +
         targets + extra + "}";
}

/* the message of the error that loading the model raises */
static std::string LoadError(const std::string& controllers) {
  shift::core::Context context;
  try {
    Load(context, Model(controllers));
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(Schedule, LinearInterpolation) {
  shift::core::Context context;
  Load(context, Model(Controller("linear")));
  const Schedule* s = Find(context, "s");
  ASSERT_NE(s, nullptr);
  // held before the first point and after the last
  EXPECT_EQ(s->Evaluate(-1.0), 0.0);
  EXPECT_EQ(s->Evaluate(5.0), 10.0);
  // straight between the points, through them
  EXPECT_DOUBLE_EQ(s->Evaluate(0.0), 0.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(0.5), 1.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(1.0), 2.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(2.0), 2.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(3.5), 6.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(4.0), 10.0);
}

TEST(Schedule, CubicInterpolation) {
  shift::core::Context context;
  Load(context, Model(Controller("cubic")));
  const Schedule* s = Find(context, "s");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->Evaluate(-1.0), 0.0);
  EXPECT_EQ(s->Evaluate(5.0), 10.0);
  // through the points
  EXPECT_NEAR(s->Evaluate(0.0), 0.0, 1e-12);
  EXPECT_NEAR(s->Evaluate(1.0), 2.0, 1e-12);
  EXPECT_NEAR(s->Evaluate(3.0), 2.0, 1e-12);
  EXPECT_NEAR(s->Evaluate(4.0), 10.0, 1e-12);
  // level between the two level points, and never beyond the data
  for(double t = 1.0; t <= 3.0; t += 0.25) {
    EXPECT_NEAR(s->Evaluate(t), 2.0, 1e-12) << t;
  }
  double last = 0.0;
  for(double t = 0.0; t <= 4.0; t += 0.01) {
    const double v = s->Evaluate(t);
    EXPECT_GE(v, 0.0 - 1e-12);
    EXPECT_LE(v, 10.0 + 1e-12);
    EXPECT_GE(v, last - 1e-12) << t; // the data never fall
    last = v;
  }
  // smooth where the linear interpolation has a corner: the same value on
  // either side of a point, and the same slope
  const double h = 1e-6;
  const double slope_before = (s->Evaluate(3.0) - s->Evaluate(3.0 - h)) / h;
  const double slope_after = (s->Evaluate(3.0 + h) - s->Evaluate(3.0)) / h;
  EXPECT_NEAR(slope_before, slope_after, 1e-4);
  // not the straight line: the cubic's value between points is not linear's
  EXPECT_GT(std::fabs(s->Evaluate(0.5) - 1.0), 1e-3);
}

TEST(Schedule, StepInterpolation) {
  shift::core::Context context;
  Load(context, Model(Controller("step")));
  const Schedule* s = Find(context, "s");
  ASSERT_NE(s, nullptr);
  // the value of the last point passed
  EXPECT_EQ(s->Evaluate(-1.0), 0.0);
  EXPECT_EQ(s->Evaluate(0.0), 0.0);
  EXPECT_EQ(s->Evaluate(0.99), 0.0);
  EXPECT_EQ(s->Evaluate(1.0), 2.0);
  EXPECT_EQ(s->Evaluate(2.5), 2.0);
  EXPECT_EQ(s->Evaluate(3.99), 2.0);
  EXPECT_EQ(s->Evaluate(4.0), 10.0);
  EXPECT_EQ(s->Evaluate(9.0), 10.0);
}

TEST(Schedule, StepWithATransition) {
  shift::core::Context context;
  Load(context, Model(Controller("step", R"(, "transition_time": 0.2)")));
  const Schedule* s = Find(context, "s");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->Evaluate(0.99), 0.0);
  // a smooth cubic from the old value to the new over the transition time
  EXPECT_DOUBLE_EQ(s->Evaluate(1.0), 0.0);
  EXPECT_NEAR(s->Evaluate(1.1), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(s->Evaluate(1.2), 2.0);
  EXPECT_DOUBLE_EQ(s->Evaluate(2.0), 2.0);
  const double h = 1e-7;
  EXPECT_NEAR((s->Evaluate(1.0 + h) - s->Evaluate(1.0)) / h, 0.0, 1e-4);
  EXPECT_NEAR((s->Evaluate(1.2) - s->Evaluate(1.2 - h)) / h, 0.0, 1e-4);
  // the last point too
  EXPECT_NEAR(s->Evaluate(4.1), 6.0, 1e-12);
  EXPECT_DOUBLE_EQ(s->Evaluate(4.2), 10.0);
  double last = 0.0;
  for(double t = 0.0; t <= 5.0; t += 0.01) {
    const double v = s->Evaluate(t);
    EXPECT_GE(v, last - 1e-12) << t;
    last = v;
  }
}

TEST(Schedule, DefaultsToLinear) {
  shift::core::Context context;
  Load(context,
       Model(R"({"type": "schedule", "name": "s", "points": [0, 0, 2, 4],
                 "targets": ["force_modules/soil/ground"]})"));
  EXPECT_DOUBLE_EQ(Find(context, "s")->Evaluate(0.5), 1.0);
}

/* the parameter of the force module "soil" */
static shift::core::parameter::Value& Soil(shift::core::Context& context,
                                          const std::string& key) {
  return context["assemblies"]["a"]["force_modules"]["soil"][key];
}

TEST(Schedule, SetsTheTargetsAtEachEvaluation) {
  shift::core::Context context;
  // the ground of the soil, an entry of the shoe center, both from one
  // schedule
  Load(context,
       Model(Controller("linear", "",
                        "[0, 0, 1, 2, 3, 2, 4, 10]",
                        R"(["force_modules/soil/ground",
                            "force_modules/soil/shoe_center/2"])")));
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  for(double t : {0.0, 0.5, 2.0, 3.5}) {
    SCOPED_TRACE(t);
    context.SetForces(t, y, dy);
    const double expected = Find(context, "s")->Evaluate(t);
    EXPECT_EQ(Soil(context, "ground").get<double>(), expected);
    EXPECT_EQ(Soil(context, "shoe_center").get<ksl_vec3_t>().z, expected);
    // the others are not touched
    EXPECT_EQ(Soil(context, "shoe_center").get<ksl_vec3_t>().x, 0.0);
    EXPECT_EQ(Soil(context, "shoe_center").get<ksl_vec3_t>().y, 0.0);
  }
}

TEST(Schedule, TheModuleReadsTheValueInTheSameEvaluation) {
  // the ground goes from 0 to -0.3 (z down: up, towards the shoe's -0.1, so
  // that it ends 0.2 m under the ground) at t = 1
  shift::core::Context context;
  Load(context, Model(Controller("linear", "", "[0, 0, 1, -0.3]")));
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  auto* module = dynamic_cast<shift::core::force_module::TrackSoil*>(
    context.get_active_force_modules()[0]);
  ASSERT_NE(module, nullptr);
  for(double t : {0.0, 0.5, 1.0, 2.0}) {
    SCOPED_TRACE(t);
    context.SetForces(t, y, dy);
    std::vector<double> outputs;
    for(const auto& key : module->get_output_keys()) {
      outputs.push_back(module->GetOutput(key));
    }
    const double ground = std::max(std::min(-0.3 * t, 0.0), -0.3);
    // the sinkage of the shoe: the point -0.1 m below the origin plane
    EXPECT_NEAR(outputs[0], std::max(-0.1 - ground, 0.0), 1e-12);
  }
}

TEST(Schedule, WorksBeforeTheControllersItSetsWhereverItIs) {
  // the belt example's PID controller comes first in its assembly, the
  // schedule after it; the gain at the time of the evaluation is the one
  // that the PID controller uses in it
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) +
                     "/force_modules/belt.json");
  std::stringstream text;
  text << file.rdbuf();
  std::string json = text.str();
  // the model's meshes are next to the example, not next to the copy
  const std::string include = R"("include_directories": ["."])";
  ASSERT_NE(json.find(include), std::string::npos);
  json.replace(json.find(include), include.size(),
               R"("include_directories": [")" +
                 std::string(SHIFT_EXAMPLES_DIR) + R"(/force_modules"])");
  const std::string anchor = R"("joint": "crank_bearing"})";
  const size_t at = json.find(anchor);
  ASSERT_NE(at, std::string::npos);
  json.insert(at + anchor.size(),
              R"(,
        {"type": "schedule", "name": "gain", "interpolation": "linear",
         "points": [0, 0.5, 1, 1.5],
         "targets": ["controllers/speed_control/kp"]})");
  shift::core::Context context;
  Load(context, json);
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  auto* pid = dynamic_cast<shift::core::controller::PID*>(
    context["assemblies"]["accessory_drive"]["controllers"]["speed_control"]
      .get_data<shift::core::controller::Base>());
  ASSERT_NE(pid, nullptr);
  for(double t : {0.0, 0.5, 1.0}) {
    SCOPED_TRACE(t);
    context.SetForces(t, y, dy);
    const double kp = 0.5 + t;
    ASSERT_NE(pid->get_error(), 0.0);
    // y has no integral of the error yet, so the effort is kp times it
    EXPECT_DOUBLE_EQ(pid->get_effort(), kp * pid->get_error());
  }
}

TEST(Schedule, SetsAnEntryOfAVector) {
  // the friction coefficient on each pulley of the belt example, a vector
  std::ifstream file(std::string(SHIFT_EXAMPLES_DIR) +
                     "/force_modules/belt.json");
  std::stringstream text;
  text << file.rdbuf();
  std::string json = text.str();
  // the model's meshes are next to the example, not next to the copy
  const std::string include = R"("include_directories": ["."])";
  ASSERT_NE(json.find(include), std::string::npos);
  json.replace(json.find(include), include.size(),
               R"("include_directories": [")" +
                 std::string(SHIFT_EXAMPLES_DIR) + R"(/force_modules"])");
  const std::string anchor = R"("joint": "crank_bearing"})";
  const size_t at = json.find(anchor);
  ASSERT_NE(at, std::string::npos);
  json.insert(at + anchor.size(),
              R"(,
        {"type": "schedule", "name": "mu", "interpolation": "linear",
         "points": [0, 0.5, 1, 0.9],
         "targets": ["force_modules/belt/friction_coefficient/1"]})");
  shift::core::Context context;
  Load(context, json);
  context.UpdateDHVA();
  std::vector<double> y(context.get_q());
  y.resize(y.size() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.5, y, dy);
  const auto& mu = context["assemblies"]["accessory_drive"]["force_modules"]
                          ["belt"]["friction_coefficient"]
                            .get<std::vector<double>>();
  ASSERT_EQ(mu.size(), 4u);
  EXPECT_DOUBLE_EQ(mu[1], 0.7);
  EXPECT_EQ(mu[0], 0.5);
  EXPECT_EQ(mu[2], 0.5);
  EXPECT_EQ(mu[3], 0.5);
}

TEST(Schedule, RefusesWhatItCannotSet) {
  auto fails = [&](const std::string& controller, const std::string& message) {
    const std::string error = LoadError(controller);
    EXPECT_NE(error.find(message), std::string::npos)
      << "wanted '" << message << "' in '" << error << "'";
  };
  const auto with = [&](const std::string& target) {
    return Controller("linear", "", "[0, 0, 1, 1]", "[\"" + target + "\"]");
  };
  // a parameter its component computes values from, or does not read each step
  fails(with("force_modules/soil/shoe_length"),
        "cannot be changed while the model runs");
  fails(with("force_modules/soil/soil"), "names an object");
  fails(with("controllers/s/points/0"), "cannot be changed while the model runs");
  fails(with("force_modules/nothing/ground"), "there is no 'nothing'");
  fails(with("force_modules/soil/nothing"), "there is no 'nothing'");
  fails(with("nothing/soil/ground"), "there is no 'nothing'");
  fails(with("force_modules/soil/ground/0"), "single number");
  fails(with("force_modules/soil/shoe_center"), "add the index");
  fails(with("force_modules/soil/shoe_center/3"), "beyond");
  fails(with("force_modules/soil/shoe_center/x"), "not an index");
  fails(with("force_modules/soil/shoe_center/-1"), "not an index");
  fails(with("force_modules/soil/ground/0/1"), "too many");
  fails(with("force_modules/soil"), "names an object");
  fails(with("force_modules"), "names an object");
  fails(with(""), "empty");
  // the controller's own settings
  fails(Controller("quadratic"), "interpolation 'quadratic'");
  fails(Controller("linear", "", "[0, 0, 1]"), "pairs");
  fails(Controller("linear", "", "[1, 0, 1, 1]"), "must increase");
  fails(Controller("linear", "", "[2, 0, 1, 1]"), "must increase");
  fails(Controller("linear", R"(, "transition_time": 0.1)"),
        "transition_time is for step");
  fails(Controller("step", R"(, "transition_time": -0.1)"), "negative");
  fails(Controller("step", R"(, "transition_time": 1.0)"),
        "less than the time between");
}
