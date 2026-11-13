/* Inputs and outputs of components (parameter::List::DeclareInput and
DeclareOutput): setting and reading them, the model file, one driver per
input, and a PID controller that drives the valve command of a hydraulic
cylinder through its input */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <json/json.h>

#include "shift/core/context.h"
#include "shift/core/controller/pid.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/hydraulic_cylinder.h"
#include "shift/core/parameter/list.h"

/* a component with a parameter, an input and an output */
class IoComponent : public shift::core::parameter::List {
public:
  IoComponent() {
    DeclareParameters();
  }

  void DeclareParameters() override {
    DeclareParameter("name", &name_, "name", false);
    DeclareParameter("gain", &gain_, "a parameter", false);
    DeclareInput("current", &current_, "an input");
    DeclareOutput("torque", &torque_, "an output");
  }

  /* what a component does when it is evaluated: read its input, compute
  its output */
  void Evaluate() {
    torque_ = gain_ * current_;
  }

  std::string name_ = "motor";
  double gain_ = 2.0;
  double current_ = 0.5;
  double torque_ = 0.0;
};

/* the message of the exception that f throws, or "" */
template <class F>
static std::string Message(F f) {
  try {
    f();
  } catch(const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(ComponentIo, SetAnInputAndReadAnOutput) {
  IoComponent motor;
  EXPECT_TRUE(motor.has_input("current"));
  EXPECT_FALSE(motor.has_input("torque"));
  EXPECT_FALSE(motor.has_input("gain"));
  EXPECT_TRUE(motor.has_output("torque"));
  EXPECT_FALSE(motor.has_output("current"));
  EXPECT_EQ(motor.GetInput("current"), 0.5);
  motor.SetInput("current", 3.0);
  EXPECT_EQ(motor.current_, 3.0);
  EXPECT_EQ(motor.GetInput("current"), 3.0);
  motor.Evaluate();
  EXPECT_EQ(motor.GetOutput("torque"), 6.0);
  /* an input is a parameter that can change as the model runs */
  EXPECT_TRUE(motor["current"].is_runtime());
  EXPECT_FALSE(motor["torque"].is_runtime());
}

TEST(ComponentIo, OnlyInputsAreSetAndOnlyOutputsRead) {
  IoComponent motor;
  const std::string output = Message([&] { motor.SetInput("torque", 1.0); });
  EXPECT_NE(output.find("has no input 'torque' (it is an output)"),
            std::string::npos)
    << output;
  EXPECT_NE(output.find("its inputs: current"), std::string::npos) << output;
  const std::string parameter =
    Message([&] { motor.SetInput("gain", 1.0); });
  EXPECT_NE(parameter.find("(it is a parameter)"), std::string::npos)
    << parameter;
  EXPECT_EQ(motor.gain_, 2.0);
  const std::string unknown =
    Message([&] { motor.SetInput("voltage", 1.0); });
  EXPECT_NE(unknown.find("'motor' has no input 'voltage'"), std::string::npos)
    << unknown;
  const std::string input = Message([&] { motor.GetOutput("current"); });
  EXPECT_NE(input.find("has no output 'current' (it is an input)"),
            std::string::npos)
    << input;
}

static Json::Value Parse(const std::string& text) {
  Json::Value json;
  std::istringstream stream(text);
  std::string errors;
  EXPECT_TRUE(Json::parseFromStream(Json::CharReaderBuilder(), stream, &json,
                                    &errors))
    << errors;
  return json;
}

TEST(ComponentIo, TheModelFileGivesAnInputButNotAnOutput) {
  IoComponent defaulted;
  Json::Value none = Parse(R"({"name": "m"})");
  defaulted.ReadJSON(none);
  EXPECT_EQ(defaulted.current_, 0.5);

  IoComponent given;
  Json::Value current = Parse(R"({"name": "m", "current": 1.25})");
  given.ReadJSON(current);
  EXPECT_EQ(given.current_, 1.25);

  IoComponent output;
  Json::Value torque = Parse(R"({"name": "m", "torque": 1})");
  const std::string message = Message([&] { output.ReadJSON(torque); });
  EXPECT_NE(message.find("'torque' is an output"), std::string::npos)
    << message;
}

TEST(ComponentIo, OneDriverPerInput) {
  IoComponent motor;
  motor.ConnectInput("current", "a__pid");
  EXPECT_EQ(motor["current"].get_driver(), "a__pid");
  /* the same driver connecting again, as on a second Connect */
  EXPECT_NO_THROW(motor.ConnectInput("current", "a__pid"));
  const std::string second =
    Message([&] { motor.ConnectInput("current", "a__schedule"); });
  EXPECT_NE(second.find("set by a__pid already"), std::string::npos)
    << second;
  EXPECT_EQ(motor["current"].get_driver(), "a__pid");
  const std::string output =
    Message([&] { motor.ConnectInput("torque", "a__pid"); });
  EXPECT_NE(output.find("has no input 'torque'"), std::string::npos)
    << output;
}

/* a load on a horizontal prismatic joint, moved by a hydraulic cylinder,
with a joint position sensor and the controllers of the argument; extra
adds components (with a comma before them) to the sensors */
static std::string Model(const std::string& controllers,
                         const std::string& extra_sensors = "",
                         double time_span = 2.0) {
  std::ostringstream model;
  model << R"({
  "model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
        << time_span << R"(, "time_step": 0.01,
                     "relative_error": 1e-7, "absolute_error": 1e-9}]},
  "assemblies": [{"name": "a",
    "bodies": [{"name": "load", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 500, "Ixx": 10, "Iyy": 10, "Izz": 10, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "slide", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["load"], "origin"], "axis": [1, 0, 0]}],
    "force_modules": [{"type": "hydraulic_cylinder", "name": "cylinder",
      "joint": "slide", "stroke": 0.5, "bore_diameter": 0.05,
      "rod_diameter": 0.03, "retracted_position": -0.1,
      "supply_pressure": 10e6, "relief_pressure": 20e6,
      "rated_flow": 4.5e-5, "friction": 1e4}],
    "sensors": [{"type": "joint", "name": "extension", "joint": "slide",
                 "quantity": "position"})"
        << extra_sensors << R"(],
    "controllers": [)"
        << controllers << R"(]}]})";
  return model.str();
}

static void Load(shift::core::Context& context, const std::string& json) {
  const std::string path = testing::TempDir() + "/component_io_test.json";
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

/* the message of the error that loading the model raises */
static std::string LoadError(const std::string& controllers,
                             const std::string& extra_sensors = "") {
  shift::core::Context context;
  return Message([&] { Load(context, Model(controllers, extra_sensors)); });
}

static const char* kPositionControl =
  R"({"type": "pid", "name": "position", "kp": 20, "target": 0.05,
      "sensor": "extension", "component": "cylinder", "input": "command"})";

/* the PID controller sets the valve command through the cylinder's input,
and the load comes to the target: the cylinder integrates the command, so a
proportional controller leaves no error. The error decays with a time
constant of about 0.55 s, so after 6 s it is about 1e-6. */
TEST(ComponentIo, PidDrivesTheValveCommandOfACylinder) {
  shift::core::Context context;
  Load(context, Model(kPositionControl, "", 6.0));
  auto cylinder = dynamic_cast<shift::core::force_module::HydraulicCylinder*>(
    context["assemblies"]["a"]["force_modules"]["cylinder"]
      .get_data<shift::core::force_module::Base>());
  auto pid = dynamic_cast<shift::core::controller::PID*>(
    context["assemblies"]["a"]["controllers"]["position"]
      .get_data<shift::core::controller::Base>());
  ASSERT_NE(cylinder, nullptr);
  ASSERT_NE(pid, nullptr);
  EXPECT_EQ((*cylinder)["command"].get_driver(), "a__position");

  std::vector<double> y;
  double largest_command = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& state) {
    y = state;
    largest_command = std::max(largest_command,
                               std::fabs(cylinder->GetInput("command")));
  });
  context.Simulate();
  ASSERT_FALSE(y.empty());
  EXPECT_NEAR(y[0], 0.05, 2e-5);
  EXPECT_NEAR(y[context.get_q_count()], 0.0, 2e-5);
  /* the command was the controller's effort, which opened the valve fully
  at the start (an error of 0.05 times kp is 1) */
  EXPECT_EQ(cylinder->GetInput("command"), pid->GetOutput("effort"));
  EXPECT_NEAR(pid->GetOutput("effort"), 20.0 * pid->GetOutput("error"),
              1e-12);
  EXPECT_GT(largest_command, 0.9);
}

TEST(ComponentIo, PidFindsItsComponentOrSaysWhy) {
  const std::string missing = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension",
        "component": "valve", "input": "command"})");
  EXPECT_NE(missing.find("component 'valve' not found"), std::string::npos)
    << missing;
  const std::string not_input = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension",
        "component": "cylinder", "input": "stroke"})");
  EXPECT_NE(not_input.find("'cylinder' has no input 'stroke' (it is a "
                           "parameter); its inputs: command"),
            std::string::npos)
    << not_input;
  const std::string both = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension",
        "joint": "slide", "component": "cylinder", "input": "command"})");
  EXPECT_NE(both.find("give either joint"), std::string::npos) << both;
  const std::string neither = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension"})");
  EXPECT_NE(neither.find("give either joint"), std::string::npos) << neither;
  const std::string half = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension",
        "component": "cylinder"})");
  EXPECT_NE(half.find("component and input go together"), std::string::npos)
    << half;
}

/* a sensor that shares the cylinder's name: the bare name is ambiguous, the
kind and name are not */
TEST(ComponentIo, TheKindSettlesAComponentNameThatTwoKindsShare) {
  const std::string sensor =
    R"(, {"type": "joint", "name": "cylinder", "joint": "slide",
          "quantity": "position"})";
  const std::string ambiguous = LoadError(kPositionControl, sensor);
  EXPECT_NE(ambiguous.find("give one of force_modules/cylinder, "
                           "sensors/cylinder"),
            std::string::npos)
    << ambiguous;
  std::string with_kind = kPositionControl;
  with_kind.replace(with_kind.find("\"cylinder\""), 10,
                    "\"force_modules/cylinder\"");
  EXPECT_EQ(LoadError(with_kind, sensor), "");
  const std::string bad_kind = LoadError(
    R"({"type": "pid", "name": "p", "kp": 1, "sensor": "extension",
        "component": "pumps/cylinder", "input": "command"})");
  EXPECT_NE(bad_kind.find("'pumps' is not a kind of component"),
            std::string::npos)
    << bad_kind;
}

/* a schedule and a PID controller cannot both set the valve command, in
either order in the file (schedules connect first, as they run first) */
TEST(ComponentIo, ScheduleAndPidCannotShareAnInput) {
  const std::string schedule =
    R"({"type": "schedule", "name": "valve", "points": [0, 0, 1, 1],
        "targets": ["force_modules/cylinder/command"]})";
  for(const std::string& controllers :
      {std::string(kPositionControl) + ", " + schedule,
       schedule + ", " + kPositionControl}) {
    const std::string message = LoadError(controllers);
    EXPECT_NE(message.find("input 'command' is set by a__valve already, so "
                           "a__position cannot set it too"),
              std::string::npos)
      << message;
  }
  /* a schedule alone still sets it, and the PID's target is an input that
  a schedule can set */
  EXPECT_EQ(LoadError(schedule), "");
  EXPECT_EQ(LoadError(std::string(kPositionControl) +
                      R"(, {"type": "schedule", "name": "move",
                            "points": [0, 0.05, 1, 0.1],
                            "targets": ["controllers/position/target"]})"),
            "");
}

/* a key is a parameter, an input or an output, not two: an output of a
parameter's key would replace it */
class CollidingComponent : public shift::core::parameter::List {
public:
  void DeclareParameters() override {
    DeclareParameter("friction", &friction_, "a parameter", false);
    DeclareOutput("friction", &force_, "an output");
  }
  double friction_ = 0.1;
  double force_ = 0.0;
};

TEST(ComponentIo, AnOutputCannotTakeAParametersKey) {
  CollidingComponent component;
  const std::string message = Message([&] { component.DeclareParameters(); });
  EXPECT_NE(message.find("'friction' is declared both as a parameter and as "
                         "an output"),
            std::string::npos)
    << message;
  EXPECT_EQ(component.Find("friction")->get_data<double>(),
            &component.friction_);
}
