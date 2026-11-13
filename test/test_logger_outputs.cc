/* The outputs that the loggers record (logger::Base::ConnectOutputs): every
output of the force modules, sensors and controllers by default, otherwise
those of the components the logger names in its force_modules, sensors and
controllers, found by name as a joint finds a body (the fully qualified name
where the name is not unique), as columns of the CSV log and datasets of the
HDF5 log */

#include <gtest/gtest.h>

#include <H5Cpp.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shift/core/context.h"

/* the rotor's inertia about its axis and the motor's torque constant; the
schedule holds the motor's current at 2 A, so the torque is 0.2 N m and the
rotor's speed 20 t rad/s */
static const double kInertia = 0.01;
static const double kTorque = 0.2;

/* an assembly of a rotor driven by a motor of the current drive, with the
sensors and controllers of the arguments */
static std::string Assembly(const std::string& name,
                            const std::string& sensors,
                            const std::string& controllers) {
  std::ostringstream assembly;
  assembly << R"({"name": ")" << name << R"(",
    "bodies": [{"name": "rotor", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": )"
           << kInertia << R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [{"name": "bearing", "type": "revolute", "axis": [0, 0, 1],
                "parent": [["fixed"], "origin"],
                "child": [["rotor"], "origin"]}],
    "force_modules": [{"type": "electric_motor", "name": "motor",
      "joint": "bearing", "torque_constant": 0.1, "resistance": 1}],
    "sensors": [)"
           << sensors << R"(],
    "controllers": [)"
           << controllers << "]}";
  return assembly.str();
}

static const char* const kSensors =
  R"({"type": "joint", "name": "speed", "joint": "bearing",
      "quantity": "velocity"})";
static const char* const kControllers =
  R"({"type": "schedule", "name": "current_profile", "points": [0, 2, 1, 2],
      "targets": ["force_modules/motor/current"]})";

/* a model of the assemblies, logged by logger */
static std::string Model(const std::string& logger,
                         const std::string& assemblies) {
  return R"({
  "model_parameters": {"version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81], "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": 1.0, "time_step": 0.1,
                     "relative_error": 1e-9, "absolute_error": 1e-11}],
    "loggers": [)" +
         logger + R"(]},
  "assemblies": [)" +
         assemblies + "]}";
}

static void Load(shift::core::Context& context, const std::string& name,
                 const std::string& json) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
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

/* the header and the last row of a CSV log, its columns split at the commas
(each line ends with one) */
struct Csv {
  std::vector<std::string> header;
  std::vector<double> last;
};

static std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream stream(line);
  std::string cell;
  while(std::getline(stream, cell, ',')) {
    cells.push_back(cell);
  }
  return cells;
}

/* simulate the model of the one assembly "a", logged to CSV with the keys
of logger */
static Csv RunCsv(const std::string& name, const std::string& logger) {
  const std::string csv = testing::TempDir() + "/" + name + ".csv";
  {
    shift::core::Context context;
    Load(context, name,
         Model(R"({"type": "csv", "csv_logfile": ")" + csv + "\"" + logger +
                 "}",
               Assembly("a", kSensors, kControllers)));
    context.Simulate();
  }
  std::ifstream file(csv);
  std::string line, last;
  std::getline(file, line);
  Csv result;
  result.header = Split(line);
  while(std::getline(file, line)) {
    if(!line.empty()) {
      last = line;
    }
  }
  for(const auto& cell : Split(last)) {
    result.last.push_back(std::stod(cell));
  }
  file.close();
  std::remove(csv.c_str());
  return result;
}

/* the columns after total_energy, those of the outputs (the model has no
chord constraints) */
static std::vector<std::string> OutputColumns(const Csv& csv) {
  for(size_t k = 0; k < csv.header.size(); k++) {
    if(csv.header[k] == "total_energy") {
      return std::vector<std::string>(csv.header.begin() + k + 1,
                                      csv.header.end());
    }
  }
  return {};
}

static double Column(const Csv& csv, const std::string& name) {
  for(size_t k = 0; k < csv.header.size(); k++) {
    if(csv.header[k] == name) {
      return csv.last.at(k);
    }
  }
  ADD_FAILURE() << "no column " << name;
  return NAN;
}

/* without "outputs", every output: the force modules', the sensors', then
the controllers', each in the order the component declares them */
TEST(LoggerOutputs, EveryOutputByDefault) {
  const Csv csv = RunCsv("logger_outputs_default", "");
  const std::vector<std::string> expected = {
    "a__motor__winding_current", "a__motor__terminal_voltage",
    "a__motor__back_voltage",    "a__motor__torque",
    "a__motor__speed",           "a__motor__electrical_power",
    "a__motor__mechanical_power", "a__speed__value",
    "a__current_profile__value"};
  EXPECT_EQ(OutputColumns(csv), expected);
  /* at 1 s: 2 A, 0.2 N m and 20 rad/s, as the joint's rate */
  EXPECT_NEAR(Column(csv, "a__motor__winding_current"), 2.0, 1e-12);
  EXPECT_NEAR(Column(csv, "a__motor__torque"), kTorque, 1e-12);
  EXPECT_NEAR(Column(csv, "a__speed__value"), kTorque / kInertia, 1e-5);
  EXPECT_NEAR(Column(csv, "a__speed__value"), Column(csv, "a__bearing__dq_0"),
              1e-12);
  EXPECT_NEAR(Column(csv, "a__motor__speed"), Column(csv, "a__bearing__dq_0"),
              1e-12);
  EXPECT_NEAR(Column(csv, "a__current_profile__value"), 2.0, 1e-12);
}

/* the components named by the logger, by kind, with the outputs of each, or
all of them; force modules first, then sensors, then controllers, and an
output already logged is not repeated */
TEST(LoggerOutputs, TheLoggerNamesTheComponents) {
  const Csv csv = RunCsv(
    "logger_outputs_chosen",
    R"(, "sensors": [{"name": "speed", "outputs": ["value"]},
                    {"name": "a__speed"}],
         "force_modules": [{"name": "motor", "outputs": ["torque"]}],
         "controllers": [{"name": "current_profile"}])");
  const std::vector<std::string> expected = {
    "a__motor__torque", "a__speed__value", "a__current_profile__value"};
  EXPECT_EQ(OutputColumns(csv), expected);
  EXPECT_NEAR(Column(csv, "a__motor__torque"), kTorque, 1e-12);
  /* the states are logged as before */
  EXPECT_EQ(csv.header.front(), "time");
  EXPECT_EQ(csv.header[1], "a__bearing__q_0");
}

/* naming one kind logs only the components named */
TEST(LoggerOutputs, AnEmptyListLogsNoOutputs) {
  const Csv csv = RunCsv("logger_outputs_none", R"(, "force_modules": [])");
  EXPECT_TRUE(OutputColumns(csv).empty());
  EXPECT_EQ(csv.header.back(), "total_energy");
  EXPECT_EQ(csv.last.size(), csv.header.size());
}

/* the message of the error that loading the model raises */
static std::string LoadError(const std::string& logger,
                             const std::string& assemblies) {
  shift::core::Context context;
  try {
    Load(context, "logger_outputs_error",
         Model(R"({"type": "csv", "csv_logfile": ")" + testing::TempDir() +
                 R"(/logger_outputs_error.csv", )" + logger + "}",
               assemblies));
  } catch(const std::exception& e) {
    return e.what();
  }
  std::remove((testing::TempDir() + "/logger_outputs_error.csv").c_str());
  return "";
}

/* names are unique within a kind and an assembly: the motors of a and b are
told apart by their fully qualified names, and a sensor that shares the
motor's name is in a list of its own */
TEST(LoggerOutputs, TheFullyQualifiedNameSettlesAName) {
  const std::string assemblies =
    Assembly("a",
             std::string(kSensors) +
               R"(, {"type": "joint", "name": "motor", "joint": "bearing",
                     "quantity": "position"})",
             kControllers) +
    ", " + Assembly("b", "", "");
  const std::string bare =
    LoadError(R"("force_modules": [{"name": "motor"}])", assemblies);
  EXPECT_NE(bare.find("Logger: force_modules: 'motor': the name is ambiguous: "
                      "use the fully qualified name, one of 'a__motor', "
                      "'b__motor'"),
            std::string::npos)
    << bare;
  EXPECT_EQ(LoadError(R"("force_modules": [{"name": "b__motor",
                                            "outputs": ["torque"]},
                                           {"name": "a__motor"}],
                         "sensors": [{"name": "motor",
                                      "outputs": ["rate"]}])",
                      assemblies),
            "");
}

TEST(LoggerOutputs, SaysWhatIsWrongWithAnEntry) {
  const std::string assembly = Assembly("a", kSensors, kControllers);
  const std::string none =
    LoadError(R"("force_modules": [{"name": "pump"}])", assembly);
  EXPECT_NE(none.find("Logger: force_modules: 'pump': there is no active "
                      "force module of that name"),
            std::string::npos)
    << none;
  const std::string kind =
    LoadError(R"("sensors": [{"name": "motor"}])", assembly);
  EXPECT_NE(kind.find("there is no active sensor of that name"),
            std::string::npos)
    << kind;
  const std::string output = LoadError(
    R"("force_modules": [{"name": "motor", "outputs": ["current"]}])",
    assembly);
  EXPECT_NE(output.find("'motor' has no output 'current' (it is an input); "
                        "its outputs: winding_current, terminal_voltage, "
                        "back_voltage, torque, speed, electrical_power, "
                        "mechanical_power"),
            std::string::npos)
    << output;
}

/* the HDF5 log holds each output as /<kind>/<component>/<output>, from the
first row: the schedule sets the motor's current before the outputs of t = 0
are logged */
TEST(LoggerOutputs, Hdf5HoldsTheOutputsByKind) {
  const std::string h5 = testing::TempDir() + "/logger_outputs.h5";
  {
    shift::core::Context context;
    Load(context, "logger_outputs_hdf5",
         Model(R"({"type": "hdf5", "logfile": ")" + h5 +
                 R"(", "force_modules": [{"name": "motor",
                                          "outputs": ["torque"]}],
                     "sensors": [{"name": "speed"}],
                     "controllers": [{"name": "current_profile"}]})",
               Assembly("a", kSensors, kControllers)));
    context.Simulate();
  }
  H5::H5File file(h5, H5F_ACC_RDONLY);
  auto read = [&file](const std::string& path) {
    H5::DataSet dataset = file.openDataSet(path);
    hsize_t n = 0;
    dataset.getSpace().getSimpleExtentDims(&n);
    std::vector<double> values(n);
    dataset.read(values.data(), H5::PredType::NATIVE_DOUBLE);
    return values;
  };
  const std::vector<double> time = read("/time");
  const std::vector<double> torque = read("/force_modules/a__motor/torque");
  const std::vector<double> speed = read("/sensors/a__speed/value");
  const std::vector<double> current =
    read("/controllers/a__current_profile/value");
  ASSERT_GT(time.size(), 5u);
  ASSERT_EQ(torque.size(), time.size());
  ASSERT_EQ(speed.size(), time.size());
  ASSERT_EQ(current.size(), time.size());
  for(size_t k = 0; k < time.size(); k++) {
    EXPECT_NEAR(torque[k], kTorque, 1e-12) << time[k];
    EXPECT_NEAR(speed[k], kTorque / kInertia * time[k], 1e-5) << time[k];
    EXPECT_NEAR(current[k], 2.0, 1e-12) << time[k];
  }
  EXPECT_FALSE(file.nameExists("/force_modules/a__motor/speed"));
  file.close();
  std::remove(h5.c_str());
}
