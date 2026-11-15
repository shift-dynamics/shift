/* Logging sensors: body_load reports the loads the force modules put on a
body, marker the pose and velocity of a frame. Both find their bodies by name
anywhere in the model, so they sit here in an assembly of their own */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <H5Cpp.h>
#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/sensor/base.h"

static const char* kBody =
  R"({"name": "body", "center_of_mass": [0, 0, 0],
      "frames": [{"name": "tip", "T": [1, 0, 0]}],
      "inertia": {"mass": 2, "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0,
                  "Iyz": 0, "Izx": 0}})";

/* the body in assembly "rig", the sensors in assembly "instruments" */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      const std::string& joint, const std::string& modules,
                      const std::string& sensors, double time_span,
                      const std::string& loggers = "",
                      const std::string& rig_extra = "") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, 0], "solvers": [{"type": "rnca"}],
      "loggers": [)" << loggers << R"(],
      "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.001, "hmax": 0.001,
                             "relative_error": 1e-9,
                             "absolute_error": 1e-11}]},
      "assemblies": [{"name": "rig", "bodies": [)" << kBody
         << R"(], "joints": [)" << joint << R"(],
         "force_modules": [)" << modules << R"(])" << rig_extra << R"(},
        {"name": "instruments", "sensors": [)" << sensors << R"(]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static shift::core::sensor::Base* FindSensor(shift::core::Context& context,
                                             const std::string& name) {
  for(auto sensor : context.get_active_sensors()) {
    if(sensor->get_name() == name) {
      return sensor;
    }
  }
  return nullptr;
}

static std::vector<double> Outputs(shift::core::Context& context,
                                   shift::core::sensor::Base* sensor) {
  sensor->UpdateOutputs(context.get_force_time(), nullptr);
  std::vector<double> values;
  for(const auto& key : sensor->get_output_keys()) {
    values.push_back(sensor->GetOutput(key));
  }
  return values;
}

/* 6 N along x and 2 N along z at the tip, 1 m along x from the origin of a
body sliding along x: the load is the force and, about the body's origin,
r x f = (0, -2, 0) N m, wherever the body has slid to */
TEST(Sensors, BodyLoadReportsTheForceOnABodyByName) {
  shift::core::Context context;
  LoadModel(context, "sensor_body_load",
            R"({"name": "j", "type": "prismatic",
                "parent": [["fixed"], "origin"],
                "child": [["body"], "origin"], "axis": [1, 0, 0]})",
            R"({"type": "applied_load", "name": "l", "body": "body",
                "frame": "tip", "coordinates": "fixed",
                "force": [6, 0, 2]})",
            R"({"type": "body_load", "name": "loads", "bodies": ["body"]})",
            1.0);
  context.Simulate();
  auto* sensor = FindSensor(context, "loads");
  ASSERT_NE(sensor, nullptr);
  const std::vector<std::string> names = sensor->get_output_keys();
  const std::vector<std::string> expected = {
    "body_fx", "body_fy", "body_fz", "body_mx", "body_my", "body_mz"};
  EXPECT_EQ(names, expected);
  const std::vector<double> values = Outputs(context, sensor);
  ASSERT_EQ(values.size(), 6u);
  const double load[6] = {6, 0, 2, 0, -2, 0};
  for(int i = 0; i < 6; i++) {
    EXPECT_NEAR(values[i], load[i], 1e-9) << names[i];
  }
  // the body has slid: a = 3 m/s^2 for 1 s
  EXPECT_GT(context.get_force_time(), 0.99);
}

TEST(Sensors, BodyLoadRejectsAnUnknownBody) {
  shift::core::Context context;
  EXPECT_THROW(
    LoadModel(context, "sensor_body_load_unknown",
              R"({"name": "j", "type": "prismatic",
                  "parent": [["fixed"], "origin"],
                  "child": [["body"], "origin"], "axis": [1, 0, 0]})",
              "",
              R"({"type": "body_load", "name": "loads",
                  "bodies": ["nobody"]})",
              0.1),
    shift::core::Exception);
}

/* the body turns about y at 0.5 rad/s: after time t, at angle a = 0.5 t, the
tip is at (cos a, 0, -sin a), pitched by a, with quaternion (cos a/2, 0,
sin a/2, 0), and moves at w x r = 0.5 (-sin a, 0, -cos a); in the tip's own
axes that is (0, 0, -0.5) */
TEST(Sensors, MarkerReportsThePoseAndVelocityOfAFrame) {
  const char* hinge =
    R"({"name": "j", "type": "revolute", "parent": [["fixed"], "origin"],
        "child": [["body"], "origin"], "axis": [0, 1, 0],
        "initial_velocity": 0.5})";
  for(const std::string axes : {"global", "local"}) {
    shift::core::Context context;
    LoadModel(context, "sensor_marker_" + axes, hinge, "",
              R"({"type": "marker", "name": "marker",
                  "frames": [["body", "tip"]], "coordinate_system": ")" +
                axes + R"("})",
              1.0);
    context.Simulate();
    auto* sensor = FindSensor(context, "marker");
    ASSERT_NE(sensor, nullptr);
    const std::vector<std::string> names = sensor->get_output_keys();
    ASSERT_EQ(names.size(), 16u);
    EXPECT_EQ(names[0], "body_tip_x");
    EXPECT_EQ(names[3], "body_tip_qw");
    EXPECT_EQ(names[7], "body_tip_roll");
    EXPECT_EQ(names[8], "body_tip_pitch");
    EXPECT_EQ(names[9], "body_tip_yaw");
    EXPECT_EQ(names[10], "body_tip_vx");
    EXPECT_EQ(names[15], "body_tip_wz");
    const std::vector<double> v = Outputs(context, sensor);
    ASSERT_EQ(v.size(), 16u);
    const double a = 0.5 * context.get_force_time();
    EXPECT_GT(a, 0.49);
    std::vector<double> expected = {std::cos(a),
                                    0,
                                    -std::sin(a),
                                    std::cos(a / 2),
                                    0,
                                    std::sin(a / 2),
                                    0,
                                    0,
                                    a,
                                    0,
                                    -0.5 * std::sin(a),
                                    0,
                                    -0.5 * std::cos(a),
                                    0,
                                    0.5,
                                    0};
    if(axes == "local") {
      expected[10] = 0;
      expected[12] = -0.5;
    }
    for(size_t i = 0; i < 16; i++) {
      EXPECT_NEAR(v[i], expected[i], 1e-7) << axes << " " << names[i];
    }
  }
}

/* the HDF5 logger writes a logging sensor's outputs as
/sensors/<assembly>__<sensor>/<output>, one row per output time: the marker's
pitch grows at 0.5 rad/s and the body load is steady */
TEST(Sensors, Hdf5LogsTheSensorsOutputs) {
  const std::string h5 = testing::TempDir() + "/sensor_outputs.h5";
  {
    shift::core::Context context;
    LoadModel(context, "sensor_hdf5",
              R"({"name": "j", "type": "revolute",
                  "parent": [["fixed"], "origin"],
                  "child": [["body"], "origin"], "axis": [0, 1, 0],
                  "initial_velocity": 0.5})",
              "",
              R"({"type": "marker", "name": "marker",
                  "frames": [["body", "tip"]]},
                 {"type": "body_load", "name": "loads",
                  "bodies": ["body"]})",
              0.5, R"({"type": "hdf5", "logfile": ")" + h5 + R"("})");
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
  const std::vector<double> pitch =
    read("/sensors/instruments__marker/body_tip_pitch");
  const std::vector<double> fz = read("/sensors/instruments__loads/body_fz");
  ASSERT_GT(time.size(), 10u);
  ASSERT_EQ(pitch.size(), time.size());
  ASSERT_EQ(fz.size(), time.size());
  for(size_t k = 0; k < time.size(); k++) {
    EXPECT_NEAR(pitch[k], 0.5 * time[k], 1e-7) << time[k];
    EXPECT_EQ(fz[k], 0.0);
  }
  file.close();
  std::remove(h5.c_str());
}

/* a PID controller with a derivative term on a joint sensor's position,
whose rate the sensor provides once connected: the controllers connect after
the sensors. It turns the hinge to its target, 0.3 rad, and holds it there */
TEST(Sensors, PidDerivativeUsesTheJointSensorsRate) {
  shift::core::Context context;
  LoadModel(context, "sensor_pid_kd",
            R"({"name": "j", "type": "revolute",
                "parent": [["fixed"], "origin"],
                "child": [["body"], "origin"], "axis": [0, 1, 0]})",
            "", "", 3.0, "",
            R"(, "sensors": [{"type": "joint", "name": "angle", "joint": "j",
                              "quantity": "position"}],
               "controllers": [{"type": "pid", "name": "hold", "kp": 100,
                                "ki": 0, "kd": 20, "target": 0.3,
                                "sensor": "angle", "joint": "j"}])");
  std::vector<double> last;
  context.set_state_observer(
    [&last](double, const std::vector<double>& y) { last = y; });
  context.Simulate();
  ASSERT_EQ(last.size(), 3u); // q, dq and the integral of the error
  EXPECT_NEAR(last[0], 0.3, 1e-4);
  EXPECT_NEAR(last[1], 0.0, 1e-3);
}
