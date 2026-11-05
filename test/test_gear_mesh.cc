/* The gear mesh force module (force_module::GearMesh): the stiffness as the
teeth roll through the mesh, the geometry of the forces on the gears, backlash
and teeth that push but never pull, the natural frequency of the pair, the
speed and torque ratios of a loaded pair, and the excitation of the
transmission error by the errors of the teeth */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/gear_mesh.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::GearMesh;

static const double kR1 = 0.05, kR2 = 0.1;
static const double kI1 = 0.01, kI2 = 0.08;
static const double kPhi = 0.3490658503988659;

/* two gears on revolute joints about parallel axes (z, down) 0.15 m apart,
meshing; the options go into the gear mesh, the drive and the load into the
model */
static void Load(shift::core::Context& context, const std::string& mesh,
                 double time_span, const std::string& drive = "",
                 double load = 0.0, double speed_1 = 0.0,
                 double speed_2 = 0.0) {
  const std::string path = testing::TempDir() + "/gear_mesh_test.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.0001,
                     "relative_error": 1e-8, "absolute_error": 1e-10}]
  },
  "frames": [{"name": "axis_2", "T": [)" << kR1 + kR2 << R"(, 0, 0]}],
  "assemblies": [{"name": "test",
    "bodies": [
      {"name": "gear_1", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": )" << kI1 << R"(,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "gear_2", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": )" << kI2 << R"(,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "turn_1", "type": "revolute",
       "parent": [["fixed"], "origin"], "child": [["gear_1"], "origin"],
       "axis": [0, 0, 1], "initial_velocity": )" << speed_1 << R"(},
      {"name": "turn_2", "type": "revolute",
       "parent": [["fixed"], "axis_2"], "child": [["gear_2"], "origin"],
       "axis": [0, 0, 1], "initial_velocity": )" << speed_2
         << (load > 0.0 ? R"(, "damping_coefficients": [0, )" +
                            std::to_string(load) + "]"
                        : std::string())
         << R"(}],)" << (drive.empty() ? "" : drive) << R"(
    "force_modules": [{"type": "gear_mesh", "name": "mesh",
      "gear_1": ["gear_1", "origin"], "gear_2": ["gear_2", "origin"],
      "pitch_radius_1": )" << kR1 << R"(, "pitch_radius_2": )" << kR2
         << R"(, "teeth_1": 20, )" << mesh << R"(}]
  }]
})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

static GearMesh* Find(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto mesh = dynamic_cast<GearMesh*>(module)) {
      return mesh;
    }
  }
  return nullptr;
}

/* evaluate the forces with the module's states e_t (the tangential
displacement of the pitch point) and turns of gear 1 as given, the gears at
rest in their joints' zero positions */
static void Evaluate(shift::core::Context& context, GearMesh* mesh, double e_t,
                     double theta) {
  context.UpdateDHVA();
  std::vector<double> y(
    context.get_state_count() + context.get_auxiliary_state_count(), 0.0);
  y[mesh->get_state_offset()] = e_t;
  y[mesh->get_state_offset() + 1] = theta;
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
}

static double Joint(shift::core::Context& context, const char* name,
                    const std::vector<double>& y) {
  auto joint = context["assemblies"]["test"]["joints"][name]
                 .get_data<shift::core::joint::Base>();
  return y[context.get_q_count() + joint->get_dq_index()];
}

TEST(GearMesh, PairsInContactThroughTheMesh) {
  // contact ratio 1.6: two pairs for 0.3 of the mesh either side of x = 0
  // (so for 0.6 of it), one for the rest, changing over a ramp of 0.1
  EXPECT_EQ(GearMesh::PairsInContact(0.0, 1.6, 0.1), 2.0);
  EXPECT_EQ(GearMesh::PairsInContact(0.2, 1.6, 0.1), 2.0);
  EXPECT_EQ(GearMesh::PairsInContact(0.8, 1.6, 0.1), 2.0);
  EXPECT_EQ(GearMesh::PairsInContact(0.5, 1.6, 0.1), 1.0);
  EXPECT_EQ(GearMesh::PairsInContact(0.4, 1.6, 0.1), 1.0);
  // half way at the edge of the double contact, 0.3 from x = 0
  EXPECT_NEAR(GearMesh::PairsInContact(0.3, 1.6, 0.1), 1.5, 1e-12);
  EXPECT_NEAR(GearMesh::PairsInContact(0.7, 1.6, 0.1), 1.5, 1e-12);
  // periodic, and the mean is the contact ratio
  EXPECT_NEAR(GearMesh::PairsInContact(2.2, 1.6, 0.1),
              GearMesh::PairsInContact(0.2, 1.6, 0.1), 1e-12);
  for(const auto& [epsilon, ramp] :
      {std::pair<double, double>{1.6, 0.1}, std::pair<double, double>{1.6, 0.0},
       std::pair<double, double>{1.2, 0.2}, std::pair<double, double>{1.9, 0.1}}) {
    SCOPED_TRACE(epsilon);
    const int n = 100000;
    double sum = 0.0, last = GearMesh::PairsInContact(0.0, epsilon, ramp);
    double biggest_step = 0.0;
    for(int i = 0; i < n; i++) {
      const double value = GearMesh::PairsInContact((i + 0.5) / n, epsilon, ramp);
      sum += value / n;
      biggest_step = std::max(biggest_step, std::fabs(value - last));
      last = value;
    }
    EXPECT_NEAR(sum, epsilon, 2e-4);
    if(ramp > 0.0) {
      // no jump in the number of pairs, and so in the force
      EXPECT_LT(biggest_step, 1e-3);
    }
  }
  EXPECT_EQ(GearMesh::PairsInContact(0.3, 1.0, 0.1), 1.0);
  EXPECT_EQ(GearMesh::PairsInContact(0.3, 2.0, 0.1), 2.0);
}

TEST(GearMesh, TheForcesOnTheGearsAreAtThePitchPoint) {
  shift::core::Context context;
  Load(context,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "smoothing": 1e-9)",
       0.0);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  // gear 1 ahead of gear 2 by 20 um at the pitch circles: the plus flank is
  // compressed along its line of action by cos(phi) 20 um
  const double e_t = 2e-5;
  Evaluate(context, mesh, e_t, 0.0);
  const double force = 1e8 * (std::cos(kPhi) * e_t - 0.5e-9);
  EXPECT_NEAR(mesh->get_force_plus(), force, 1e-9 * force);
  EXPECT_EQ(mesh->get_force_minus(), 0.0);
  EXPECT_NEAR(mesh->get_mesh_force(), force * std::cos(kPhi), 1e-9 * force);
  // the force on gear 2: along the tangent (+y here, for the z axes down and
  // the line of centers along +x) and apart along the line of centers, with
  // the opposite on gear 1, at the pitch point
  const auto& loads = context.get_body_loads();
  const int i1 = context.GetBodyLoadIndex(context.FindActiveBody("gear_1"));
  const int i2 = context.GetBodyLoadIndex(context.FindActiveBody("gear_2"));
  const double tangent = force * std::cos(kPhi);
  const double radial = force * std::sin(kPhi);
  EXPECT_NEAR(loads[i2].lin.x, radial, 1e-9 * radial);
  EXPECT_NEAR(loads[i2].lin.y, tangent, 1e-9 * tangent);
  EXPECT_NEAR(loads[i1].lin.x, -radial, 1e-9 * radial);
  EXPECT_NEAR(loads[i1].lin.y, -tangent, 1e-9 * tangent);
  // torques about the gears' own axes: tangent times the pitch radius, in
  // the ratio of the radii
  const double pitch_point = kR1;
  const double torque_1 = loads[i1].ang.z - 0.0 * loads[i1].lin.y;
  const double torque_2 = loads[i2].ang.z - (kR1 + kR2) * loads[i2].lin.y;
  EXPECT_NEAR(torque_1, -kR1 * tangent, 1e-9 * kR1 * tangent);
  EXPECT_NEAR(torque_2, -kR2 * tangent, 1e-9 * kR2 * tangent);
  EXPECT_NEAR(torque_1 / torque_2, kR1 / kR2, 1e-9);
  (void)pitch_point;
  // the other way: the minus flank, the forces change sign along the
  // tangent but not along the line of centers
  Evaluate(context, mesh, -e_t, 0.0);
  EXPECT_EQ(mesh->get_force_plus(), 0.0);
  EXPECT_NEAR(mesh->get_force_minus(), force, 1e-9 * force);
  EXPECT_NEAR(context.get_body_loads()[i2].lin.y, -tangent, 1e-9 * tangent);
  EXPECT_NEAR(context.get_body_loads()[i2].lin.x, radial, 1e-9 * radial);
}

TEST(GearMesh, BacklashLeavesTheTeethApartAndTheyNeverPull) {
  shift::core::Context context;
  // 40 um of circumferential backlash: 20 um either way at the pitch circles
  Load(context,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "backlash": 4e-5,
          "damping": 1e3, "smoothing": 1e-9)",
       0.0);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  for(double e_t : {0.0, 1e-5, -1.9e-5, 2e-5}) {
    Evaluate(context, mesh, e_t, 0.0);
    EXPECT_EQ(mesh->get_force_plus(), 0.0) << e_t;
    EXPECT_EQ(mesh->get_force_minus(), 0.0) << e_t;
  }
  // 5 um past the gap on the plus side
  Evaluate(context, mesh, 2.5e-5, 0.0);
  EXPECT_NEAR(mesh->get_force_plus(),
              1e8 * std::cos(kPhi) * 5e-6 - 0.5e-9 * 1e8, 1e-6);
  EXPECT_EQ(mesh->get_force_minus(), 0.0);
  // never negative, however the states lie
  for(double e_t = -1e-4; e_t < 1e-4; e_t += 3.1e-7) {
    Evaluate(context, mesh, e_t, 0.37);
    EXPECT_GE(mesh->get_force_plus(), 0.0);
    EXPECT_GE(mesh->get_force_minus(), 0.0);
  }
}

TEST(GearMesh, TheStiffnessFollowsThePositionInTheMesh) {
  shift::core::Context context;
  Load(context,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.6, "ramp": 0.1,
          "smoothing": 1e-9)",
       0.0);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  // 20 teeth: a mesh is 1/20 of a turn of gear 1. At the start two pairs
  // share the load, half a mesh later one
  const double turn = 2.0 * M_PI / 20.0;
  Evaluate(context, mesh, 2e-5, 0.0);
  const double double_pair = mesh->get_force_plus();
  Evaluate(context, mesh, 2e-5, 0.5 * turn);
  const double single_pair = mesh->get_force_plus();
  EXPECT_NEAR(double_pair / single_pair, 2.0, 1e-6);
  Evaluate(context, mesh, 2e-5, turn);
  EXPECT_NEAR(mesh->get_force_plus(), double_pair, 1e-6 * double_pair);
  EXPECT_LT(std::min(mesh->get_phase(), 1.0 - mesh->get_phase()), 1e-9);
  // and the starting position of the mesh moves it
  shift::core::Context shifted;
  Load(shifted,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.6, "ramp": 0.1,
          "initial_phase": 0.5, "smoothing": 1e-9)",
       0.0);
  Evaluate(shifted, Find(shifted), 2e-5, 0.0);
  EXPECT_NEAR(Find(shifted)->get_force_plus(), single_pair,
              1e-6 * single_pair);
}

TEST(GearMesh, TheTorsionalNaturalFrequencyOfThePair) {
  // one pair of teeth always in contact, no damping and no backlash: the
  // pitch point's displacement E oscillates at
  //   w^2 = k cos^2(phi) (r1^2 / I1 + r2^2 / I2)
  const double k = 1e8;
  const double omega =
    std::sqrt(k * std::cos(kPhi) * std::cos(kPhi) *
              (kR1 * kR1 / kI1 + kR2 * kR2 / kI2));
  shift::core::Context context;
  // gear 1 starts turning, gear 2 at rest: the mesh compresses
  Load(context,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "smoothing": 1e-9)",
       0.02, "", 0.0, 0.05, 0.0);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  std::vector<double> times, errors;
  context.set_state_observer(
    [&](double t, const std::vector<double>& y) {
      times.push_back(t);
      errors.push_back(y[mesh->get_state_offset()]);
    });
  context.Simulate();
  // the rising zero crossings of E
  std::vector<double> crossings;
  for(size_t i = 1; i < errors.size(); i++) {
    if(errors[i - 1] < 0.0 && errors[i] >= 0.0) {
      crossings.push_back(times[i - 1] + (times[i] - times[i - 1]) *
                                           (-errors[i - 1]) /
                                           (errors[i] - errors[i - 1]));
    }
  }
  ASSERT_GE(crossings.size(), 8u);
  const double period =
    (crossings.back() - crossings.front()) / (crossings.size() - 1);
  EXPECT_NEAR(2.0 * M_PI / period, omega, 5e-3 * omega);
}

/* a pair driven at a speed by a PID controller on gear 1, with a viscous load
on gear 2 */
static const char* kDrive = R"(
    "sensors": [{"type": "joint", "name": "speed_1", "joint": "turn_1",
                 "quantity": "velocity"}],
    "controllers": [{"type": "pid", "name": "drive", "kp": 5.0, "ki": 200.0,
                     "kd": 0, "target": 100.0, "sensor": "speed_1",
                     "joint": "turn_1"}],)";

TEST(GearMesh, ALoadedPairTurnsInTheRatioOfTheRadiiAndCarriesTheLoad) {
  shift::core::Context context;
  const double load = 0.2; // N m s: the torque on gear 2 is 0.2 omega_2
  Load(context,
       R"("tooth_stiffness": 1e8, "contact_ratio": 1.6, "damping": 7e3,
          "smoothing": 1e-9)",
       0.4, kDrive, load);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  std::vector<double> state;
  // the force of the teeth ripples as the number of pairs changes: its mean
  // over the last 0.1 s, of the force at each accepted step
  double sum = 0.0;
  int count = 0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    state = y;
    if(t > 0.3) {
      mesh->UpdateOutputs(t, y.data());
      sum += mesh->get_mesh_force();
      count++;
    }
  });
  context.Simulate();
  ASSERT_FALSE(state.empty());
  ASSERT_GT(count, 10);
  const double w1 = Joint(context, "turn_1", state);
  const double w2 = Joint(context, "turn_2", state);
  // the gears turn in the opposite senses, at the ratio of the pitch radii
  EXPECT_NEAR(w1, 100.0, 1.0);
  EXPECT_NEAR(w2 / w1, -kR1 / kR2, 2e-3);
  // and the teeth carry the load: the tangential force at the pitch circle
  // is the torque on gear 2 over its radius
  const double expected = load * std::fabs(w2) / kR2;
  EXPECT_NEAR(sum / count, expected, 0.02 * expected);
}

TEST(GearMesh, AnErrorOfTheTeethExcitesTheTransmissionError) {
  // the gears driven with constant stiffness: the transmission error is
  // steady; a periodic error of the teeth, once a mesh, makes it vary at the
  // mesh frequency (gear whine)
  auto ripple = [&](const std::string& mesh) {
    shift::core::Context context;
    Load(context, mesh, 0.3, kDrive, 0.2);
    GearMesh* module = Find(context);
    std::vector<double> errors;
    context.set_state_observer([&](double t, const std::vector<double>& y) {
      if(t > 0.2) {
        errors.push_back(y[module->get_state_offset()]);
      }
    });
    context.Simulate();
    const auto [low, high] = std::minmax_element(errors.begin(), errors.end());
    return *high - *low;
  };
  const double steady = ripple(
    R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "damping": 7e3,
       "smoothing": 1e-9)");
  const double whine = ripple(
    R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "damping": 7e3,
       "smoothing": 1e-9,
       "profile_error": [0, 0, 0.25, 5e-6, 0.5, 0, 0.75, -5e-6, 1, 0])");
  EXPECT_GT(whine, 2e-6);
  EXPECT_LT(steady, 0.05 * whine);
}

TEST(GearMesh, WhatItRefuses) {
  auto message = [&](const std::string& mesh) {
    shift::core::Context context;
    try {
      Load(context, mesh, 0.0);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(message(R"("contact_ratio": 1.5)").find("tooth_stiffness"),
            std::string::npos);
  EXPECT_NE(message(R"("tooth_stiffness": 1e8, "contact_ratio": 2.5)")
              .find("contact_ratio"),
            std::string::npos);
  EXPECT_NE(message(R"("tooth_stiffness": 1e8, "contact_ratio": 1.6,
                      "ramp": 0.9)")
              .find("ramp"),
            std::string::npos);
  EXPECT_NE(message(R"("tooth_stiffness": 1e8, "backlash": -1)")
              .find("backlash"),
            std::string::npos);
  EXPECT_NE(message(R"("stiffness_table": [0, 1e8, 0.5, 1e8])")
              .find("stiffness_table"),
            std::string::npos); // does not span [0, 1]
  EXPECT_EQ(message(R"("tooth_stiffness": 1e8)"), "");
  EXPECT_EQ(message(R"("stiffness_table": [0, 1e8, 0.5, 2e8, 1, 1e8])"), "");
}
