/* The gear mesh force module (force_module::GearMesh): the stiffness as the
teeth roll through the mesh, the geometry of the forces on the gears, backlash
and teeth that push but never pull, the natural frequency of the pair, the
speed and torque ratios of a loaded pair, and the excitation of the
transmission error by the errors of the teeth */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/gear_mesh.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::GearMesh;
using shift::core::force_module::GearTeeth;

static const double kR1 = 0.05, kR2 = 0.1;
static const double kI1 = 0.01, kI2 = 0.08;
static const double kPhi = 0.3490658503988659;

/* two gears on revolute joints about parallel axes (z, down) 0.15 m apart,
meshing; the options go into the gear mesh, the drive and the load into the
model */
static void Load(shift::core::Context& context, const std::string& mesh,
                 double time_span, const std::string& drive = "",
                 double load = 0.0, double speed_1 = 0.0,
                 double speed_2 = 0.0, double center = kR1 + kR2) {
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
  "frames": [{"name": "axis_2", "T": [)" << center << R"(, 0, 0]}],
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
    // 2e-5 is exactly at the edge of the gap, where rounding may leave ~1e-25 N
    EXPECT_NEAR(mesh->get_force_plus(), 0.0, 1e-15) << e_t;
    EXPECT_NEAR(mesh->get_force_minus(), 0.0, 1e-15) << e_t;
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

TEST(GearMeshInternal, ARingTurnsTheSameWayAndCarriesTheLoad) {
  shift::core::Context context;
  const double load = 0.2;
  Load(context,
       R"("internal": true, "tooth_stiffness": 1e8, "contact_ratio": 1.6,
          "damping": 7e3, "smoothing": 1e-9)",
       0.4, kDrive, load, 0.0, 0.0, kR2 - kR1);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  std::vector<double> state;
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
  ASSERT_GT(count, 10);
  const double w1 = Joint(context, "turn_1", state);
  const double w2 = Joint(context, "turn_2", state);
  EXPECT_NEAR(w2 / w1, kR1 / kR2, 2e-3);
  const double expected = load * std::fabs(w2) / kR2;
  EXPECT_NEAR(std::fabs(sum / count), expected, 0.02 * expected);
}

TEST(GearMeshInternal, APinionPushedTowardsThePitchPointCompressesTheTeeth) {
  // the centers of an internal pair get farther apart as the pinion moves
  // into the ring's teeth, the opposite of an external pair
  shift::core::Context context;
  Load(context, R"("internal": true, "tooth_stiffness": 1e8,
                   "contact_ratio": 1.0, "ramp": 0, "smoothing": 1e-13)",
       0.1, "", 0.0, 0.0, 0.0, kR2 - kR1);
  GearMesh* mesh = Find(context);
  Evaluate(context, mesh, 1e-5, 0.0);
  const double cp = std::cos(20.0 * M_PI / 180.0);
  EXPECT_NEAR(mesh->get_force_plus(), 1e8 * cp * 1e-5, 1e-4);
  EXPECT_NEAR(mesh->get_radial_force(),
              mesh->get_mesh_force() * std::tan(20.0 * M_PI / 180.0), 1e-6);
}

TEST(GearMeshHelical, TheForcesAreThoseOfAHelicalGear) {
  // the textbook relations: f_a = f_t tan(beta),
  // f_r = f_t tan(phi_n) / cos(beta)
  const double beta = 0.3, phi = 20.0 * M_PI / 180.0;
  shift::core::Context context;
  Load(context, R"("helix_angle": 0.3, "tooth_stiffness": 1e8,
                   "contact_ratio": 1.0, "ramp": 0, "smoothing": 1e-13)",
       0.1);
  GearMesh* mesh = Find(context);
  Evaluate(context, mesh, 1e-5, 0.0);
  const double tangent = mesh->get_mesh_force();
  EXPECT_NEAR(tangent, 1e8 * 1e-5 * std::pow(std::cos(phi) * std::cos(beta), 2.0),
              1e-4);
  EXPECT_NEAR(std::fabs(mesh->get_axial_force()), tangent * std::tan(beta), 1e-6);
  EXPECT_NEAR(mesh->get_radial_force(),
              tangent * std::tan(phi) / std::cos(beta), 1e-6);
  // a right hand helix on gear 1 pushes gear 2 against the axis when gear 1
  // drives it along the tangent, and the opposite for a left hand one
  EXPECT_LT(mesh->get_axial_force(), 0.0);
  shift::core::Context left;
  Load(left, R"("helix_angle": -0.3, "tooth_stiffness": 1e8,
                "contact_ratio": 1.0, "ramp": 0)",
       0.1);
  Evaluate(left, Find(left), 1e-5, 0.0);
  EXPECT_GT(Find(left)->get_axial_force(), 0.0);
}

TEST(GearMeshHelical, AWholeToothOfOverlapLeavesAConstantStiffness) {
  // an overlap of one pitch: every position in the mesh is present somewhere
  // along the face, so the stiffness is the mean, k_t times the contact ratio
  const double pitch = 2.0 * M_PI * kR1 / 20.0;
  const double beta = 0.3;
  std::ostringstream options;
  options.precision(17);
  options << R"("helix_angle": 0.3, "tooth_stiffness": 1e8,
                "contact_ratio": 1.6, "face_width": )"
          << pitch / std::tan(beta);
  shift::core::Context context;
  Load(context, options.str(), 0.1);
  GearMesh* mesh = Find(context);
  for(double x = 0.0; x <= 1.0; x += 0.1) {
    EXPECT_NEAR(mesh->StiffnessAt(x), 1.6e8, 0.02 * 1.6e8);
  }
}

TEST(GearTeeth, TheContactRatioIsThatOfTheInvolutes) {
  // module 2 mm, 25 and 50 teeth, 20 degrees
  GearTeeth teeth;
  teeth.youngs_modulus = 2.06e11;
  const double phi = 20.0 * M_PI / 180.0, m = 0.002;
  const double r_1 = 0.025, r_2 = 0.05;
  teeth.Configure(r_1, r_2, 25, 50, phi, m, false);
  const double b_1 = r_1 * std::cos(phi), b_2 = r_2 * std::cos(phi);
  const double path = std::sqrt(std::pow(r_1 + m, 2.0) - b_1 * b_1) +
                      std::sqrt(std::pow(r_2 + m, 2.0) - b_2 * b_2) -
                      (r_1 + r_2) * std::sin(phi);
  EXPECT_NEAR(teeth.get_contact_ratio(), path / (M_PI * m * std::cos(phi)), 1e-12);
  EXPECT_GT(teeth.get_contact_ratio(), 1.6);
  EXPECT_LT(teeth.get_contact_ratio(), 1.8);
}

TEST(GearTeeth, AgreesWithTheSingleStiffnessOfISO6336) {
  // ISO 6336-1 for solid steel gears without shift:
  // c' = 0.8 / (0.04723 + 0.15551 / z_1 + 0.25791 / z_2) N/(mm um)
  GearTeeth teeth;
  teeth.youngs_modulus = 2.06e11;
  const double phi = 20.0 * M_PI / 180.0;
  for(int z_1 : {20, 25, 30}) {
    const int z_2 = 2 * z_1;
    const double m = 0.002;
    teeth.Configure(0.5 * m * z_1, 0.5 * m * z_2, z_1, z_2, phi, m, false);
    const double single = teeth.MeshStiffness(0.5, 1.0); // N/m per m of width
    const double iso = 0.8 / (0.04723 + 0.15551 / z_1 + 0.25791 / z_2) * 1e9;
    EXPECT_NEAR(single / iso, 1.0, 0.1) << z_1;
  }
}

TEST(GearTeeth, ThePairsShareTheMeshAndTheStiffnessScales) {
  GearTeeth teeth;
  teeth.youngs_modulus = 2.06e11;
  teeth.Configure(0.025, 0.05, 25, 50, 20.0 * M_PI / 180.0, 0.002, false);
  const double epsilon = teeth.get_contact_ratio();
  // a pair is stiffest in the middle of its path
  const double middle = teeth.PairStiffness(0.5 * epsilon, 0.02);
  EXPECT_GT(middle, teeth.PairStiffness(0.0, 0.02));
  EXPECT_GT(middle, teeth.PairStiffness(epsilon, 0.02));
  // one pair at x = 0.5, two at x = 0
  EXPECT_NEAR(teeth.MeshStiffness(0.5, 0.02), middle, 1e-6 * middle);
  EXPECT_GT(teeth.MeshStiffness(0.0, 0.02), 1.5 * middle);
  // in proportion to the face width and to the modulus
  EXPECT_NEAR(teeth.PairStiffness(0.7, 0.04), 2.0 * teeth.PairStiffness(0.7, 0.02),
              1e-3);
  teeth.youngs_modulus *= 0.5;
  EXPECT_NEAR(teeth.PairStiffness(0.7, 0.02) * 2.0,
              teeth.PairStiffness(0.7, 0.04), 1e-3);
}

TEST(GearTeeth, AMeshTakesItsStiffnessFromTheTeeth) {
  // the pitch radii of the test, 20 and 40 teeth
  shift::core::Context context;
  Load(context, R"("face_width": 0.02,
                   "tooth_geometry": {"youngs_modulus": 2.06e11})",
       0.1);
  GearMesh* mesh = Find(context);
  ASSERT_NE(mesh, nullptr);
  EXPECT_GT(mesh->get_contact_ratio(), 1.5);
  EXPECT_LT(mesh->get_contact_ratio(), 1.8);
  EXPECT_GT(mesh->StiffnessAt(0.0), 1.3 * mesh->StiffnessAt(0.5));
  EXPECT_GT(mesh->StiffnessAt(0.5), 1e8);
  EXPECT_LT(mesh->StiffnessAt(0.5), 1e9);
}

TEST(GearTeeth, ARingMeshesAndWhatDoesNotMeshIsRefused) {
  GearTeeth teeth;
  teeth.youngs_modulus = 2.06e11;
  const double phi = 20.0 * M_PI / 180.0;
  // 20 teeth in 60 would interfere with full depth teeth; 30 in 90 do not
  EXPECT_THROW(teeth.Configure(0.02, 0.06, 20, 60, phi, 0.002, true),
               std::exception);
  teeth.Configure(0.03, 0.09, 30, 90, phi, 0.002, true);
  EXPECT_GT(teeth.get_contact_ratio(), 1.6); // internal pairs overlap more
  EXPECT_GT(teeth.MeshStiffness(0.5, 0.02), 1e8);
  // the teeth of a tiny pinion reach inside the wheel's base circle
  EXPECT_THROW(teeth.Configure(0.006, 0.1, 6, 100, phi, 0.002, false),
               std::exception);
  shift::core::Context context;
  EXPECT_THROW(Load(context, R"("tooth_geometry": {"youngs_modulus": 2e11})", 0.1),
               std::exception); // no face_width
}

TEST(GearMeshOil, TheSqueezeFilmIsThatOfTheReynoldsEquation) {
  // the gears crossing their backlash: a direct integration of the pressure
  // between two cylinders, p' = 12 mu v x / h^3, against the module's force
  const double mu = 0.05, width = 0.02, omega = 2.0;
  const double phi = 20.0 * M_PI / 180.0;
  std::ostringstream options;
  options << R"("tooth_stiffness": 1e8, "contact_ratio": 1.0, "ramp": 0,
                "backlash": 1e-3, "smoothing": 1e-9, "face_width": )"
          << width << R"(, "oil_viscosity": )" << mu;
  shift::core::Context context;
  Load(context, options.str(), 0.1, "", 0.0, 0.0, 0.0);
  GearMesh* mesh = Find(context);
  auto joint = context["assemblies"]["test"]["joints"]["turn_1"]
                 .get_data<shift::core::joint::Base>();
  context.get_dq()[joint->get_dq_index()] = omega;
  Evaluate(context, mesh, 0.0, 0.0);
  const double closing = omega * kR1 * std::cos(phi);
  const double gap = 0.5 * std::cos(phi) * 1e-3 + 1e-6;
  const double r_1 = kR1 * std::sin(phi), r_2 = kR2 * std::sin(phi);
  const double radius = r_1 * r_2 / (r_1 + r_2);
  double integral = 0.0;
  const double dx = 1e-5;
  for(double x = -0.5 * dx + 0.2; x > -0.2; x -= dx) {
    integral += x * x / std::pow(gap + x * x / (2.0 * radius), 3.0) * dx;
  }
  const double force = 12.0 * mu * closing * width * integral;
  EXPECT_NEAR(mesh->get_squeeze_force(), force, 0.01 * force);
  // and the flank that opens has none: only the one closing
  EXPECT_NEAR(mesh->get_force_plus(), force, 0.01 * force);
  EXPECT_EQ(mesh->get_force_minus(), 0.0);
}
