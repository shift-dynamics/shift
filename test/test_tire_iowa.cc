/* The real-time tire (force_module::tire::Iowa): the forces against the
expressions of the model, and the contact by slices of the wheel on flat
ground, on a slope and on a step smaller than the contact patch */

#include <gtest/gtest.h>
#include <tiffio.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "ksl/coscrew.h"
#include "shift/core/context.h"
#include "shift/core/force_module/tire/base.h"
#include "shift/core/force_module/tire/bekker_wong.h"
#include "shift/core/force_module/tire/fiala.h"
#include "shift/core/force_module/tire/iowa.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::tire::BekkerWong;
using shift::core::force_module::tire::Contact;
using shift::core::force_module::tire::Fiala;
using shift::core::force_module::tire::Iowa;

static void Configure(Iowa& tire) {
  tire.unloaded_radius = 0.3;
  tire.vertical_stiffness = 2e5;
  tire.vertical_damping = 1000.0;
  tire.friction_coefficient = 0.8;
  tire.rolling_resistance = 0.02;
  tire.cornering_stiffness = 5e4;
  tire.aligning_coefficient = 0.03;
}

/* a wheel 1 cm into the ground, at the speed v, 4 kN of load at rest; the
spin gives the slip: negative rolling forward */
static Contact Rolling(double v_x, double v_y, double slip) {
  Contact c;
  c.in_contact = true;
  c.depth = 0.02;
  c.hub_velocity_w = {{v_x, v_y, 0.0}};
  c.vx = v_x;
  c.omega = -v_x * (1.0 + slip) / (0.3 - 0.02);
  c.spin_axis_w = {{0.0, 1.0, 0.0}};
  c.hub_w = {{0.0, 0.0, -0.28}};
  return c;
}

static ksl_coscrew_t Force(Iowa& tire, const Contact& contact) {
  ksl_coscrew_t force;
  tire.Evaluate(contact, nullptr, nullptr, &force);
  return force;
}

TEST(IowaTire, TheLoadIsASpringAndADamperThatDoNotPull) {
  Iowa tire;
  Configure(tire);
  Contact c = Rolling(0.0, 0.0, 0.0);
  EXPECT_NEAR(Force(tire, c).lin.z, -4000.0, 1e-9); // z down: up on the wheel
  c.hub_velocity_w.z = 1.0; // moving into the ground
  EXPECT_NEAR(Force(tire, c).lin.z, -5000.0, 1e-9);
  c.hub_velocity_w.z = -10.0; // leaving faster than the spring extends
  const ksl_coscrew_t lifted = Force(tire, c);
  EXPECT_EQ(lifted.lin.z, 0.0);
  EXPECT_EQ(lifted.lin.x, 0.0);
  c.in_contact = false;
  EXPECT_EQ(Force(tire, c).lin.z, 0.0);
}

TEST(IowaTire, TheFrictionRisesToItsPeakAndFallsToItsSlidingValue) {
  Iowa tire;
  Configure(tire);
  EXPECT_EQ(tire.LongitudinalFriction(0.0), 0.0);
  EXPECT_NEAR(tire.LongitudinalFriction(0.1), 0.42, 1e-12);
  EXPECT_NEAR(tire.LongitudinalFriction(0.2), 0.84, 1e-12);
  EXPECT_NEAR(tire.LongitudinalFriction(0.25), 0.84, 1e-12);
  EXPECT_NEAR(tire.LongitudinalFriction(0.375), 0.82, 1e-12);
  EXPECT_NEAR(tire.LongitudinalFriction(0.5), 0.8, 1e-12);
  EXPECT_NEAR(tire.LongitudinalFriction(0.9), 0.8, 1e-12);
}

TEST(IowaTire, DrivingPushesForwardAndBrakingBack) {
  Iowa tire;
  Configure(tire);
  // driving at a slip of 10 %: the rim is 10 % faster than the hub, so the
  // slip is 0.1 / 1.1, faded by |v_p| / (v_e + |v_p|)
  const double v = 20.0, tread = 2.0;
  const double slip = tread / 22.0 * tread / (0.0635 + tread);
  const ksl_coscrew_t drive = Force(tire, Rolling(v, 0.0, 0.1));
  EXPECT_NEAR(tire.get_slip(), slip, 1e-12);
  const double rolling = 0.02 * 4000.0 * v / (0.0635 + v);
  EXPECT_NEAR(drive.lin.x, 4000.0 * 5.0 * 0.84 * slip - rolling, 1e-9);
  // locked: the slip is -1 and the tire slides at its sliding friction
  Contact locked = Rolling(v, 0.0, 0.0);
  locked.omega = 0.0;
  const ksl_coscrew_t sliding = Force(tire, locked);
  EXPECT_NEAR(tire.get_slip(), -1.0 * v / (0.0635 + v), 1e-12);
  EXPECT_NEAR(sliding.lin.x, -0.8 * 4000.0 - rolling, 1e-9);
}

TEST(IowaTire, TheSlipsFadeOutAtRest) {
  Iowa tire;
  Configure(tire);
  Force(tire, Rolling(1e-5, 1e-5, 0.0));
  EXPECT_NEAR(tire.get_slip(), 0.0, 1e-9);
  EXPECT_LT(std::fabs(tire.get_slip_angle()), 1e-6);
  const ksl_coscrew_t rest = Force(tire, Rolling(0.0, 0.0, 0.0));
  EXPECT_EQ(rest.lin.x, 0.0);
  EXPECT_EQ(rest.lin.y, 0.0);
}

TEST(IowaTire, TheLateralForceLeavesWithTheCorneringStiffnessAndSaturates) {
  Iowa tire;
  Configure(tire);
  const double limit = 3200.0, stiffness = 5e4;
  EXPECT_NEAR(tire.LateralForce(1e-5, limit), stiffness * 1e-5, 1e-3);
  const double end = 2.5 * limit / stiffness;
  EXPECT_NEAR(tire.LateralForce(end, limit), limit, 1e-9);
  EXPECT_EQ(tire.LateralForce(2.0 * end, limit), limit);
  // no slope where it reaches the limit, and rising up to it
  EXPECT_NEAR(tire.LateralForce(end * (1.0 - 1e-4), limit), limit, 1e-3);
  double last = 0.0;
  for(double alpha = 0.0; alpha <= end; alpha += end / 100.0) {
    const double force = tire.LateralForce(alpha, limit);
    EXPECT_GE(force, last);
    last = force;
  }
  // it opposes the sideways velocity, y to the right
  EXPECT_LT(Force(tire, Rolling(20.0, 1.0, 0.0)).lin.y, 0.0);
  EXPECT_GT(Force(tire, Rolling(20.0, -1.0, 0.0)).lin.y, 0.0);
}

TEST(IowaTire, TheFrictionEllipseLimitsTheForces) {
  Iowa tire;
  Configure(tire);
  tire.rolling_resistance = 0.0;
  const ksl_coscrew_t f = Force(tire, Rolling(20.0, 3.0, 0.3));
  EXPECT_NEAR(std::hypot(f.lin.x, f.lin.y), 0.8 * 4000.0, 1e-6);
  EXPECT_GT(f.lin.x, 100.0);
  EXPECT_LT(f.lin.y, -100.0);
}

TEST(IowaTire, TheAligningMomentActsBehindTheContact) {
  Iowa tire;
  Configure(tire);
  tire.rolling_resistance = 0.0;
  const ksl_coscrew_t f = Force(tire, Rolling(20.0, 0.5, 0.0));
  // the force acts 3 cm behind the point of contact: x f_y, x = -t
  EXPECT_NEAR(f.ang.z, -0.03 * f.lin.y, 1e-9);
  // camber: the load across the wheel acts behind it the same way
  Contact tilted = Rolling(0.0, 0.0, 0.0);
  const double gamma = 0.1;
  tilted.spin_axis_w = {{0.0, std::cos(gamma), std::sin(gamma)}};
  const ksl_coscrew_t c = Force(tire, tilted);
  const double moment = 0.03 * 4000.0 * std::sin(gamma);
  EXPECT_NEAR(c.ang.z, moment * std::cos(gamma), 1e-9);
  EXPECT_NEAR(c.ang.y, -moment * std::sin(gamma), 1e-9);
}

TEST(IowaTire, TheRollingResistanceActsAtTheHub) {
  Iowa tire;
  Configure(tire);
  tire.aligning_coefficient = 0.0;
  const ksl_coscrew_t f = Force(tire, Rolling(20.0, 0.0, 0.0));
  // a force at the hub, 28 cm above the origin of the w frame, is the same
  // force there with the moment arm x f
  EXPECT_LT(f.lin.x, 0.0);
  EXPECT_NEAR(f.ang.y, -0.28 * f.lin.x, 1e-9);
}

TEST(IowaTire, WhatItRefuses) {
  Iowa tire;
  Configure(tire);
  EXPECT_NO_THROW(tire.Prepare());
  tire.friction_coefficient = 0.0;
  EXPECT_THROW(tire.Prepare(), std::exception);
  Configure(tire);
  tire.segments = -1;
  EXPECT_THROW(tire.Prepare(), std::exception);
}

/* the relaxed slip ratio, the state of a tire with a relaxation length */
static ksl_coscrew_t Relaxed(Iowa& tire, const Contact& contact, double kappa,
                             double* rate) {
  ksl_coscrew_t force;
  tire.Evaluate(contact, &kappa, rate, &force);
  return force;
}

TEST(IowaTire, ARelaxationLengthAddsTheRelaxedSlipAsAState) {
  Iowa tire;
  Configure(tire);
  EXPECT_EQ(tire.get_state_count(), 0u);
  tire.relaxation_length = 0.3;
  EXPECT_EQ(tire.get_state_count(), 1u);
  tire.relaxation_length = -0.1;
  EXPECT_THROW(tire.Prepare(), std::exception);
}

TEST(IowaTire, TheRelaxedSlipLagsTheSlipOfTheTread) {
  // l dk'/dt = -v_p - (|v_x| + v_eps) k', from rest of the state and settled
  Iowa tire;
  Configure(tire);
  tire.relaxation_length = 0.3;
  const Contact c = Rolling(10.0, 0.0, 0.1); // driving: the tread slips back
  const double v_p = c.vx + c.omega * (0.3 - 0.02);
  double rate = 0.0;
  const ksl_coscrew_t start = Relaxed(tire, c, 0.0, &rate);
  EXPECT_NEAR(rate, -v_p / 0.3, 1e-9);
  EXPECT_GT(rate, 0.0);
  EXPECT_NEAR(start.lin.x, -0.02 * 4000.0 * 10.0 / (0.0635 + 10.0), 1e-9);
  const double settled = -v_p / (10.0 + 0.0635);
  const ksl_coscrew_t force = Relaxed(tire, c, settled, &rate);
  EXPECT_NEAR(rate, 0.0, 1e-9);
  EXPECT_EQ(tire.get_slip(), settled);
  EXPECT_NEAR(force.lin.x,
              4000.0 * tire.LongitudinalFriction(settled) -
                0.02 * 4000.0 * 10.0 / (0.0635 + 10.0),
              1e-9);
}

TEST(IowaTire, TheRelaxedSlipIsHeldWithinOne) {
  Iowa tire;
  Configure(tire);
  tire.relaxation_length = 0.3;
  const Contact spinning = Rolling(1.0, 0.0, 5.0);
  double rate = 0.0;
  Relaxed(tire, spinning, 1.0, &rate);
  EXPECT_EQ(rate, 0.0);
  Relaxed(tire, spinning, 0.9, &rate);
  EXPECT_GT(rate, 0.0);
  const Contact locked = Rolling(10.0, 0.0, -1.5); // spun backwards
  Relaxed(tire, locked, -1.0, &rate);
  EXPECT_EQ(rate, 0.0);
  Contact flying = locked;
  flying.in_contact = false;
  Relaxed(tire, flying, 0.5, &rate);
  EXPECT_NEAR(rate, -(10.0 + 0.0635) * 0.5 / 0.3, 1e-9);
}

TEST(IowaTire, WithARelaxationLengthTheLongitudinalForceTakesTheFrictionFirst) {
  Iowa tire;
  Configure(tire);
  tire.relaxation_length = 0.3;
  tire.rolling_resistance = 0.0;
  const double limit = 0.8 * 4000.0;
  double rate = 0.0;
  // past the limit alone: the longitudinal force is the limit, no lateral
  const ksl_coscrew_t peak = Relaxed(tire, Rolling(10.0, 3.0, 0.0), 0.2, &rate);
  EXPECT_NEAR(peak.lin.x, limit, 1e-9);
  EXPECT_EQ(peak.lin.y, 0.0);
  EXPECT_EQ(peak.ang.z, 0.0);
  // within it: the lateral force takes what is left
  const ksl_coscrew_t both = Relaxed(tire, Rolling(10.0, 3.0, 0.0), 0.1, &rate);
  const double f_x = 4000.0 * tire.LongitudinalFriction(0.1);
  EXPECT_NEAR(both.lin.x, f_x, 1e-9);
  EXPECT_NEAR(both.lin.y, -std::sqrt(limit * limit - f_x * f_x), 1e-9);
  EXPECT_NEAR(both.ang.z, -0.03 * both.lin.y, 1e-9);
}

TEST(IowaTire, OnSoftSoilTheLoadAndTheResistanceAreBekkers) {
  // W = (3 - n)/3 b k sqrt(d) z^(n + 1/2), r_c = b k z^(n + 1)/(n + 1)
  Iowa tire;
  Configure(tire);
  tire.width = 0.2;
  shift::core::force_module::SoilProperties& soil = tire.soil;
  soil.n = 0.7;
  soil.k_c = 5270.0;
  soil.k_phi = 1515040.0;
  const double k = 5270.0 / 0.2 + 1515040.0;
  const double z = 0.02;
  double load, resistance;
  tire.SoilForces(z, &load, &resistance);
  EXPECT_NEAR(load, 2.3 / 3.0 * 0.2 * k * std::sqrt(0.6) * std::pow(z, 1.2),
              1e-9);
  EXPECT_NEAR(resistance, 0.2 * k * std::pow(z, 1.7) / 1.7, 1e-9);
}

/* a wheel that moves freely along x and z and does not spin */
static void LoadWheel(shift::core::Context& context, const std::string& tire,
                      const std::string& terrain, double x0, double z0,
                      double v0, double time_span,
                      const std::string& type = "iowa") {
  const std::string path = testing::TempDir() + "/iowa_test.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "include_directories": [")"
         << testing::TempDir() << R"("],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.001, "relative_error": 1e-7,
                     "absolute_error": 1e-8}]
  },
  "frames": [{"name": "terrain", "T": [0, 0, 0],
              "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
  "terrain": [)"
         << terrain << R"(],
  "assemblies": [{"name": "rig",
    "force_modules": [{"type": "tire", "name": "tire", "tire_type": ")"
         << type << R"(", "wheel_body": "wheel", ")" << type
         << R"(": {"unloaded_radius": 0.3,
       "vertical_stiffness": 2e5, "vertical_damping": 4000, )"
         << tire << R"(}}],
    "bodies": [
      {"name": "cart", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                   "Iyz": 0, "Izx": 0}},
      {"name": "wheel", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 299, "Ixx": 1, "Iyy": 1, "Izz": 1, "Ixy": 0,
                   "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "x", "type": "prismatic", "parent": [["fixed"], "origin"],
       "child": [["cart"], "origin"], "axis": [1, 0, 0],
       "initial_displacement": )"
         << x0 << R"(, "initial_velocity": )" << v0 << R"(},
      {"name": "z", "type": "prismatic", "parent": [["cart"], "origin"],
       "child": [["wheel"], "origin"], "axis": [0, 0, 1],
       "initial_displacement": )"
         << z0 << R"(}]
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

static shift::core::force_module::tire::Base* Find(
  shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto tire =
         dynamic_cast<shift::core::force_module::tire::Base*>(module)) {
      return tire;
    }
  }
  return nullptr;
}

static std::vector<double> Simulate(shift::core::Context& context) {
  std::vector<double> state;
  context.set_state_observer(
    [&state](double, const std::vector<double>& y) { state = y; });
  context.Simulate();
  return state;
}

static double Position(shift::core::Context& context,
                       const std::vector<double>& y, const char* joint) {
  return y[context["assemblies"]["rig"]["joints"][joint]
             .get_data<shift::core::joint::Base>()
             ->get_q_index()];
}

static double Velocity(shift::core::Context& context,
                       const std::vector<double>& y, const char* joint) {
  return y[context.get_q_count() +
           context["assemblies"]["rig"]["joints"][joint]
             .get_data<shift::core::joint::Base>()
             ->get_dq_index()];
}

/* a single channel float elevation map of the surface z(x) over columns of
x_scale, from x = 0 */
static std::string WriteMap(const std::function<float(double)>& z,
                            double x_scale, int columns) {
  const std::string name = "iowa_map.tiff";
  TIFF* tiff = TIFFOpen((testing::TempDir() + "/" + name).c_str(), "w");
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, columns);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, 8);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP, 1);
  std::vector<float> row(columns);
  for(int c = 0; c < columns; c++) {
    row[c] = z(c * x_scale);
  }
  for(int r = 0; r < 8; r++) {
    TIFFWriteScanline(tiff, row.data(), r, 0);
  }
  TIFFClose(tiff);
  std::ostringstream terrain;
  terrain << R"({"type": "basic", "name": "map", "filename": ")" << name
          << R"(", "parent": [["fixed"], "terrain"], "x_scale": )" << x_scale
          << R"(, "y_scale": 1.0, "z_scale": 1.0,
              "interpolation_type": "bilinear"})";
  return terrain.str();
}

TEST(IowaTireContact, OnFlatGroundTheDeflectionCarriesTheLoad) {
  // 299 kg (the cart rides on the x joint) on a 2e5 N/m tire. The slices find
  // the area of the circular segment of the deflection, so the wheel's center
  // rests about that far below its unloaded height
  shift::core::Context context;
  LoadWheel(context, R"("segments": 40)", "", 0.0, -0.29, 0.0, 1.5);
  const auto state = Simulate(context);
  const double deflection = 299.0 * 9.81 / 2e5;
  EXPECT_NEAR(Find(context)->get_contact().depth, deflection, 1e-5);
  EXPECT_NEAR(Position(context, state, "z"), -(0.3 - deflection), 1e-3);
  const auto& w = Find(context)->get_contact().w_frame;
  EXPECT_NEAR(w.t.x, 0.0, 1e-9);
  EXPECT_NEAR(w.R.v2.z, 1.0, 1e-12);
}

TEST(IowaTireContact, ASinglePointOfContactGivesTheSameRest) {
  shift::core::Context context;
  LoadWheel(context, R"("segments": 0)", "", 0.0, -0.29, 0.0, 1.5);
  const auto state = Simulate(context);
  const double deflection = 299.0 * 9.81 / 2e5;
  EXPECT_NEAR(Position(context, state, "z"), -(0.3 - deflection), 1e-5);
}

TEST(IowaTireContact, ALockedWheelSlidesToAStopAtItsFriction) {
  // a wheel that does not spin slides at the sliding friction of the
  // tire: v = v0 - mu g t
  shift::core::Context context;
  LoadWheel(context,
            R"("segments": 20, "friction_coefficient": 0.5,
               "rolling_resistance": 0)",
            "", 0.0, -0.285, 10.0, 1.0);
  const auto state = Simulate(context);
  EXPECT_NEAR(Velocity(context, state, "x"), 10.0 - 0.5 * 9.81 * 1.0, 0.02);
}

TEST(IowaTireContact, OnASlopeTheNormalIsThatOfTheSlope) {
  // the surface z = -0.1 x rises along x
  shift::core::Context context;
  const std::string map =
    WriteMap([](double x) { return -0.1 * (x - 3.0); }, 0.5, 13);
  LoadWheel(context, R"("segments": 20)", map, 3.0, -0.285, 0.0, 0.01);
  Simulate(context);
  const auto& w = Find(context)->get_contact().w_frame;
  const double size = std::sqrt(1.01);
  EXPECT_NEAR(w.R.v2.x, 0.1 / size, 1e-6);
  EXPECT_NEAR(w.R.v2.y, 0.0, 1e-9);
  EXPECT_NEAR(w.R.v2.z, 1.0 / size, 1e-6);
  std::remove((testing::TempDir() + "/iowa_map.tiff").c_str());
}

TEST(IowaTireContact, AStepSmallerThanThePatchMovesTheForceAndTiltsTheNormal) {
  // ground 2 cm higher ahead of x = 3, ramping over 2 cm: the wheel over the
  // edge is pressed in more ahead of its center, so the point of action is
  // ahead of the hub and the normal leans back from the step; a point of
  // contact under the hub would see only the ground under it
  shift::core::Context context;
  const std::string map = WriteMap(
    [](double x) {
      return x < 2.995 ? 0.0f : (x > 3.015 ? -0.02f : -(x - 2.995f));
    },
    0.01, 601);
  LoadWheel(context, R"("segments": 40)", map, 3.0, -0.285, 0.0, 0.001);
  Simulate(context);
  const Contact& c = Find(context)->get_contact();
  EXPECT_TRUE(c.in_contact);
  EXPECT_GT(c.w_frame.t.x, 3.02);
  EXPECT_GT(c.w_frame.R.v2.x, 0.05);
  // deeper than on the lower ground alone (0.015), not as deep as on the
  // upper (0.035), for the same area
  EXPECT_GT(c.depth, 0.015);
  EXPECT_LT(c.depth, 0.035);
  std::remove((testing::TempDir() + "/iowa_map.tiff").c_str());
}

TEST(IowaTireContact, WithARelaxationLengthALockedWheelStillSlidesAtItsFriction) {
  // the relaxed slip reaches the slip of the sliding tread within a fraction
  // of a relaxation length
  shift::core::Context context;
  LoadWheel(context,
            R"("segments": 20, "friction_coefficient": 0.5,
               "rolling_resistance": 0, "relaxation_length": 0.3)",
            "", 0.0, -0.285, 10.0, 1.0);
  const auto state = Simulate(context);
  const double v = Velocity(context, state, "x");
  EXPECT_NEAR(v, 10.0 - 0.5 * 9.81 * 1.0, 0.05);
  // settled at -v_p / (|v_x| + v_eps), with v_p = v_x for a locked wheel
  EXPECT_NEAR(dynamic_cast<const Iowa*>(Find(context)->get_model())
                ->get_slip(),
              -v / (v + 0.0635), 1e-3);
}

TEST(IowaTireContact, OnSoftSoilTheWheelSinksToBekkersSinkage) {
  // at rest W = (3 - n)/3 b k sqrt(d) z^(n + 1/2), so
  // z = (3 W / ((3 - n) b k sqrt(d)))^(1 / (n + 1/2)); sandy loam
  shift::core::Context context;
  LoadWheel(context,
            R"("segments": 0, "width": 0.235,
               "soil": {"name": "sandy_loam"})",
            "", 0.0, -0.29, 0.0, 1.5);
  const auto state = Simulate(context);
  const double n = 0.7, k = 5270.0 / 0.235 + 1515040.0;
  const double sinkage =
    std::pow(3.0 * 299.0 * 9.81 / ((3.0 - n) * 0.235 * k * std::sqrt(0.6)),
             1.0 / (n + 0.5));
  EXPECT_NEAR(Find(context)->get_contact().depth, sinkage, 1e-5);
  EXPECT_NEAR(Position(context, state, "z"), -(0.3 - sinkage), 1e-4);
}

TEST(TireParameters, EachModelReadsTheBlockOfItsName) {
  {
    shift::core::Context context;
    LoadWheel(context, R"("cslip": 40000, "u_max": 0.7)", "", 0.0, -0.29, 0.0,
              0.1, "fiala");
    const auto* tire = dynamic_cast<const Fiala*>(Find(context)->get_model());
    ASSERT_NE(tire, nullptr);
    EXPECT_EQ(tire->unloaded_radius, 0.3);
    EXPECT_EQ(tire->vertical_stiffness, 2e5);
    EXPECT_EQ(tire->cslip, 40000.0);
    EXPECT_EQ(tire->u_max, 0.7);
    EXPECT_EQ(tire->width, 0.235); // a default
  }
  {
    shift::core::Context context;
    LoadWheel(context,
              R"("soil_damping": 50, "max_stress_angle": 0.35, "strips": 3,
                 "bulldozing": false, "soil": {"name": "lete_sand"})",
              "", 0.0, -0.29, 0.0, 0.1, "bekker_wong");
    const auto* tire =
      dynamic_cast<const BekkerWong*>(Find(context)->get_model());
    ASSERT_NE(tire, nullptr);
    EXPECT_EQ(tire->radius, 0.3);
    EXPECT_EQ(tire->vertical_damping, 50.0);
    EXPECT_EQ(tire->c1, 0.35);
    EXPECT_EQ(tire->strips, 3);
    EXPECT_FALSE(tire->bulldozing);
    EXPECT_EQ(tire->soil.n, 0.79); // from the table
  }
  {
    shift::core::Context context;
    LoadWheel(context, R"("segments": 6, "aligning_coefficient": 0.04)", "",
              0.0, -0.29, 0.0, 0.1);
    const auto* tire = dynamic_cast<const Iowa*>(Find(context)->get_model());
    ASSERT_NE(tire, nullptr);
    EXPECT_EQ(tire->segments, 6);
    EXPECT_EQ(tire->aligning_coefficient, 0.04);
    EXPECT_EQ(tire->vertical_stiffness, 2e5);
  }
}

TEST(TireParameters, AParameterBesideTheBlockIsRefusedNotIgnored) {
  shift::core::Context context;
  try {
    LoadWheel(context,
              R"("segments": 0}, "vertical_stiffness": 1, "fiala": {"width": 0.2)",
              "", 0.0, -0.29, 0.0, 0.1);
    ADD_FAILURE() << "loaded";
  } catch(const std::exception& e) {
    EXPECT_NE(std::string(e.what()).find("vertical_stiffness"),
              std::string::npos);
  }
}
