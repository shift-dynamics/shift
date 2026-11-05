/* The journal bearing force module (force_module::JournalBearing): the film
integrals against a direct integration of the pressure of the oil film, the
classical results of the short bearing (the forces, the attitude angle and the
load), Petroff's friction, the damping of the squeeze, and a shaft in
equilibrium under its weight */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "shift/core/context.h"
#include "shift/core/force_module/journal_bearing.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::JournalBearing;

static const double kRadius = 0.025, kLength = 0.025, kClearance = 5e-5;

/* a shaft that can move across the bearing axis (z, down) and spin about
it: two prismatic joints and a revolute joint; the bearing is the fixed body.
gravity is along the x axis */
static void Load(shift::core::Context& context, double viscosity, double mass,
                 double time_span, const std::string& extra = "",
                 double spin = 0.0) {
  const std::string path = testing::TempDir() + "/journal_bearing_test.json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [9.81, 0.0, 0.0],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.0005,
                     "relative_error": 1e-8, "absolute_error": 1e-11}]
  },
  "assemblies": [{"name": "test",
    "bodies": [
      {"name": "carriage_x", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1e-3, "Ixx": 1e-6, "Iyy": 1e-6, "Izz": 1e-6,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "carriage_y", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1e-3, "Ixx": 1e-6, "Iyy": 1e-6, "Izz": 1e-6,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}},
      {"name": "shaft", "center_of_mass": [0, 0, 0],
       "inertia": {"mass": )"
         << mass << R"(, "Ixx": 1, "Iyy": 1, "Izz": 1000,
                   "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [
      {"name": "slide_x", "type": "prismatic",
       "parent": [["fixed"], "origin"], "child": [["carriage_x"], "origin"],
       "axis": [1, 0, 0]},
      {"name": "slide_y", "type": "prismatic",
       "parent": [["carriage_x"], "origin"],
       "child": [["carriage_y"], "origin"], "axis": [0, 1, 0]},
      {"name": "spin", "type": "revolute",
       "parent": [["carriage_y"], "origin"], "child": [["shaft"], "origin"],
       "axis": [0, 0, 1], "initial_velocity": )"
         << spin << R"(}],
    "force_modules": [{"type": "journal_bearing", "name": "bearing",
      "journal": ["shaft", "origin"], "bearing": ["fixed", "origin"],
      "radius": )" << kRadius << R"(, "length": )" << kLength
         << R"(, "clearance": )" << kClearance << R"(, "viscosity": )"
         << viscosity << extra << R"(}]
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

static JournalBearing* Find(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto bearing = dynamic_cast<JournalBearing*>(module)) {
      return bearing;
    }
  }
  return nullptr;
}

/* the shaft at (x, y) from the center, moving at (vx, vy) and spinning at
omega, with the forces evaluated */
static void Evaluate(shift::core::Context& context, double x, double y,
                     double vx, double vy, double omega) {
  auto joint = [&](const char* name) {
    return context["assemblies"]["test"]["joints"][name]
      .get_data<shift::core::joint::Base>();
  };
  context.get_q()[joint("slide_x")->get_q_index()] = x;
  context.get_q()[joint("slide_y")->get_q_index()] = y;
  context.get_dq()[joint("slide_x")->get_dq_index()] = vx;
  context.get_dq()[joint("slide_y")->get_dq_index()] = vy;
  context.get_dq()[joint("spin")->get_dq_index()] = omega;
  context.UpdateDHVA();
  std::vector<double> state(
    context.get_state_count() + context.get_auxiliary_state_count(), 0.0);
  std::vector<double> dy(state.size(), 0.0);
  context.SetForces(0.0, state, dy);
}

/* a direct integration of the pressure of the film over the journal, by the
midpoint rule on the angle: the integrals I_1 and I_2 (see
JournalBearing) with the pressure where it is positive only */
static void BruteForceIntegrals(double epsilon, double s, double q, double* i_1,
                                double* i_2) {
  const int n = 400000;
  double sum_1 = 0.0, sum_2 = 0.0;
  for(int k = 0; k < n; k++) {
    const double theta = 2.0 * M_PI * (k + 0.5) / n;
    const double film = 1.0 + epsilon * std::cos(theta);
    const double g = std::max(0.0, s * std::sin(theta) - q * std::cos(theta)) /
                     (film * film * film);
    sum_1 += g * std::cos(theta);
    sum_2 += g * std::sin(theta);
  }
  *i_1 = sum_1 * 2.0 * M_PI / n;
  *i_2 = sum_2 * 2.0 * M_PI / n;
}

TEST(JournalBearing, TheFilmIntegralsAgreeWithADirectIntegrationOfThePressure) {
  // random eccentricities, rotations and squeeze, from a fixed seed
  unsigned int seed = 12345u;
  auto uniform = [&]() {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) / 16777216.0;
  };
  for(int trial = 0; trial < 60; trial++) {
    const double epsilon = 0.97 * uniform();
    const double s = (2.0 * uniform() - 1.0) * 3.0;
    const double q = (2.0 * uniform() - 1.0) * 3.0;
    double a_1, a_2, b_1, b_2;
    JournalBearing::FilmIntegrals(epsilon, s, q, &a_1, &a_2);
    BruteForceIntegrals(epsilon, s, q, &b_1, &b_2);
    const double scale = std::fabs(b_1) + std::fabs(b_2) + 1e-6;
    EXPECT_NEAR(a_1, b_1, 2e-5 * scale)
      << epsilon << " " << s << " " << q;
    EXPECT_NEAR(a_2, b_2, 2e-5 * scale)
      << epsilon << " " << s << " " << q;
  }
  double i_1, i_2;
  JournalBearing::FilmIntegrals(0.5, 0.0, 0.0, &i_1, &i_2);
  EXPECT_EQ(i_1, 0.0);
  EXPECT_EQ(i_2, 0.0);
}

TEST(JournalBearing, ASteadyFilmGivesTheClassicalShortBearingResults) {
  const double mu = 0.01, omega = 100.0;
  shift::core::Context context;
  Load(context, mu, 10.0, 0.0);
  JournalBearing* bearing = Find(context);
  ASSERT_NE(bearing, nullptr);
  const double k = mu * kRadius * std::pow(kLength, 3) * omega /
                   (kClearance * kClearance);
  for(double epsilon : {0.05, 0.3, 0.6, 0.9}) {
    SCOPED_TRACE(epsilon);
    // the shaft displaced along x: the line of centers is x, across it y
    Evaluate(context, epsilon * kClearance, 0.0, 0.0, 0.0, omega);
    const double radial =
      -k * epsilon * epsilon / std::pow(1.0 - epsilon * epsilon, 2.0);
    const double tangential =
      k * M_PI * epsilon / (4.0 * std::pow(1.0 - epsilon * epsilon, 1.5));
    EXPECT_NEAR(bearing->get_radial_force(), radial,
                1e-9 * (std::fabs(radial) + std::fabs(tangential)));
    EXPECT_NEAR(bearing->get_tangential_force(), tangential,
                1e-9 * (std::fabs(radial) + std::fabs(tangential)));
    // in the fixed frame
    EXPECT_NEAR(bearing->get_force().x, radial,
                1e-9 * (std::fabs(radial) + std::fabs(tangential)));
    EXPECT_NEAR(bearing->get_force().y, tangential,
                1e-9 * (std::fabs(radial) + std::fabs(tangential)));
    // the attitude angle: tan(phi) = pi sqrt(1 - e^2) / (4 e)
    EXPECT_NEAR(tangential / -radial,
                M_PI * std::sqrt(1.0 - epsilon * epsilon) / (4.0 * epsilon),
                1e-9 * tangential / -radial);
    // and the load: W = mu R L^3 w e sqrt(16 e^2 + pi^2 (1 - e^2)) /
    // (4 c^2 (1 - e^2)^2)
    const double load = k * epsilon *
                        std::sqrt(16.0 * epsilon * epsilon +
                                  M_PI * M_PI * (1.0 - epsilon * epsilon)) /
                        (4.0 * std::pow(1.0 - epsilon * epsilon, 2.0));
    EXPECT_NEAR(bearing->get_force().x * bearing->get_force().x +
                  bearing->get_force().y * bearing->get_force().y,
                load * load, 1e-9 * load * load);
    EXPECT_NEAR(bearing->get_min_film_thickness(),
                kClearance * (1.0 - epsilon), 1e-15);
    // the oil leaves the ends at R c L eps w
    EXPECT_NEAR(bearing->get_side_flow(),
                kRadius * kClearance * kLength * epsilon * omega,
                1e-9 * kRadius * kClearance * kLength * epsilon * omega);
  }
  // turning the other way, the tangential force reverses
  Evaluate(context, 0.5 * kClearance, 0.0, 0.0, 0.0, -omega);
  EXPECT_LT(bearing->get_tangential_force(), 0.0);
  EXPECT_LT(bearing->get_radial_force(), 0.0);
  // and the force follows the line of centers round the bearing
  Evaluate(context, 0.5 * kClearance * std::cos(0.7),
           0.5 * kClearance * std::sin(0.7), 0.0, 0.0, omega);
  const double radial = bearing->get_radial_force();
  const double tangential = bearing->get_tangential_force();
  EXPECT_NEAR(bearing->get_force().x,
              radial * std::cos(0.7) - tangential * std::sin(0.7),
              1e-9 * std::fabs(radial));
  EXPECT_NEAR(bearing->get_force().y,
              radial * std::sin(0.7) + tangential * std::cos(0.7),
              1e-9 * std::fabs(radial));
}

TEST(JournalBearing, PetroffsFrictionAndTheWorkOfThePressureForce) {
  const double mu = 0.01, omega = 100.0;
  shift::core::Context context;
  Load(context, mu, 10.0, 0.0);
  JournalBearing* bearing = Find(context);
  ASSERT_NE(bearing, nullptr);
  // concentric: Petroff's T = 2 pi mu R^3 L w / c, against the rotation
  Evaluate(context, 1e-6 * kClearance, 0.0, 0.0, 0.0, omega);
  const double petroff =
    2.0 * M_PI * mu * std::pow(kRadius, 3) * kLength * omega / kClearance;
  EXPECT_NEAR(bearing->get_friction_torque(), -petroff, 1e-4 * petroff);
  // eccentric: the shear of the oil is greater by 1 / sqrt(1 - e^2), and the
  // pressure adds e |F_t| / 2 (the work of the film's force across the line
  // of centers; Hamrock, Fundamentals of Fluid Film Lubrication)
  const double epsilon = 0.6;
  Evaluate(context, epsilon * kClearance, 0.0, 0.0, 0.0, omega);
  const double expected =
    -petroff / std::sqrt(1.0 - epsilon * epsilon) -
    0.5 * kClearance * epsilon * bearing->get_tangential_force();
  EXPECT_NEAR(bearing->get_friction_torque(), expected,
              1e-9 * std::fabs(expected));
  // resisting the rotation, whichever way the shaft turns
  EXPECT_LT(bearing->get_friction_torque(), 0.0);
  Evaluate(context, epsilon * kClearance, 0.0, 0.0, 0.0, -omega);
  EXPECT_GT(bearing->get_friction_torque(), 0.0);
  // the shaft and the bearing turning together shear nothing
  EXPECT_GT(std::fabs(bearing->get_friction_torque()), 0.0);
}

TEST(JournalBearing, TheSqueezeDampsTheMotionOfTheCenter) {
  // concentric (to a millionth): the film's force opposes the velocity of
  // the center, whichever way it goes, by the damping of half a film:
  // C = pi mu R L^3 / (2 c^3)
  const double mu = 0.01, speed = 1e-3;
  shift::core::Context context;
  Load(context, mu, 10.0, 0.0);
  JournalBearing* bearing = Find(context);
  ASSERT_NE(bearing, nullptr);
  const double damping = M_PI * mu * kRadius * std::pow(kLength, 3) /
                         (2.0 * std::pow(kClearance, 3));
  for(const auto& [vx, vy] : {std::pair<double, double>{speed, 0.0},
                              std::pair<double, double>{0.0, speed},
                              std::pair<double, double>{-speed, 0.0},
                              std::pair<double, double>{0.0, -speed}}) {
    Evaluate(context, 1e-9 * kClearance, 0.0, vx, vy, 0.0);
    EXPECT_NEAR(bearing->get_force().x, -damping * vx,
                2e-3 * damping * speed);
    EXPECT_NEAR(bearing->get_force().y, -damping * vy,
                2e-3 * damping * speed);
  }
  // and exactly concentric too
  Evaluate(context, 0.0, 0.0, speed, 0.0, 0.0);
  EXPECT_NEAR(bearing->get_force().x, -damping * speed,
              1e-9 * damping * speed);
}

TEST(JournalBearing, TheShaftTouchingTheBearingIsPushedBackByTheWall) {
  shift::core::Context context;
  Load(context, 0.01, 10.0, 0.0, R"(, "wall_stiffness": 1e8)");
  JournalBearing* bearing = Find(context);
  ASSERT_NE(bearing, nullptr);
  Evaluate(context, 0.995 * kClearance, 0.0, 0.0, 0.0, 0.0);
  const double fluid = bearing->get_force().x;
  Evaluate(context, 1.1 * kClearance, 0.0, 0.0, 0.0, 0.0);
  EXPECT_LT(bearing->get_force().x, fluid - 1e8 * 0.09 * kClearance);
  EXPECT_LT(bearing->get_min_film_thickness(), 0.0);
}

/* the weight of a shaft carried by the film: the shaft settles where the
bearing's load is the weight, at the attitude angle of the short bearing */
TEST(JournalBearing, AShaftUnderItsWeightSettlesWhereTheFilmCarriesIt) {
  const double mu = 0.005, omega = 100.0, mass = 10.0;
  shift::core::Context context;
  Load(context, mu, mass, 0.6, "", omega);
  JournalBearing* bearing = Find(context);
  ASSERT_NE(bearing, nullptr);
  std::vector<double> state;
  context.set_state_observer(
    [&](double, const std::vector<double>& y) { state = y; });
  context.Simulate();
  ASSERT_FALSE(state.empty());
  // the eccentricity at which the load is the weight (W = (m + 2e-3) g)
  const double weight = (mass + 2e-3) * 9.81;
  const double k = mu * kRadius * std::pow(kLength, 3) * omega /
                   (4.0 * kClearance * kClearance);
  auto load = [&](double e) {
    return k * e *
           std::sqrt(16.0 * e * e + M_PI * M_PI * (1.0 - e * e)) /
           std::pow(1.0 - e * e, 2.0);
  };
  double low = 0.01, high = 0.99;
  for(int i = 0; i < 200; i++) {
    const double mid = 0.5 * (low + high);
    (load(mid) < weight ? low : high) = mid;
  }
  const double epsilon = 0.5 * (low + high);
  ASSERT_GT(epsilon, 0.4);
  ASSERT_LT(epsilon, 0.8);
  auto joint = [&](const char* name) {
    return context["assemblies"]["test"]["joints"][name]
      .get_data<shift::core::joint::Base>();
  };
  const double x = state[joint("slide_x")->get_q_index()];
  const double y = state[joint("slide_y")->get_q_index()];
  const double e = std::hypot(x, y) / kClearance;
  EXPECT_NEAR(e, epsilon, 0.01 * epsilon);
  // the journal at rest: the film's force balances the weight
  EXPECT_NEAR(state[context.get_q_count() + joint("slide_x")->get_dq_index()],
              0.0, 1e-4);
  EXPECT_NEAR(bearing->get_force().x, -weight, 0.01 * weight);
  EXPECT_NEAR(bearing->get_force().y, 0.0, 0.02 * weight);
  // the attitude angle between the load (+x) and the line of centers
  const double angle = std::acos(x / std::hypot(x, y));
  EXPECT_NEAR(std::tan(angle),
              M_PI * std::sqrt(1.0 - e * e) / (4.0 * e),
              0.02 * std::tan(angle));
  // the shaft sits off the load line: the film's force across the line of
  // centers carries the load, so the shaft is displaced across it
  EXPECT_NE(y, 0.0);
}

TEST(JournalBearing, WhatItRefuses) {
  auto message = [&](const std::string& extra, double viscosity = 0.01) {
    shift::core::Context context;
    try {
      Load(context, viscosity, 10.0, 0.0, extra);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(message("", -1.0).find("viscosity"), std::string::npos);
  EXPECT_NE(message(R"(, "wall_stiffness": -1)").find("wall_stiffness"),
            std::string::npos);
  EXPECT_EQ(message(""), "");
}
