/* Stick-slip friction of the contact module (the bristle model of pairs
listed in stick_slip): a box on an inclined plane holds below the limit of
static friction where regularized Coulomb friction lets it creep, with the
deflection of the bristles that carries the load, slides at the acceleration of
Coulomb friction above it, and holds where the static friction is above the
Coulomb friction */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "test/contact/contact_record.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/force_module/contact.h"

static const double kGravity = 9.81;

struct Incline {
  double tan_slope = 0.2;     //!< tangent of the angle of the plane
  double friction = 0.5;      //!< Coulomb friction of both materials
  std::string stick_slip = ""; //!< the stick_slip array of the contact module
  std::string stick_slip_all = ""; //!< the stick_slip_all of the contact module
  double lift = 0.0; //!< speed off the plane at the start
  double rock = 0.0; //!< angular speed about the y axis at the start
  double time_span = 1.0;
};

/* the center of the box at the start: 0.1 m from the plane less the depth
that the four springs (the stiffnesses of the two materials in series, 5e4
each) press in under the load, so that it starts at rest in equilibrium
against the plane, with the load already on */
static ksl_vec3_t Start(double tan_slope) {
  const double angle = std::atan(tan_slope);
  const double c = std::cos(angle), s = std::sin(angle);
  const double depth = 10.0 * kGravity * c / (4.0 * 5e4);
  return {{-(0.1 - depth) * s, 0.0, -(0.1 - depth) * c}};
}

/* a box (a convex hull, resting on four springs) on a plane, tilted about the
y axis by the angle of the slope (z down, up is -z) */
static void RunIncline(shift::core::Context& context, const Incline& incline,
                std::vector<double>* last_state = nullptr) {
  const double angle = std::atan(incline.tan_slope);
  const double c = std::cos(angle), s = std::sin(angle);
  const std::string path = testing::TempDir() + "/stick_slip_test.json";
  std::ostringstream rotation;
  rotation.precision(17);
  rotation << R"("R": [[)" << c << ", 0, " << s << "], [0, 1, 0], [" << -s
           << ", 0, " << c << "]]";
  std::ostringstream model;
  model.precision(17);
  model << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, )" << kGravity << R"(],
    "enable_contact_detection": true,
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "relative_error": 1e-8,
                     "absolute_error": 1e-10, "time_span": )"
        << incline.time_span << R"(, "time_step": 0.001}]
  },
  "assemblies": [{"name": "incline",
    "bodies": [
      {"name": "ground", "geometry": [],
       "collision_geometry": [{"name": "ground", "type": "plane",
                               "material": "surface"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 1.0, "Ixx": 1.0, "Iyy": 1.0, "Izz": 1.0,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}},
      {"name": "box", "geometry": [],
       "collision_geometry": [{"name": "box", "type": "convex_hull",
                               "half_extents": [0.1, 0.1, 0.1],
                               "material": "block"}],
       "center_of_mass": [0, 0, 0],
       "inertia": {"mass": 10.0, "Ixx": 0.067, "Iyy": 0.067, "Izz": 0.067,
                   "Ixy": 0.0, "Iyz": 0.0, "Izx": 0.0}}],
    "joints": [
      {"name": "ground_joint", "type": "rigid",
       "parent": [["fixed"], "origin"], "child": [["ground"], "origin"],
       "initial_displacement": {"T": [0, 0, 0], )" << rotation.str() << R"(}},
      {"name": "box_joint", "type": "float",
       "parent": [["fixed"], "origin"], "child": [["box"], "origin"],
       "initial_displacement": {"T": [)" << Start(incline.tan_slope).x << ", 0, "
        << Start(incline.tan_slope).z << R"(], )" << rotation.str() << R"(},
       "initial_velocity": [0, 0, )" << -incline.lift << R"(, 0, )"
        << incline.rock << R"(, 0]}],
    "force_modules": [{"name": "contact", "type": "contact",
      "friction_velocity": 0.01, )"
        << (incline.stick_slip.empty() ? ""
                                       : "\"stick_slip\": [" +
                                           incline.stick_slip + "],")
        << (incline.stick_slip_all.empty() ? ""
                                           : "\"stick_slip_all\": " +
                                               incline.stick_slip_all + ",")
        << R"(
      "materials": [
        {"name": "surface", "restitution": 0.1, "friction": )"
        << incline.friction << R"(, "contact_stiffness": 1e5},
        {"name": "block", "restitution": 0.1, "friction": )"
        << incline.friction << R"(, "contact_stiffness": 1e5}]}]
  }]
})";
  {
    std::ofstream file(path);
    file << model.str();
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
  if(last_state) {
    context.set_state_observer(
      [last_state](double, const std::vector<double>& y) { *last_state = y; });
  }
  context.Simulate();
  shift::test::RecordContacts(context);
}

static shift::core::body::Base* Box(shift::core::Context& context) {
  for(auto body : context.get_active_articulated_bodies()) {
    if(body->get_name() == "box") {
      return body;
    }
  }
  return nullptr;
}

/* the distance moved down the slope from the start, and the speed */
static double Downhill(shift::core::Context& context, double tan_slope) {
  const double angle = std::atan(tan_slope);
  const ksl_vec3_t p = context.get_body_origin_pose(Box(context)).t;
  const double c = std::cos(angle), s = std::sin(angle);
  const ksl_vec3_t start = Start(tan_slope);
  return -c * (p.x - start.x) + s * (p.z - start.z);
}

static double Speed(shift::core::Context& context) {
  const ksl_screw_t v =
    context.GetBodyVelocity(context.GetBodyLoadIndex(Box(context)));
  return std::sqrt(v.lin.x * v.lin.x + v.lin.y * v.lin.y + v.lin.z * v.lin.z);
}

static shift::core::force_module::Contact* ContactModule(
  shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto contact = dynamic_cast<shift::core::force_module::Contact*>(module)) {
      return contact;
    }
  }
  return nullptr;
}

static const char* kHolding = R"({"geometry_a": "ground", "geometry_b": "box",
  "bristle_stiffness": 2000, "bristle_damping": 5, "static_friction": 0.6,
  "stribeck_velocity": 0.05})";

TEST(StickSlip, AHeldBodyDoesNotCreepWhereRegularizedFrictionDoes) {
  // a slope of 0.2 under friction of 0.5: the friction needed, 0.2 of the
  // load, is 0.4 of the friction there is. Regularized Coulomb friction gives
  // 0.4 mu at a sliding speed of 0.4 / sqrt(1 - 0.4^2) of friction_velocity,
  // so the box creeps at 4.4 mm/s; bristles hold it
  Incline creeping;
  creeping.time_span = 1.5;
  shift::core::Context regular;
  RunIncline(regular, creeping);
  EXPECT_NEAR(Speed(regular), 0.01 * 0.4 / std::sqrt(1.0 - 0.16), 2e-4);
  EXPECT_GT(Downhill(regular, creeping.tan_slope), 3e-3);

  Incline holding = creeping;
  holding.stick_slip = kHolding;
  shift::core::Context held;
  RunIncline(held, holding);
  EXPECT_LT(Speed(held), 1e-6);
  // held for good: the same place half a second later
  Incline later = holding;
  later.time_span = 2.0;
  shift::core::Context held_later;
  RunIncline(held_later, later);
  EXPECT_NEAR(Downhill(held, holding.tan_slope),
              Downhill(held_later, later.tan_slope), 1e-6);
}

TEST(StickSlip, TheDeflectionOfTheBristlesCarriesTheLoad) {
  // at rest the bristles of the four points are all deflected by
  // z = tan(slope) / sigma_0, so that sigma_0 z F is the part of the weight
  // along the slope (the deflections are equal, and the normal forces sum to
  // the part of the weight against the plane)
  Incline holding;
  holding.stick_slip = kHolding;
  holding.time_span = 1.5;
  shift::core::Context context;
  std::vector<double> state;
  RunIncline(context, holding, &state);
  auto contact = ContactModule(context);
  ASSERT_NE(contact, nullptr);
  const size_t offset = contact->get_state_offset();
  ASSERT_GE(state.size(), offset + 3 * 8);
  int deflected = 0;
  for(int k = 0; k < 8; k++) {
    const double* z = &state[offset + 3 * k];
    const double size = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    if(k < 4) {
      // the four corners of the box
      EXPECT_NEAR(size, holding.tan_slope / 2000.0,
                  0.02 * holding.tan_slope / 2000.0)
        << k;
      deflected++;
    } else {
      EXPECT_EQ(size, 0.0) << k; // no more points
    }
  }
  EXPECT_EQ(deflected, 4);
  // the output: how near the limit of holding, tan(slope) / mu_s
  contact->UpdateOutputs(0.0, state.data());
  std::vector<double> outputs;
  for(const auto& key : contact->get_output_keys()) {
    outputs.push_back(contact->GetOutput(key));
  }
  const auto names = contact->get_output_keys();
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "stick_ground_box");
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_NEAR(outputs[0], holding.tan_slope / 0.6, 0.02 * holding.tan_slope / 0.6);
}

TEST(StickSlip, BeyondTheLimitItSlidesAtTheAccelerationOfCoulombFriction) {
  // tan(slope) = 0.8 against friction 0.5 (and no more static friction than
  // that): a = g cos(slope) (tan(slope) - mu)
  Incline sliding;
  sliding.tan_slope = 0.8;
  sliding.stick_slip = R"({"geometry_a": "ground", "geometry_b": "box",
    "bristle_stiffness": 2000, "stribeck_velocity": 0.05})";
  sliding.time_span = 0.6;
  shift::core::Context early;
  RunIncline(early, sliding);
  sliding.time_span = 1.0;
  shift::core::Context late;
  RunIncline(late, sliding);
  const double cos_slope = 1.0 / std::sqrt(1.0 + 0.64);
  const double acceleration = kGravity * cos_slope * (0.8 - 0.5);
  EXPECT_NEAR((Speed(late) - Speed(early)) / 0.4, acceleration,
              0.03 * acceleration);
}

TEST(StickSlip, StaticFrictionAboveTheCoulombFrictionHoldsWhereCoulombSlides) {
  // 0.8 of the load is more than the friction of 0.5 that Coulomb friction
  // gives, but less than the static friction of 0.9: the box holds. The
  // load comes on at once, the bristles take up the sliding that builds it
  // (0.04 mm at sigma_0 = 2e4 per m), which is at a speed well inside the
  // Stribeck velocity
  Incline steep;
  steep.tan_slope = 0.8;
  steep.time_span = 3.0;
  steep.stick_slip = R"({"geometry_a": "ground", "geometry_b": "box",
    "bristle_stiffness": 20000, "bristle_damping": 500,
    "static_friction": 0.9, "stribeck_velocity": 0.5})";
  shift::core::Context held;
  RunIncline(held, steep);
  EXPECT_LT(Speed(held), 1e-6);
  Incline coulomb = steep;
  coulomb.stick_slip = "";
  shift::core::Context slides;
  RunIncline(slides, coulomb);
  EXPECT_GT(Speed(slides), 1.0);
}

TEST(StickSlip, APairNotListedKeepsItsStatelessFriction) {
  Incline plain;
  plain.time_span = 0.1;
  shift::core::Context context;
  RunIncline(context, plain);
  auto contact = ContactModule(context);
  ASSERT_NE(contact, nullptr);
  EXPECT_EQ(contact->get_equation_count(), 0u);
  EXPECT_TRUE(contact->get_output_keys().empty());
}

TEST(StickSlip, WhatItRefuses) {
  auto message = [&](const std::string& pair) {
    Incline incline;
    incline.stick_slip = pair;
    incline.time_span = 0.0;
    shift::core::Context context;
    try {
      RunIncline(context, incline);
    } catch(const std::exception& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(message(R"({"geometry_a": "ground", "geometry_b": "nothing",
                      "bristle_stiffness": 1000})")
              .find("two different collision geometries"),
            std::string::npos);
  EXPECT_NE(message(R"({"geometry_a": "ground", "geometry_b": "box",
                      "bristle_stiffness": 0})")
              .find("bristle_stiffness"),
            std::string::npos);
  EXPECT_NE(message(R"({"geometry_a": "ground", "geometry_b": "box",
                      "bristle_stiffness": 1000, "static_friction": 0.3})")
              .find("static_friction"),
            std::string::npos);
  EXPECT_NE(message(R"({"geometry_a": "ground", "geometry_b": "box",
                      "bristle_stiffness": 1000},
                     {"geometry_a": "box", "geometry_b": "ground",
                      "bristle_stiffness": 1000})")
              .find("twice"),
            std::string::npos);
  EXPECT_EQ(message(R"({"geometry_a": "ground", "geometry_b": "box",
                      "bristle_stiffness": 1000})"),
            "");
}

static const char* kAll = R"({"bristle_stiffness": 2000, "bristle_damping": 5,
  "static_ratio": 1.2, "stribeck_velocity": 0.05, "pairs": 2})";

TEST(StickSlipAll, EveryPairHoldsWithoutBeingListed) {
  // the same bristles as the listed pair of kHolding, with no pair named: the
  // box stays where the listed pair holds it
  Incline slope;
  slope.time_span = 1.5;
  slope.stick_slip = kHolding;
  shift::core::Context listed;
  RunIncline(listed, slope);
  slope.stick_slip = "";
  slope.stick_slip_all = kAll;
  shift::core::Context holding;
  RunIncline(holding, slope);
  EXPECT_NEAR(Downhill(holding, slope.tan_slope),
              Downhill(listed, slope.tan_slope), 1e-8);
  EXPECT_LT(Speed(holding), 1e-6);
  EXPECT_EQ(ContactModule(holding)->get_shared_pairs(), 1);
  const auto names = ContactModule(holding)->get_output_keys();
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "stick_all");
  EXPECT_EQ(names[1], "stick_pairs");
}

TEST(StickSlipAll, ThePairsGetTheFrictionOfTheirMaterials) {
  // past the static limit of 0.6 (0.5 x 1.2), it slides, at the acceleration
  // of the Coulomb friction of the pair
  Incline slope;
  slope.tan_slope = 0.7;
  slope.time_span = 0.5;
  slope.stick_slip_all = kAll;
  shift::core::Context context;
  RunIncline(context, slope);
  const double angle = std::atan(slope.tan_slope);
  const double a =
    kGravity * (std::sin(angle) - 0.5 * std::cos(angle));
  EXPECT_NEAR(Speed(context), a * slope.time_span, 0.05 * a * slope.time_span);
}

TEST(StickSlipAll, AListedPairKeepsItsOwnBristles) {
  Incline slope;
  slope.stick_slip = kHolding;
  slope.stick_slip_all = kAll;
  slope.time_span = 0.2;
  shift::core::Context context;
  RunIncline(context, slope);
  EXPECT_EQ(ContactModule(context)->get_shared_pairs(), 0);
}

TEST(StickSlipAll, ASetGoesBackWhenThePairHasLeft) {
  // the box leaves the plane at once: its pair takes a set, gives it back when
  // the bristles have relaxed, and the set is free at the end
  Incline slope;
  slope.lift = 3.0;
  slope.time_span = 0.25;
  slope.stick_slip_all = kAll;
  shift::core::Context context;
  RunIncline(context, slope);
  EXPECT_GT(Speed(context), 0.5);
  EXPECT_EQ(ContactModule(context)->get_shared_pairs(), 0);
}

TEST(StickSlipAll, WhatItRefuses) {
  Incline slope;
  slope.stick_slip_all = R"({"bristle_stiffness": 2000, "static_ratio": 0.5})";
  shift::core::Context context;
  EXPECT_THROW(RunIncline(context, slope), std::exception);
}

TEST(StickSlip, TheBristleDampingStopsABodyWalkingDownTheSlope) {
  // the friction of a corner is its normal force times the deflection of its
  // bristles, so rocking can feed energy into itself; with bristle_damping a
  // box set rocking settles and stays. Without it the outcome depends on the
  // rounding of the build, so only the damped case is checked
  Incline slope;
  slope.tan_slope = 0.35;
  slope.rock = 1e-3;
  slope.stick_slip = R"({"geometry_a": "ground", "geometry_b": "box",
    "bristle_stiffness": 2000, "bristle_damping": 5, "static_friction": 0.6,
    "stribeck_velocity": 0.05})";
  slope.time_span = 1.0;
  shift::core::Context early;
  RunIncline(early, slope);
  slope.time_span = 4.0;
  shift::core::Context late;
  RunIncline(late, slope);
  EXPECT_NEAR(Downhill(late, slope.tan_slope), Downhill(early, slope.tan_slope),
              1e-7);
}
