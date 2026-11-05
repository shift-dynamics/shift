/* Belt force module (force_module::Belt): the geometry of the spans, the
tensions from the turns of the pulleys, a slack span, the rates of the
stretches, the friction limit of slip, and a drive spinning up a pulley */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/belt.h"
#include "shift/core/joint/base.h"

using shift::core::force_module::Belt;

static const double kR0 = 0.1, kR1 = 0.05, kD = 0.5, kEA = 1e5;

/* pulleys p0 at the origin and p1 at [D, 0, 0] on revolute joints J0 and J1
about the fixed z axis; with a third pulley, p2 turns on a slider moving
along y, below the line between them, where a pulley of radius 0.03 on
the back of the belt presses into the lower span */
static void LoadBelt(shift::core::Context& context, const std::string& name,
                     const std::string& belt, bool third = false,
                     const std::string& joint_0 = "",
                     double time_span = 0.01) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  auto body = [](const std::string& name, double izz) {
    return R"({"name": ")" + name + R"(", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 1, "Ixx": 0.01, "Iyy": 0.01, "Izz": )" +
           std::to_string(izz) + R"(, "Ixy": 0, "Iyz": 0, "Izx": 0}})";
  };
  auto revolute = [](const std::string& name, const std::string& parent,
                     const std::string& child, const std::string& extra) {
    return R"({"name": ")" + name + R"(", "type": "revolute",
      "parent": )" + parent + R"(, "child": [[")" + child +
           R"("], "origin"], "axis": [0, 0, 1])" + extra + "}";
  };
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [{"type": "ode113", "time_span": )"
         << time_span << R"(, "time_step": 0.01,
                     "relative_error": 1e-9, "absolute_error": 1e-11}]
  },
  "frames": [{"name": "p1_home", "T": [)"
         << kD << R"(, 0, 0]},
             {"name": "p2_home", "T": [0.25, -0.1, 0]}],
  "assemblies": [{"name": "test",
    "bodies": [)"
         << body("p0", 0.01) << ", " << body("p1", 0.002);
    if(third) {
      file << ", " << body("slider", 0.01) << ", " << body("p2", 0.001);
    }
    file << R"(],
    "joints": [)"
         << revolute("J0", R"([["fixed"], "origin"])", "p0", joint_0) << ", "
         << revolute("J1", R"([["fixed"], "p1_home"])", "p1", "");
    if(third) {
      file << R"(, {"name": "S", "type": "prismatic",
        "parent": [["fixed"], "p2_home"], "child": [["slider"], "origin"],
        "axis": [0, 1, 0]}, )"
           << revolute("J2", R"([["slider"], "origin"])", "p2", "");
    }
    file << R"(],
    "force_modules": [{"type": "belt", "name": "belt", )"
         << belt << R"(}]
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

static Belt* FindBelt(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto belt = dynamic_cast<Belt*>(module)) {
      return belt;
    }
  }
  return nullptr;
}

static const shift::core::joint::Base* FindJoint(
  shift::core::Context& context, const std::string& name) {
  for(auto joint : context.get_active_joints()) {
    if(joint->get_name() == name) {
      return joint;
    }
  }
  return nullptr;
}

/* evaluate the belt at the state y = [q, dq, the belt's states], returning
the rates */
static std::vector<double> Evaluate(shift::core::Context& context,
                                    const std::vector<double>& y) {
  const size_t q_count = context.get_q_count();
  for(size_t i = 0; i < q_count; i++) {
    context.get_q()[i] = y[i];
    context.get_dq()[i] = y[q_count + i];
  }
  context.UpdateDHVA();
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
  return dy;
}

static std::vector<double> InitialState(shift::core::Context& context) {
  std::vector<double> y(context.get_state_count() +
                          context.get_auxiliary_state_count(),
                        0.0);
  context.InitializeAuxiliaryStates(y.data() + context.get_state_count());
  return y;
}

static const char* kBelt = R"("frames": [["p0", "origin"], ["p1", "origin"]],
  "radii": [0.1, 0.05], "axial_stiffness": 1e5, "pretension": 100)";

/* the spans are the outer tangents of the two pulleys; at the start each
carries the pretension, and the larger pulley has the larger wrap */
TEST(Belt, SpansAreTheTangentsAndCarryThePretension) {
  shift::core::Context context;
  LoadBelt(context, "belt_geometry", kBelt);
  Belt* belt = FindBelt(context);
  ASSERT_NE(belt, nullptr);
  ASSERT_EQ(context.get_auxiliary_state_count(), 2u);
  Evaluate(context, InitialState(context));
  const double length = std::sqrt(kD * kD - (kR0 - kR1) * (kR0 - kR1));
  const double extra = 2.0 * std::asin((kR0 - kR1) / kD);
  for(int k = 0; k < 2; k++) {
    EXPECT_NEAR(belt->get_span_lengths()[k], length, 1e-14);
    EXPECT_NEAR(belt->get_tensions()[k], 100.0, 1e-9);
  }
  EXPECT_NEAR(belt->get_wrap_angles()[0], M_PI + extra, 1e-12);
  EXPECT_NEAR(belt->get_wrap_angles()[1], M_PI - extra, 1e-12);
  EXPECT_EQ(belt->get_output_names(),
            (std::vector<std::string>{"span_0_tension", "span_1_tension"}));
}

/* turning the first pulley forward by theta feeds r theta of belt into the
span leaving it, which slackens, and takes as much from the other */
TEST(Belt, TurningAPulleyMovesTensionBetweenTheSpans) {
  shift::core::Context context;
  LoadBelt(context, "belt_turn", kBelt);
  Belt* belt = FindBelt(context);
  ASSERT_NE(belt, nullptr);
  std::vector<double> y = InitialState(context);
  const double theta = 2e-3;
  y[FindJoint(context, "J0")->get_q_index()] = theta;
  y[context.get_state_count()] = theta; /* the turn of pulley 0 */
  Evaluate(context, y);
  const double length = belt->get_span_lengths()[0];
  const double e0 = 100.0 * length / (kEA + 100.0);
  const double e_leaving = e0 - kR0 * theta, e_arriving = e0 + kR0 * theta;
  EXPECT_NEAR(belt->get_stretches()[0], e_leaving, 1e-15);
  EXPECT_NEAR(belt->get_stretches()[1], e_arriving, 1e-15);
  EXPECT_NEAR(belt->get_tensions()[0],
              kEA * e_leaving / (length - e_leaving), 1e-9);
  EXPECT_NEAR(belt->get_tensions()[1],
              kEA * e_arriving / (length - e_arriving), 1e-9);
}

/* without pretension, the span that the turn shortens goes slack */
TEST(Belt, ASlackSpanCarriesNoForce) {
  shift::core::Context context;
  LoadBelt(context, "belt_slack", R"("frames": [["p0", "origin"],
    ["p1", "origin"]], "radii": [0.1, 0.05], "axial_stiffness": 1e5)");
  Belt* belt = FindBelt(context);
  ASSERT_NE(belt, nullptr);
  std::vector<double> y = InitialState(context);
  const double theta = 1e-3;
  y[FindJoint(context, "J0")->get_q_index()] = theta;
  y[context.get_state_count()] = theta;
  Evaluate(context, y);
  EXPECT_EQ(belt->get_tensions()[0], 0.0);
  const double length = belt->get_span_lengths()[1];
  const double e = kR0 * theta;
  EXPECT_NEAR(belt->get_tensions()[1], kEA * e / (length - e), 1e-9);
}

/* the stretch rates and the turn rates match differences of the stretches
along the motion, with a tensioner pulley moving across the belt */
TEST(Belt, StretchRatesMatchTheMotion) {
  shift::core::Context context;
  LoadBelt(context, "belt_rates", R"("frames": [["p0", "origin"],
    ["p2", "origin"], ["p1", "origin"]], "radii": [0.1, -0.03, 0.05],
    "axial_stiffness": 1e5, "pretension": 100)",
           true);
  Belt* belt = FindBelt(context);
  ASSERT_NE(belt, nullptr);
  std::vector<double> y = InitialState(context);
  const size_t q_count = context.get_q_count();
  const size_t states = context.get_state_count();
  const struct {
    const char* joint;
    double q, dq;
  } motion[] = {{"J0", 0.3, 4.0}, {"J1", -0.2, 7.0}, {"S", 0.02, 0.5},
                {"J2", 1.0, -3.0}};
  for(const auto& m : motion) {
    const auto joint = FindJoint(context, m.joint);
    ASSERT_NE(joint, nullptr);
    y[joint->get_q_index()] = m.q;
    y[q_count + joint->get_dq_index()] = m.dq;
  }
  y[states + 0] = 0.25; /* turns of the pulleys */
  y[states + 1] = 0.9;
  y[states + 2] = -0.15;
  const std::vector<double> dy = Evaluate(context, y);
  const std::vector<double> rates = belt->get_stretch_rates();
  const double h = 1e-6;
  std::vector<double> plus = y, minus = y;
  for(const auto& m : motion) {
    const auto joint = FindJoint(context, m.joint);
    plus[joint->get_q_index()] += h * m.dq;
    minus[joint->get_q_index()] -= h * m.dq;
  }
  for(size_t i = states; i < y.size(); i++) {
    plus[i] += h * dy[i];
    minus[i] -= h * dy[i];
  }
  Evaluate(context, plus);
  const std::vector<double> e_plus = belt->get_stretches();
  Evaluate(context, minus);
  const std::vector<double> e_minus = belt->get_stretches();
  for(size_t k = 0; k < 3; k++) {
    const double difference = (e_plus[k] - e_minus[k]) / (2.0 * h);
    EXPECT_NEAR(rates[k], difference, 1e-7) << "span " << k;
    EXPECT_GT(std::fabs(rates[k]), 0.01) << "span " << k;
  }
}

/* with friction, the belt holds a tension ratio up to exp(mu phi) on each
pulley and slips beyond it, toward the tight side */
TEST(Belt, SlipStartsAtTheFrictionLimit) {
  shift::core::Context context;
  LoadBelt(context, "belt_slip", R"("frames": [["p0", "origin"],
    ["p1", "origin"]], "radii": [0.1, 0.05], "axial_stiffness": 1e5,
    "pretension": 100, "friction_coefficient": [0.3, 0.3],
    "slip_damping": 100)");
  Belt* belt = FindBelt(context);
  ASSERT_NE(belt, nullptr);
  ASSERT_EQ(context.get_auxiliary_state_count(), 4u);
  const size_t states = context.get_state_count();
  for(double theta : {2e-4, 4e-3}) {
    std::vector<double> y = InitialState(context);
    y[FindJoint(context, "J0")->get_q_index()] = theta;
    y[states] = theta;
    const std::vector<double> dy = Evaluate(context, y);
    const std::vector<double>& t = belt->get_tensions();
    const std::vector<double>& phi = belt->get_wrap_angles();
    /* pulley 0: span 1 arrives, span 0 leaves; pulley 1 the other way */
    const double limit_0 = t[0] * (std::exp(0.3 * phi[0]) - 1.0);
    const double limit_1 = t[0] * (std::exp(0.3 * phi[1]) - 1.0);
    const double excess_0 = std::max(0.0, (t[1] - t[0]) - limit_0);
    const double excess_1 = std::max(0.0, (t[1] - t[0]) - limit_1);
    EXPECT_NEAR(belt->get_slip_speeds()[0], -excess_0 / 100.0, 1e-12);
    EXPECT_NEAR(belt->get_slip_speeds()[1], excess_1 / 100.0, 1e-12);
    EXPECT_NEAR(dy[states + 2], belt->get_slip_speeds()[0], 1e-15);
    EXPECT_NEAR(dy[states + 3], belt->get_slip_speeds()[1], 1e-15);
    if(theta < 1e-3) {
      EXPECT_EQ(belt->get_slip_speeds()[0], 0.0);
      EXPECT_EQ(belt->get_slip_speeds()[1], 0.0);
    } else {
      EXPECT_LT(belt->get_slip_speeds()[0], -0.1);
      EXPECT_GT(belt->get_slip_speeds()[1], 0.1);
    }
  }
}

/* the first pulley, spinning, drives the second through the belt: the
torques r (T_leaving - T_arriving) keep J0 w0 / r0 + J1 w1 / r1, and once
the belt's vibration has died out the rims move together, w1 r1 = w0 r0 */
TEST(Belt, DriveBringsTheRimsToTheSameSpeed) {
  shift::core::Context context;
  LoadBelt(context, "belt_drive", R"("frames": [["p0", "origin"],
    ["p1", "origin"]], "radii": [0.1, 0.05], "axial_stiffness": 1e5,
    "damping": 1e-3, "pretension": 200)",
           false, R"(, "initial_velocity": 20)", 1.0);
  std::vector<double> y;
  context.set_state_observer(
    [&y](double, const std::vector<double>& state) { y = state; });
  context.Simulate();
  const size_t q_count = context.get_q_count();
  const double w0 = y[q_count + FindJoint(context, "J0")->get_dq_index()];
  const double w1 = y[q_count + FindJoint(context, "J1")->get_dq_index()];
  const double j0 = 0.01, j1 = 0.002;
  const double held = j0 * 20.0 / kR0;
  EXPECT_NEAR(j0 * w0 / kR0 + j1 * w1 / kR1, held, 1e-6 * held);
  const double w0_end = held / (j0 / kR0 + j1 * kR0 / (kR1 * kR1));
  EXPECT_NEAR(w0, w0_end, 1e-5 * w0_end);
  EXPECT_NEAR(w1 * kR1, w0 * kR0, 1e-5 * w0 * kR0);
  /* the pulleys turned many times: the turn states are not limited to a
  revolution */
  EXPECT_GT(y[context.get_state_count()], 3.0 * M_PI);
}

TEST(Belt, PulleysMustBeListedCounterclockwise) {
  shift::core::Context context;
  EXPECT_THROW(LoadBelt(context, "belt_clockwise", R"("frames": [["p0",
    "origin"], ["p1", "origin"], ["p2", "origin"]], "radii": [0.1, 0.05,
    0.04], "axial_stiffness": 1e5)",
                        true),
               shift::core::Exception);
}

TEST(Belt, OverlappingPulleysHaveNoSpan) {
  shift::core::Context context;
  EXPECT_THROW(LoadBelt(context, "belt_overlap", R"("frames": [["p0",
    "origin"], ["p1", "origin"]], "radii": [0.6, 0.05],
    "axial_stiffness": 1e5)"),
               shift::core::Exception);
}
