/* The run of least time of the planar car (controller::MinimumTime): on a
straight it is no faster than the tires' friction allows and faster than its
first run, and through a bend it keeps the car's outline on the road */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "shift/core/controller/minimum_time.h"

/* the Polo R WRC of examples/polo_r_wrc on gravel (test_slide_equilibrium) */
static shift::core::controller::MinimumTime Polo() {
  shift::core::controller::MinimumTime plan;
  auto& car = plan.car;
  car.mass = 1413.0;
  car.wheelbase = 2.41;
  car.center_of_mass = 1.29;
  car.height = 0.45;
  car.track = 1.5;
  car.front_load_transfer = 0.55;
  car.front_roll_steer = 0.0023;
  car.rear_roll_steer = 0.003;
  car.front.bias_torque = 80.0;
  car.front.bias_ratio = 0.25;
  car.rear.bias_torque = 80.0;
  car.rear.bias_ratio = 0.35;
  car.tire.unloaded_radius = 0.314;
  car.tire.vertical_stiffness = 180000.0;
  car.tire.friction_coefficient = 0.6;
  car.tire.rolling_resistance = 0.025;
  car.tire.cornering_stiffness = 35000.0;
  car.tire.aligning_coefficient = 0.03;
  car.tire.combined_slip = true;
  car.yaw_inertia = 2100.0;
  car.drive_inertia = 60.0;
  plan.torque_limit = [](double) { return 3000.0; };
  plan.brake_torque = 4000.0;
  plan.steering_limit = 0.5;
  plan.half_width = 3.0;
  plan.outline[0] = -0.65;
  plan.outline[1] = 3.2;
  plan.outline[2] = 0.92;
  plan.step = 1.0;
  plan.start_speed = 5.0;
  plan.guess_speed = [](double) { return 5.0; };
  plan.max_iterations = 400;
  return plan;
}

TEST(MinimumTime, OnAStraightTheTiresFrictionBoundsTheTime) {
  auto plan = Polo();
  plan.end = 40.0;
  ASSERT_TRUE(plan.Solve());
  const double t = plan.get_times().back();
  /* no faster than all of the friction forward, 5 m/s at the start */
  const double a = plan.car.tire.friction_coefficient * plan.car.gravity;
  const double fastest = (-5.0 + std::sqrt(25.0 + 2.0 * a * plan.end)) / a;
  EXPECT_GT(t, fastest);
  EXPECT_LT(t, plan.end / 5.0); // the first run, at 5 m/s
  /* it speeds up all the way, ending centred on the road */
  const auto& x = plan.get_states();
  const int n = plan.get_stations();
  constexpr int ns = shift::core::controller::MinimumTime::kStates;
  EXPECT_GT(x[(n - 1) * ns], x[(n / 2) * ns]);
  EXPECT_NEAR(x[(n - 1) * ns + 3], 0.0, plan.end_lateral + plan.tolerance);
}

TEST(MinimumTime, ThroughABendTheCarStaysOnTheRoad) {
  auto plan = Polo();
  /* 20 m straight, a right bend of 15 m radius through a quarter turn, then
  20 m straight */
  const double arc = 0.5 * M_PI * 15.0;
  plan.curvature = [arc](double s) {
    return s > 20.0 && s < 20.0 + arc ? 1.0 / 15.0 : 0.0;
  };
  plan.end = 40.0 + arc;
  ASSERT_TRUE(plan.Solve()) << "violation " << plan.get_violation() << " after "
                            << plan.get_iterations() << " iterations";
  EXPECT_LE(plan.get_violation(), plan.tolerance);
  /* the corners of the car on the road at every station */
  const auto& x = plan.get_states();
  constexpr int ns = shift::core::controller::MinimumTime::kStates;
  double fastest = 0.0;
  for(int k = 0; k < plan.get_stations(); ++k) {
    const double* s = &x[k * ns];
    const double psi = s[4] - std::atan2(s[1], s[0]);
    for(const double along : {plan.outline[0], plan.outline[1]}) {
      for(const double side : {-plan.outline[2], plan.outline[2]}) {
        const double p = s[3] + along * std::sin(psi) + side * std::cos(psi);
        EXPECT_LE(std::fabs(p), plan.half_width + plan.tolerance);
      }
    }
    const double station = k * plan.step;
    if(station > 20.0 + 0.4 * arc && station < 20.0 + 0.6 * arc) {
      fastest = std::max(fastest, std::hypot(s[0], s[1]));
    }
  }
  /* mid-bend no faster than the friction holds on the widest circle through
  the quarter turn that the car's width leaves of the road, from its inside
  edge, rho = r + w / (1 - cos(pi / 4)) */
  const double a = plan.car.tire.friction_coefficient * plan.car.gravity;
  const double inside = 15.0 - plan.half_width + plan.outline[2];
  const double width = 2.0 * (plan.half_width - plan.outline[2]);
  const double widest = inside + width / (1.0 - std::cos(0.25 * M_PI));
  EXPECT_LT(fastest, std::sqrt(a * widest));
  EXPECT_GT(fastest, 5.0);
}

TEST(MinimumTime, DownhillTheRunIsFaster) {
  auto level = Polo();
  level.end = 40.0;
  ASSERT_TRUE(level.Solve());
  /* a grade of a tenth, falling ahead */
  auto downhill = Polo();
  downhill.end = 40.0;
  downhill.slope = [&downhill](double, double* g) {
    const double grade = 0.1, norm = std::sqrt(1.0 + grade * grade);
    g[0] = downhill.car.gravity * grade / norm;
    g[1] = 0.0;
    g[2] = downhill.car.gravity / norm;
  };
  ASSERT_TRUE(downhill.Solve());
  EXPECT_LT(downhill.get_times().back(), level.get_times().back());
}

TEST(MinimumTime, TheTrackingGainsBringTheCarBackToItsRun) {
  auto plan = Polo();
  const double arc = 0.5 * M_PI * 15.0;
  plan.curvature = [arc](double s) {
    return s > 20.0 && s < 20.0 + arc ? 1.0 / 15.0 : 0.0;
  };
  plan.end = 40.0 + arc;
  ASSERT_TRUE(plan.Solve());
  constexpr int ns = shift::core::controller::MinimumTime::kStates;
  constexpr int nt = shift::core::controller::MinimumTime::kTrackStates;
  const double scales[nt + 2] = {1.0, 0.5, 0.3, 0.3, 0.1, 0.2, 0.05, 1.0};
  std::vector<double> gains;
  ASSERT_TRUE(plan.TrackingGains(scales, &gains));
  const auto& ref = plan.get_states();
  /* from 0.5 m right of the run, the steering's change and the carriers'
  speed of the regulator, by the plan's steps */
  auto run = [&](bool regulated) {
    double x[ns];
    std::copy_n(&ref[0], ns, x);
    x[3] += 0.5;
    const int n = plan.get_stations() - 1;
    for(int k = 0; k < n; ++k) {
      const double* r = &ref[k * ns];
      double u[2] = {(ref[(k + 1) * ns + 5] - r[5]) / plan.step, 0.0};
      x[6] = r[6];
      if(regulated) {
        for(int j = 0; j < nt; ++j) {
          u[0] -= gains[k * 2 * nt + j] * (x[j] - r[j]);
          x[6] -= gains[k * 2 * nt + nt + j] * (x[j] - r[j]);
        }
      }
      double next[ns], time, c[shift::core::controller::MinimumTime::kConstraints];
      plan.Step(k, x, u, next, &time, c);
      std::copy_n(next, ns, x);
    }
    return std::fabs(x[3] - ref[n * ns + 3]);
  };
  const double regulated = run(true);
  EXPECT_LT(regulated, 0.2);
  EXPECT_LT(regulated, 0.5 * run(false));
}
