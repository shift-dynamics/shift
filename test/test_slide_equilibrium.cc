/* The steady slides of a four-wheel drive car with no centre differential
(controller::SlideEquilibrium): the equations hold at the solution, and the
slides behave as the physics has them */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <lapacke.h>
#include <cstdio>

#include "shift/core/controller/slide_equilibrium.h"

/* the Polo R WRC of examples/polo_r_wrc on gravel, with combined slip */
static shift::core::controller::SlideEquilibrium Polo() {
  shift::core::controller::SlideEquilibrium car;
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
  car.yaw_inertia = 2100.0;
  car.drive_inertia = 60.0;
  car.tire.combined_slip = true;
  return car;
}

TEST(SlideEquilibrium, TheEquationsHoldAtTheSolution) {
  const auto car = Polo();
  shift::core::controller::SlideEquilibrium::State s;
  ASSERT_TRUE(car.Find(30.0, -0.05, &s));
  double e[5];
  const double x[5] = {s.speed, s.steering, s.spin, s.front_difference,
                       s.rear_difference};
  car.Residuals(30.0, -0.05, x, e, nullptr);
  for(double x : e) {
    EXPECT_NEAR(x, 0.0, 1e-8);
  }
  EXPECT_NEAR(s.yaw_rate, s.speed / 30.0, 1e-12);
}

TEST(SlideEquilibrium, DISABLED_Compare) {
  // the steady states of the full model (examples/polo_r_wrc, combined
  // slip) held by 0.1 rad of steering and the throttle 0.5, 0.7, 0.9
  const auto car = Polo();
  const double states[4][3] = {{12.74, -4.4, 12.74 / 31.2},
                               {16.69, -5.9, 0.34}, {19.9, -8.2, 0.30},
                               {20.06, -14.8, 0.29}};
  for(const auto& q : states) {
    const double rho = q[0] / q[2];
    shift::core::controller::SlideEquilibrium::State s;
    const bool ok = car.Find(rho, q[1] * M_PI / 180.0, &s);
    std::printf("rho %5.1f beta %5.1f %s v %5.2f (%5.2f) delta %6.3f spin "
                "%6.3f d %6.3f %6.3f torque %5.0f res %.1e\n",
                rho, q[1], ok ? "ok" : "FAIL", s.speed, q[0], s.steering,
                s.spin, s.front_difference, s.rear_difference, s.drive_torque,
                s.residual);
    for(int i = 0; i < 4; ++i) {
      std::printf("   wheel %d kappa %6.3f alpha %6.3f load %5.0f\n", i,
                  s.slip[i], s.slip_angle[i], s.load[i]);
    }
  }
}

TEST(SlideEquilibrium, DISABLED_AllRoots) {
  // every root from a wide grid of guesses, on 30 m
  const auto car = Polo();
  for(double deg : {8.0, 12.0, 16.0, 20.0, 25.0, 30.0}) {
    std::vector<std::array<double, 5>> roots;
    for(double v = 6.0; v <= 26.0; v += 2.0) {
      for(double d = -0.5; d <= 0.6; d += 0.1) {
        for(double sigma : {0.02, 0.1, 0.3, 0.6, 1.0, 2.0}) {
          shift::core::controller::SlideEquilibrium::State s;
          s.speed = v;
          s.steering = d;
          s.spin = sigma;
          if(!car.Solve(30.0, -deg * M_PI / 180.0, &s)) {
            continue;
          }
          bool seen = false;
          for(const auto& r : roots) {
            seen = seen || (std::fabs(r[0] - s.speed) < 0.05 &&
                            std::fabs(r[1] - s.steering) < 0.005 &&
                            std::fabs(r[2] - s.spin) < 0.005);
          }
          if(!seen) {
            roots.push_back({s.speed, s.steering, s.spin, s.drive_torque,
                             s.slip_angle[0]});
          }
        }
      }
    }
    for(const auto& r : roots) {
      std::printf("beta %4.0f: v %5.2f delta %6.3f spin %6.3f torque %6.0f "
                  "alpha_lf %6.3f\n",
                  deg, r[0], r[1], r[2], r[3], r[4]);
    }
    if(roots.empty()) {
      std::printf("beta %4.0f: none\n", deg);
    }
  }
}

TEST(SlideEquilibrium, TheMotionIsSteadyAtTheSlide) {
  const auto car = Polo();
  shift::core::controller::SlideEquilibrium::State s;
  const double beta = -0.35;
  ASSERT_TRUE(car.Find(30.0, beta, &s));
  const double vx = s.speed * std::cos(beta);
  const double state[4] = {vx, s.speed * std::sin(beta), s.yaw_rate,
                           (1.0 + s.spin) * vx / car.tire.unloaded_radius};
  double rates[4];
  car.Derivatives(state, s.steering, s.drive_torque, rates);
  EXPECT_NEAR(rates[0], 0.0, 1e-4);
  EXPECT_NEAR(rates[1], 0.0, 1e-4);
  EXPECT_NEAR(rates[2], 0.0, 1e-4);
  EXPECT_NEAR(rates[3], 0.0, 1e-3);
}

TEST(SlideEquilibrium, TheRegulatorStabilizesTheSlide) {
  // the closed loop of the gain, with the path's states, is stable
  const auto car = Polo();
  shift::core::controller::SlideEquilibrium::State s;
  const double beta = -0.35;
  ASSERT_TRUE(car.Find(30.0, beta, &s));
  double a[36], b[12];
  car.Linearize(beta, s, a, b);
  double q[36] = {};
  const double scales[6] = {2.0, 0.7, 0.1, 5.0, 1.0, 0.1};
  for(int i = 0; i < 6; ++i) {
    q[i * 6 + i] = 1.0 / (scales[i] * scales[i]);
  }
  const double r[4] = {1.0 / 0.01, 0, 0, 1.0 / 250000.0};
  double k[12];
  ASSERT_TRUE(shift::core::controller::SlideEquilibrium::Regulator(
    a, b, q, r, 0.05, k));
  // the continuous closed loop A - B K, stepped by Euler over 60 s
  double x[6] = {0.5, 0.3, 0.05, 2.0, 1.0, 0.05};
  for(int step = 0; step < 60000; ++step) {
    double u[2] = {0, 0};
    for(int i = 0; i < 2; ++i) {
      for(int j = 0; j < 6; ++j) {
        u[i] -= k[i * 6 + j] * x[j];
      }
    }
    double dx[6];
    for(int i = 0; i < 6; ++i) {
      dx[i] = b[i * 2] * u[0] + b[i * 2 + 1] * u[1];
      for(int j = 0; j < 6; ++j) {
        dx[i] += a[i * 6 + j] * x[j];
      }
    }
    for(int i = 0; i < 6; ++i) {
      x[i] += 1e-3 * dx[i];
    }
  }
  for(double v : x) {
    EXPECT_LT(std::fabs(v), 1e-3);
  }
}

TEST(SlideEquilibrium, DISABLED_Stability) {
  // the eigenvalues of the motion about the slides on 30 m, open and with
  // the regulator
  const auto car = Polo();
  double q[36] = {};
  const double scales[6] = {2.0, 0.7, 0.1, 5.0, 1.0, 0.1};
  for(int i = 0; i < 6; ++i) {
    q[i * 6 + i] = 1.0 / (scales[i] * scales[i]);
  }
  const double r[4] = {1.0 / 0.01, 0, 0, 1.0 / 250000.0};
  for(double deg : {5.0, 10.0, 15.0, 20.0, 25.0, 30.0}) {
    shift::core::controller::SlideEquilibrium::State s;
    const double beta = -deg * M_PI / 180.0;
    if(!car.Find(30.0, beta, &s)) {
      std::printf("beta %4.0f: no slide\n", deg);
      continue;
    }
    double a[36], b[12], k[12];
    car.Linearize(beta, s, a, b);
    const bool ok = shift::core::controller::SlideEquilibrium::Regulator(
      a, b, q, r, 0.05, k);
    double closed[36];
    for(int i = 0; i < 6; ++i) {
      for(int j = 0; j < 6; ++j) {
        closed[i * 6 + j] = a[i * 6 + j] - b[i * 2] * k[j] -
                            b[i * 2 + 1] * k[6 + j];
      }
    }
    for(double* m : {a, closed}) {
      double wr[6], wi[6], copy[36];
      std::copy(m, m + 36, copy);
      LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'N', 'N', 6, copy, 6, wr, wi, nullptr,
                    1, nullptr, 1);
      std::printf("beta %4.0f %s:", deg, m == a ? "open  " : "closed");
      for(int i = 0; i < 6; ++i) {
        std::printf(" %7.2f%+7.2fi", wr[i], wi[i]);
      }
      std::printf("\n");
    }
    std::printf("   K%s steering:", ok ? "" : "(FAILED)");
    for(int j = 0; j < 6; ++j) {
      std::printf(" %8.4f", k[j]);
    }
    std::printf("  torque:");
    for(int j = 0; j < 6; ++j) {
      std::printf(" %8.1f", k[6 + j]);
    }
    std::printf("\n");
  }
}

TEST(SlideEquilibrium, DISABLED_Gains) {
  // the regulator's gains on the radius 30 m, by sideslip
  const auto car = Polo();
  double q[36] = {};
  const double scales[6] = {2.0, 0.7, 0.1, 5.0, 1.0, 0.1};
  for(int i = 0; i < 6; ++i) {
    q[i * 6 + i] = 1.0 / (scales[i] * scales[i]);
  }
  const double r[4] = {1.0 / 0.01, 0, 0, 1.0 / 250000.0};
  for(double beta : {-0.1, -0.2, -0.35}) {
    shift::core::controller::SlideEquilibrium::State s;
    ASSERT_TRUE(car.Find(30.0, beta, &s));
    double a[36], b[12], k[12];
    car.Linearize(beta, s, a, b);
    ASSERT_TRUE(shift::core::controller::SlideEquilibrium::Regulator(
      a, b, q, r, 0.05, k));
    std::printf("beta %5.2f steering:", beta);
    for(int j = 0; j < 6; ++j) {
      std::printf(" %9.4f", k[j]);
    }
    std::printf("\n           torque:  ");
    for(int j = 0; j < 6; ++j) {
      std::printf(" %9.1f", k[6 + j]);
    }
    std::printf("\n");
  }
}

/* With the steering held off its optimum (at a limit), the torque corrected
by the coupling is the one of least cost: the stage cost and the cost to go
of the regulator, found by running the discrete closed loop out, for A = -I,
held exactly over dt */
TEST(SlideEquilibrium, TheCouplingGivesTheTorqueOfLeastCostWithTheSteeringHeld) {
  constexpr int n = 6, m = 2;
  double a[n * n] = {}, q[n * n] = {};
  for(int i = 0; i < n; ++i) {
    a[i * n + i] = -1.0;
    q[i * n + i] = 1.0 + 0.5 * i;
  }
  const double b[n * m] = {1.0, 0.4, 0.3, 1.0, -0.5, 0.8,
                           0.2, -0.6, 0.7, 0.1, 0.0, 0.9};
  const double r[m * m] = {0.01, 0.0, 0.0, 0.005}, dt = 0.05;
  double k[m * n], coupling = 0.0;
  ASSERT_TRUE(shift::core::controller::SlideEquilibrium::Regulator(
    a, b, q, r, dt, k, &coupling));
  EXPECT_GT(std::fabs(coupling), 1e-3);
  const double ad = std::exp(-dt), bd = 1.0 - std::exp(-dt);
  auto control = [&](const double* x, double* u) {
    for(int i = 0; i < m; ++i) {
      u[i] = 0.0;
      for(int j = 0; j < n; ++j) {
        u[i] -= k[i * n + j] * x[j];
      }
    }
  };
  auto stage = [&](const double* x, const double* u) {
    double c = 0.0;
    for(int i = 0; i < n; ++i) {
      c += q[i * n + i] * x[i] * x[i];
    }
    return (c + r[0] * u[0] * u[0] + r[3] * u[1] * u[1]) * dt;
  };
  auto step = [&](double* x, const double* u) {
    for(int i = 0; i < n; ++i) {
      x[i] = ad * x[i] + bd * (b[i * m] * u[0] + b[i * m + 1] * u[1]);
    }
  };
  // the cost of u at s, the regulator after
  const double s[n] = {0.5, -0.3, 0.8, 0.2, -0.6, 0.4};
  auto cost = [&](const double* u) {
    double x[n], v[m];
    std::copy(s, s + n, x);
    double c = stage(x, u);
    step(x, u);
    for(int t = 0; t < 20000; ++t) {
      control(x, v);
      c += stage(x, v);
      step(x, v);
    }
    return c;
  };
  double best[m];
  control(s, best);
  const double off = 0.3;
  const double held[m] = {best[0] + off, best[1] - coupling * off};
  const double c = cost(held);
  for(double e : {-0.01, 0.01}) {
    const double other[m] = {held[0], held[1] + e};
    EXPECT_LT(c, cost(other));
  }
  // and the uncorrected torque costs more
  const double uncorrected[m] = {held[0], best[1]};
  EXPECT_LT(c, cost(uncorrected));
}

/* The regulator of the inputs' changes: with a vanishing cost on the rates,
the input it sets does not depend on the last one (the gain of the last
inputs is I) and is the regulator's of the state, delayed by the one sample
in which the last input carries R; with a cost on the rates, it changes the
input less in a sample than the plain regulator does */
TEST(SlideEquilibrium, TheRateRegulatorWeighsTheChangesOfTheInputs) {
  constexpr int n = 6, m = 2, na = n + m;
  double a[n * n] = {}, q[n * n] = {};
  for(int i = 0; i < n; ++i) {
    a[i * n + i] = -1.0;
    q[i * n + i] = 1.0 + 0.5 * i;
  }
  const double b[n * m] = {1.0, 0.4, 0.3, 1.0, -0.5, 0.8,
                           0.2, -0.6, 0.7, 0.1, 0.0, 0.9};
  const double r[m * m] = {0.01, 0.0, 0.0, 0.005}, dt = 0.05;
  double k[m * n];
  ASSERT_TRUE(shift::core::controller::SlideEquilibrium::Regulator(
    a, b, q, r, dt, k));
  const double free[m * m] = {1e-12, 0.0, 0.0, 1e-12};
  double kf[m * na];
  ASSERT_TRUE(shift::core::controller::SlideEquilibrium::RateRegulator(
    a, b, q, r, free, dt, kf));
  for(int i = 0; i < m; ++i) {
    for(int j = 0; j < m; ++j) {
      EXPECT_NEAR(kf[i * na + n + j], i == j ? 1.0 : 0.0, 1e-6);
    }
  }
  // a costly rate: the input moves less in a sample from the last one, 0
  const double costly[m * m] = {1.0, 0.0, 0.0, 1.0};
  double kc[m * na];
  ASSERT_TRUE(shift::core::controller::SlideEquilibrium::RateRegulator(
    a, b, q, r, costly, dt, kc));
  const double s[n] = {0.5, -0.3, 0.8, 0.2, -0.6, 0.4};
  for(int i = 0; i < m; ++i) {
    double plain = 0.0, change = 0.0;
    for(int j = 0; j < n; ++j) {
      plain -= k[i * n + j] * s[j];
      change -= kc[i * na + j] * s[j];
    }
    EXPECT_LT(std::fabs(change), 0.5 * std::fabs(plain));
  }
}

TEST(SlideEquilibrium, TheTabulatedDeflectionKeepsTheMotion) {
  /* the tires' deflection by load from samples moves the car as solving it
  at each load does */
  auto car = Polo();
  const double s[4] = {12.0, -2.0, 0.6, 12.0 * 1.1 / 0.314};
  double exact[4], sampled[4];
  car.Derivatives(s, 0.1, 500.0, exact);
  car.TabulateDeflection(1.5 * car.mass * car.gravity, 2.0);
  car.Derivatives(s, 0.1, 500.0, sampled);
  for(int i = 0; i < 4; ++i) {
    EXPECT_NEAR(sampled[i], exact[i], 1e-6 * (1.0 + std::fabs(exact[i])));
  }
}
