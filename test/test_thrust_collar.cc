/* Thrust collar force module (force_module::ThrustCollar): a rotor on a slide,
above a fixed seat, against the closed forms of the squeeze of the film, the
shear of the oil, the friction of the surfaces and the lift of the pads */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/thrust_collar.h"

using shift::core::force_module::ThrustCollar;

static const double kGravity = 9.81;
static const double kInner = 0.02, kOuter = 0.05;
static const double kViscosity = 0.05;
static const double kMass = 10.0;
static const double kInertia = 0.02; //!< of the rotor about its axis
static const double kRing = 2.0 / 3.0 * (0.05 * 0.05 * 0.05 - 0.02 * 0.02 * 0.02) /
                            (0.05 * 0.05 - 0.02 * 0.02);

/* the rotor, gap above the seat (z is down), on a slide, and
turning at speed. module: the keys of the collar after its frames */
static void LoadModel(shift::core::Context& context, const std::string& name,
                      double gap, double speed, const std::string& module,
                      double time_span, double hmax = 1e-3) {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file.precision(17);
    file << R"({"model_parameters": {"version": "0.2",
      "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
                "time": "seconds"},
      "gravity": [0, 0, )" << kGravity << R"(],
      "solvers": [{"type": "rnca"}],
      "integrators": [{"type": "ode113", "relative_error": 1e-8,
                       "absolute_error": 1e-11, "hmax": )" << hmax << R"(,
                       "time_span": )" << time_span << R"(,
                       "time_step": 0.01}]},
      "frames": [{"name": "seat", "T": [0, 0, 0]},
                 {"name": "start", "T": [0, 0, )" << -gap << R"(]}],
      "assemblies": [{"name": "m", "bodies": [
        {"name": "slider", "center_of_mass": [0, 0, 0], "geometry": [],
         "inertia": {"mass": )" << 0.5 * kMass << R"(, "Ixx": 1, "Iyy": 1,
                     "Izz": 1, "Ixy": 0, "Iyz": 0, "Izx": 0}},
        {"name": "rotor", "center_of_mass": [0, 0, 0], "geometry": [],
         "inertia": {"mass": )" << 0.5 * kMass << R"(, "Ixx": 0.02, "Iyy": 0.02,
                     "Izz": )" << kInertia << R"(, "Ixy": 0, "Iyz": 0,
                     "Izx": 0}}],
      "joints": [
        {"name": "slide", "type": "prismatic", "axis": [0, 0, 1],
         "parent": [["fixed"], "start"], "child": [["slider"], "origin"]},
        {"name": "spin", "type": "revolute", "axis": [0, 0, 1],
         "parent": [["slider"], "origin"], "child": [["rotor"], "origin"],
         "initial_velocity": )" << speed << R"(}],
      "force_modules": [{"type": "thrust_collar", "name": "collar",
        "frames": [["rotor", "origin"], ["fixed", "seat"]],
        "inner_radius": )" << kInner << R"(, "outer_radius": )" << kOuter
         << R"(, )" << module << R"(}]}]})";
  }
  try {
    context.LoadJSON(path);
    std::remove(path.c_str());
  } catch(...) {
    std::remove(path.c_str());
    throw;
  }
}

/* the ring's factor of the squeeze, c h^3 */
static double SqueezeFactor() {
  const double a2 = kInner * kInner, b2 = kOuter * kOuter;
  return 1.5 * M_PI * kViscosity *
         (b2 * b2 - a2 * a2 - (b2 - a2) * (b2 - a2) / std::log(kOuter / kInner));
}

TEST(ThrustCollar, FilmNeverCloses) {
  ThrustCollar collar;
  EXPECT_NEAR(collar.FilmThickness(1.0), 1.0 + 1e-6, 1e-9);
  EXPECT_NEAR(collar.FilmThickness(-1.0), 1e-6, 1e-9);
  EXPECT_NEAR(collar.FilmThickness(0.0), 1.5e-6, 1e-12);
}

TEST(ThrustCollar, FilmSqueezesOutAsReynolds) {
  // the weight on the squeeze alone, slowly: 1 / h^2 grows evenly in time
  const double h0 = 2e-4, roughness = 1e-8, span = 0.4;
  shift::core::Context context;
  std::ostringstream module;
  module << R"("viscosity": 0.05, "film_damping_limit": 1e12,
               "slip_speed": 1e-8, "roughness": )"
         << roughness;
  LoadModel(context, "tc_squeeze", h0, 0.0, module.str(), span, 1e-4);
  double t0 = 0.0, q0 = 0.0, t1 = 0.0, q1 = 0.0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    if(t0 == 0.0 && t > 0.1) { // past the start from rest
      t0 = t;
      q0 = y[0];
    }
    t1 = t;
    q1 = y[0];
  });
  context.Simulate();
  ASSERT_GT(t1, t0);
  const double film0 = h0 - q0 + roughness, film1 = h0 - q1 + roughness;
  const double expected = 1.0 / (film0 * film0) +
                          2.0 * kMass * kGravity * (t1 - t0) / SqueezeFactor();
  EXPECT_NEAR(1.0 / (film1 * film1) / expected, 1.0, 1e-3);
  EXPECT_LT(film1, 0.9 * film0);
}

TEST(ThrustCollar, OilSlowsTheRotorAsPetroff) {
  // the shear of the oil alone, over a film that closes little in the time
  const double gap = 1e-4, speed = 50.0, span = 0.2;
  shift::core::Context context;
  LoadModel(context, "tc_shear", gap, speed,
            R"("viscosity": 0.05, "film_damping_limit": 1e12,
               "roughness": 1e-8)",
            span, 1e-4);
  double spin = 0.0, q = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    q = y[0];
    spin = y[3];
  });
  context.Simulate();
  // over the mean film, which closes by little
  const double film = gap - 0.5 * q;
  const double a2 = kInner * kInner, b2 = kOuter * kOuter;
  const double c = 0.5 * M_PI * kViscosity * (b2 * b2 - a2 * a2) / film;
  EXPECT_NEAR(spin / (speed * std::exp(-c * span / kInertia)), 1.0, 1e-2);
}

TEST(ThrustCollar, SurfacesStopTheRotor) {
  // dry, resting on the surfaces: the torque of the weight at the radius of
  // the ring, so the speed falls evenly
  const double speed = 20.0, span = 0.3, friction = 0.2;
  shift::core::Context context;
  std::ostringstream module;
  module << R"("friction": )" << friction
         << R"(, "contact_stiffness": 1e7, "contact_damping": 5000)";
  LoadModel(context, "tc_dry", 0.0, speed, module.str(), span);
  double spin = 0.0, q = 0.0, dq = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    q = y[0];
    dq = y[2];
    spin = y[3];
  });
  context.Simulate();
  const double weight = kMass * kGravity;
  EXPECT_NEAR(q, weight / 1e7, 2e-6); // on the stiffness
  EXPECT_NEAR(dq, 0.0, 1e-5);
  const double torque = friction * weight * kRing;
  EXPECT_NEAR(spin, speed - torque * span / kInertia, 0.02);
}

TEST(ThrustCollar, PadsLiftTheRotor) {
  // the rotor settles on the film at which the pads carry its weight, off the
  // surfaces, and follows that film as the oil slows it
  const double speed = 300.0, span = 0.5;
  shift::core::Context context;
  const std::string module =
    R"("viscosity": 0.05, "pads": 6, "taper": 3e-5, "roughness": 1e-7,
       "film_damping_limit": 1e5)";
  LoadModel(context, "tc_pads", 5e-5, speed, module, span);
  auto* collar = dynamic_cast<ThrustCollar*>(
    (*context.get_assemblies()[0])["force_modules"]["collar"]
      .get_data<shift::core::force_module::Base>());
  ASSERT_NE(collar, nullptr);
  double gap = 0.0, spin = 0.0;
  context.set_state_observer([&](double, const std::vector<double>& y) {
    gap = 5e-5 - y[0];
    spin = y[3];
  });
  context.Simulate();
  // the film at which the pads carry the weight at the final speed
  double lo = 1e-7, hi = 1e-3;
  for(int i = 0; i < 80; i++) {
    const double mid = std::sqrt(lo * hi);
    (collar->PadLoad(mid, spin) > kMass * kGravity ? lo : hi) = mid;
  }
  EXPECT_GT(gap, 2e-6);
  EXPECT_NEAR((gap + 1e-7) / lo, 1.0, 0.03);
}

TEST(ThrustCollar, EndPlayStopsTheSecondShoulder) {
  // the rotor starts beyond the end play, so the second shoulder pushes it
  // down, and it comes to rest on the seat under its weight
  const double play = 1e-3;
  shift::core::Context context;
  LoadModel(context, "tc_play", 1.5e-3, 0.0,
            R"("end_play": 1e-3, "contact_stiffness": 1e7,
               "contact_damping": 5000)",
            0.3);
  double q = 0.0;
  context.set_state_observer(
    [&](double, const std::vector<double>& y) { q = y[0]; });
  context.Simulate();
  // settled on the seat under its weight
  EXPECT_NEAR(1.5e-3 - q, -kMass * kGravity / 1e7, 2e-6);
  EXPECT_GT(q, play);
}

TEST(ThrustCollar, BadParametersAreRefused) {
  for(const char* module : {R"("pads": 6)", R"("taper": 1e-5)",
                            R"("roughness": 0)", R"("viscosity": -1)",
                            R"("film_damping_limit": 0)"}) {
    SCOPED_TRACE(module);
    shift::core::Context context;
    EXPECT_THROW(LoadModel(context, "tc_bad", 1e-3, 0.0, module, 0.0),
                 std::exception);
  }
}

TEST(ThrustCollar, RotorComesDownAsItSlows) {
  // examples/force_modules/thrust_collar.json: on the film at speed, on the
  // surfaces at rest
  shift::core::Context context;
  context.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                   "/force_modules/thrust_collar.json");
  const double start = 1e-4;
  double early = 0.0, late = 0.0, spin = 0.0;
  context.set_state_observer([&](double t, const std::vector<double>& y) {
    if(t < 0.2) {
      early = start - y[0];
    }
    late = start - y[0];
    spin = y[3];
  });
  context.Simulate();
  EXPECT_GT(early, 3e-6);
  EXPECT_LT(late, 3e-6);
  EXPECT_LT(std::fabs(spin), 1.0);
}
