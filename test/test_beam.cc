/* Beam force module (force_module::Beam): the stiffness of each direction of
motion of the tip, the section properties, and a cantilever settling under a
tip load */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/beam.h"

using shift::core::force_module::Beam;

/* a tip body on a joint at the end of a beam of length 1 along the fixed x
axis; frame "root" at the fixed origin is end A */
static void LoadBeam(shift::core::Context& context, const std::string& name,
                     const std::string& joint, const std::string& beam,
                     const std::string& root_rotation =
                       "[[1, 0, 0], [0, 1, 0], [0, 0, 1]]",
                     const std::string& tip = "[1, 0, 0]",
                     const std::string& integrator =
                       R"({"type": "rk4", "time_span": 0.01,
                           "time_step": 0.01})") {
  const std::string path = testing::TempDir() + "/" + name + ".json";
  {
    std::ofstream file(path);
    file << R"({
  "model_parameters": {
    "version": "0.2",
    "units": {"angle": "radians", "length": "meters", "mass": "kilograms",
              "time": "seconds"},
    "gravity": [0.0, 0.0, 9.81],
    "solvers": [{"type": "rnca"}],
    "integrators": [)"
         << integrator << R"(]
  },
  "frames": [{"name": "root", "T": [0, 0, 0], "R": )"
         << root_rotation << R"(},
             {"name": "tip_home", "T": )"
         << tip << R"(}],
  "assemblies": [{"name": "test",
    "bodies": [{"name": "tip", "center_of_mass": [0, 0, 0],
      "inertia": {"mass": 2, "Ixx": 0.01, "Iyy": 0.01, "Izz": 0.01,
                  "Ixy": 0, "Iyz": 0, "Izx": 0}}],
    "joints": [)"
         << joint << R"(],
    "force_modules": [{"type": "beam", "name": "beam",
      "frames": [["fixed", "root"], ["tip", "origin"]], )"
         << beam << R"(}]
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

static Beam* FindBeam(shift::core::Context& context) {
  for(auto module : context.get_active_force_modules()) {
    if(auto beam = dynamic_cast<Beam*>(module)) {
      return beam;
    }
  }
  return nullptr;
}

/* a joint of the tip at the end of the beam */
static std::string TipJoint(const std::string& type, const std::string& axis) {
  return R"({"name": "J", "type": ")" + type +
         R"(", "parent": [["fixed"], "tip_home"],
            "child": [["tip"], "origin"], "axis": )" +
         axis + "}";
}

static const char* kSteelTube = R"("young_modulus": 200e9,
  "poisson_ratio": 0.3, "section": "rectangle", "width": 0.04,
  "height": 0.02)";

/* the force and moment f on the tip for a displacement q of the joint, in
the rest axes (the fixed axes here) */
static std::vector<double> TipForce(const std::string& name,
                                    const std::string& type,
                                    const std::string& axis, double q) {
  shift::core::Context context;
  LoadBeam(context, name, TipJoint(type, axis), kSteelTube);
  Beam* beam = FindBeam(context);
  EXPECT_NE(beam, nullptr);
  context.get_q()[0] = q;
  context.get_dq()[0] = 0.0;
  context.UpdateDHVA();
  std::vector<double> y(context.get_state_count(), 0.0);
  y[0] = q;
  std::vector<double> dy(y.size(), 0.0);
  context.SetForces(0.0, y, dy);
  return beam->get_forces();
}

TEST(Beam, SectionOfARectangle) {
  shift::core::Context context;
  LoadBeam(context, "beam_section", TipJoint("prismatic", "[1, 0, 0]"),
           kSteelTube);
  Beam* beam = FindBeam(context);
  ASSERT_NE(beam, nullptr);
  EXPECT_NEAR(beam->get_length(), 1.0, 1e-12);
  EXPECT_NEAR(beam->get_area(), 8e-4, 1e-15);
  EXPECT_NEAR(beam->get_Iy(), 0.04 * std::pow(0.02, 3) / 12.0, 1e-18);
  EXPECT_NEAR(beam->get_Iz(), 0.02 * std::pow(0.04, 3) / 12.0, 1e-18);
  /* a 2:1 rectangle: J = 0.229 a b^3 (Roark) */
  EXPECT_NEAR(beam->get_J() / (0.04 * std::pow(0.02, 3)), 0.229, 1e-3);
  EXPECT_NEAR(beam->get_shear_modulus(), 200e9 / 2.6, 1.0);
}

TEST(Beam, StiffnessOfEachMotionOfTheTip) {
  const double E = 200e9, L = 1.0, A = 8e-4;
  const double Iy = 0.04 * std::pow(0.02, 3) / 12.0;
  const double Iz = 0.02 * std::pow(0.04, 3) / 12.0;
  const double d = 1e-4, r = 1e-4;
  /* tension */
  auto f = TipForce("beam_axial", "prismatic", "[1, 0, 0]", d);
  EXPECT_NEAR(f[0], -E * A / L * d, 1e-6 * E * A / L * d);
  /* bending in the x-y plane: a sideways shift and a turn about z */
  f = TipForce("beam_v", "prismatic", "[0, 1, 0]", d);
  EXPECT_NEAR(f[1], -12 * E * Iz / (L * L * L) * d, 1e-6 * std::fabs(f[1]));
  EXPECT_NEAR(f[5], 6 * E * Iz / (L * L) * d, 1e-6 * std::fabs(f[5]));
  f = TipForce("beam_rz", "revolute", "[0, 0, 1]", r);
  EXPECT_NEAR(f[1], 6 * E * Iz / (L * L) * r, 1e-5 * std::fabs(f[1]));
  EXPECT_NEAR(f[5], -4 * E * Iz / L * r, 1e-5 * std::fabs(f[5]));
  /* bending in the x-z plane: the coupling has the other sign */
  f = TipForce("beam_w", "prismatic", "[0, 0, 1]", d);
  EXPECT_NEAR(f[2], -12 * E * Iy / (L * L * L) * d, 1e-6 * std::fabs(f[2]));
  EXPECT_NEAR(f[4], -6 * E * Iy / (L * L) * d, 1e-6 * std::fabs(f[4]));
  f = TipForce("beam_ry", "revolute", "[0, 1, 0]", r);
  EXPECT_NEAR(f[2], -6 * E * Iy / (L * L) * r, 1e-5 * std::fabs(f[2]));
  EXPECT_NEAR(f[4], -4 * E * Iy / L * r, 1e-5 * std::fabs(f[4]));
  /* torsion */
  shift::core::Context context;
  LoadBeam(context, "beam_section_j", TipJoint("prismatic", "[1, 0, 0]"),
           kSteelTube);
  const double GJ = FindBeam(context)->get_shear_modulus() *
                    FindBeam(context)->get_J();
  f = TipForce("beam_rx", "revolute", "[1, 0, 0]", r);
  EXPECT_NEAR(f[3], -GJ / L * r, 1e-5 * GJ / L * r);
}

/* a cantilever along the fixed y axis (end A's x axis), its tip mass on a
free joint, settles under its weight to the tip deflection P L^3 / (3 E I)
and slope P L^2 / (2 E I) */
TEST(Beam, CantileverSettlesUnderATipLoad) {
  const double E = 1e9, d = 0.05, L = 1.0, m = 2.0, g = 9.81;
  const double I = M_PI / 64.0 * std::pow(d, 4);
  const double P = m * g;
  shift::core::Context context;
  /* end A's axes: x along fixed y, y along fixed -x, z along fixed z */
  LoadBeam(context, "beam_cantilever",
           R"({"name": "J", "type": "float", "parent": [["fixed"], "tip_home"],
               "child": [["tip"], "origin"]})",
           R"("young_modulus": 1e9, "poisson_ratio": 0.3,
              "section": "circle", "diameter": 0.05, "damping": 0.01)",
           "[[0, -1, 0], [1, 0, 0], [0, 0, 1]]", "[0, 1, 0]",
           R"({"type": "ode113", "time_span": 10.0, "time_step": 0.1,
               "relative_error": 1e-8, "absolute_error": 1e-10})");
  Beam* beam = FindBeam(context);
  ASSERT_NE(beam, nullptr);
  context.Simulate();
  /* the coordinates are in the rest axes, the tip's (the fixed axes): the
  deflection along fixed z, and the slope as a turn about A's y, fixed -x */
  const std::vector<double>& x = beam->get_displacements();
  const double w = P * L * L * L / (3.0 * E * I);
  const double slope = P * L * L / (2.0 * E * I);
  EXPECT_NEAR(x[2], w, 1e-4 * w);
  EXPECT_NEAR(x[3], slope, 1e-4 * slope); /* -(-slope) about fixed x */
  EXPECT_NEAR(x[0], 0.0, 1e-4 * w);
  EXPECT_NEAR(x[1], 0.0, 1e-4 * w);
  EXPECT_NEAR(x[4], 0.0, 1e-4 * slope);
  EXPECT_NEAR(x[5], 0.0, 1e-4 * slope);
}

TEST(Beam, EndBMustLieOnTheAxisOfEndA) {
  shift::core::Context context;
  EXPECT_THROW(LoadBeam(context, "beam_off_axis",
                        TipJoint("prismatic", "[1, 0, 0]"), kSteelTube,
                        "[[1, 0, 0], [0, 1, 0], [0, 0, 1]]", "[1, 0.1, 0]"),
               shift::core::Exception);
}
