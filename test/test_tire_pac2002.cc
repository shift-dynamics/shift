/* Sign and magnitude checks of the PAC2002 tire model in the SAE contact
frame of tire::Contact (x heading, y right, z down) */

#include "gtest/gtest.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>

#include "shift/config.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/tire/pac2002.h"

#include "ksl/coscrew.h"

using shift::core::force_module::tire::Contact;
using shift::core::force_module::tire::Pac2002;
using shift::core::force_module::tire::Pac2002Coefficients;

class TirePac2002 : public testing::Test {
protected:
  void SetUp() override {
    coefficients_ = Pac2002Coefficients::Read(
      std::string(SHIFT_EXAMPLES_DIR) + "/vehicle/pac2002_235_60R15.tir");
  }

  /* upright wheel at the static load of FNOMIN, hub velocity (vx, vy) in the
  w frame, spinning at the rolling rate times (1 + slip) */
  ksl_coscrew_t Evaluate(double vx, double vy, double slip = 0.0,
                         double camber = 0.0) {
    Pac2002 model(coefficients_, 9.81);
    double states[2] = {0.0, 0.0};
    double rates[2];
    return Evaluate(model, MakeContact(vx, vy, slip, camber), states, rates);
  }

  ksl_coscrew_t Evaluate(Pac2002& model, const Contact& contact,
                         const double* states, double* rates) {
    ksl_coscrew_t force;
    model.Evaluate(contact, states, rates, &force);
    return force;
  }

  Contact MakeContact(double vx, double vy, double slip = 0.0,
                      double camber = 0.0) const {
    const double r0 = coefficients_.unloaded_radius;
    Contact contact;
    contact.in_contact = true;
    contact.depth = coefficients_.fnomin / coefficients_.vertical_stiffness;
    const double r_eff = (2.0 * r0 + (r0 - contact.depth)) / 3.0;
    contact.center_velocity_w = {{vx, vy, 0.0}};
    contact.spin_axis_w = {{0.0, std::cos(camber), std::sin(camber)}};
    contact.vx = vx;
    /* SAE: forward rolling spins negatively about the hub y axis */
    contact.omega = -vx / r_eff * (1.0 + slip);
    return contact;
  }

  /* the relaxed slips at which a transient model is in equilibrium: the
  steady state slips kappa = -Vsx / (|Vx| + e), tan(alpha) = Vsy / (|Vx| + e)
  in the ISO axes */
  void SteadySlips(const Contact& contact, double* states) const {
    const double r0 = coefficients_.unloaded_radius;
    const double r_eff = (2.0 * r0 + (r0 - contact.depth)) / 3.0;
    const double vx = contact.center_velocity_w.x;
    const double vx_reg = std::fabs(vx) + 0.1;
    const double vsx = vx - (-contact.omega) * r_eff;
    states[2] = -vsx / vx_reg;
    states[3] = -contact.center_velocity_w.y / vx_reg;
  }

  Pac2002Coefficients coefficients_;
};

TEST_F(TirePac2002, ReadsPropertyFile) {
  EXPECT_DOUBLE_EQ(coefficients_.unloaded_radius, 0.344);
  EXPECT_DOUBLE_EQ(coefficients_.fnomin, 4850);
  EXPECT_DOUBLE_EQ(coefficients_.pky1, -21.92);
  EXPECT_EQ(coefficients_.use_mode, 4);
  EXPECT_TRUE(coefficients_.relaxation);
  EXPECT_TRUE(coefficients_.measured_left);
}

TEST_F(TirePac2002, NormalForcePushesWheelUp) {
  const ksl_coscrew_t f = Evaluate(20.0, 0.0);
  // the depth was chosen for Fz = FNOMIN; z is down
  EXPECT_NEAR(f.lin.z, -coefficients_.fnomin, 1e-6 * coefficients_.fnomin);
}

TEST_F(TirePac2002, LateralForceOpposesSliding) {
  // hub sliding to the right (+y): the force pushes left (-y)
  const ksl_coscrew_t right = Evaluate(20.0, 1.0);
  EXPECT_LT(right.lin.y, -1000.0);
  const ksl_coscrew_t left = Evaluate(20.0, -1.0);
  EXPECT_GT(left.lin.y, 1000.0);
  // cornering stiffness near PKY1 * FNOMIN at the nominal load
  const double alpha = std::atan2(1.0, 20.1);
  EXPECT_NEAR(-right.lin.y / alpha,
              std::fabs(coefficients_.pky1) * coefficients_.fnomin,
              0.35 * std::fabs(coefficients_.pky1) * coefficients_.fnomin);
}

TEST_F(TirePac2002, AligningMomentTurnsWheelIntoVelocity) {
  // sliding to the right, the velocity points right of the heading; the
  // aligning moment turns the heading toward it, x toward y: +z (down)
  const ksl_coscrew_t right = Evaluate(20.0, 0.5);
  EXPECT_GT(right.ang.z, 0.0);
  const ksl_coscrew_t left = Evaluate(20.0, -0.5);
  EXPECT_LT(left.ang.z, 0.0);
}

TEST_F(TirePac2002, LongitudinalForceFollowsSlip) {
  // driving: the wheel spins faster than it rolls, the force is forward
  EXPECT_GT(Evaluate(20.0, 0.0, 0.05).lin.x, 1000.0);
  // braking: backward
  EXPECT_LT(Evaluate(20.0, 0.0, -0.05).lin.x, -1000.0);
  // and limited by the friction, about PDX1 * Fz
  EXPECT_LT(std::fabs(Evaluate(20.0, 0.0, 0.5).lin.x),
            1.1 * coefficients_.pdx1 * coefficients_.fnomin);
}

TEST_F(TirePac2002, RollingResistanceOpposesSpin) {
  // forward rolling spins negatively about y; the moment is positive
  const ksl_coscrew_t forward = Evaluate(20.0, 0.0);
  EXPECT_GT(forward.ang.y, 0.0);
  EXPECT_NEAR(forward.ang.y,
              coefficients_.qsy1 * coefficients_.unloaded_radius *
                coefficients_.fnomin,
              0.5 * coefficients_.qsy1 * coefficients_.unloaded_radius *
                coefficients_.fnomin);
  EXPECT_LT(Evaluate(-20.0, 0.0).ang.y, 0.0);
}

TEST_F(TirePac2002, MirroredTireIsSymmetric) {
  // the mirrored tire on the other side produces the mirror image forces
  const ksl_coscrew_t f = Evaluate(20.0, 0.3, 0.02, 0.01);
  coefficients_.Mirror();
  const ksl_coscrew_t m = Evaluate(20.0, -0.3, 0.02, -0.01);
  EXPECT_NEAR(m.lin.x, f.lin.x, 1e-9 * std::fabs(f.lin.x) + 1e-9);
  EXPECT_NEAR(m.lin.y, -f.lin.y, 1e-9 * std::fabs(f.lin.y) + 1e-9);
  EXPECT_NEAR(m.lin.z, f.lin.z, 1e-9 * std::fabs(f.lin.z));
  EXPECT_NEAR(m.ang.x, -f.ang.x, 1e-9 * std::fabs(f.ang.x) + 1e-9);
  EXPECT_NEAR(m.ang.z, -f.ang.z, 1e-9 * std::fabs(f.ang.z) + 1e-9);
}

TEST_F(TirePac2002, ConvertsUnitsToSI) {
  // rewrite the SI file in millimeters, kilonewtons and milliseconds, with
  // its dimensioned values scaled to match; the coefficients read the same
  std::ifstream in(std::string(SHIFT_EXAMPLES_DIR) +
                   "/vehicle/pac2002_235_60R15.tir");
  std::stringstream text;
  text << in.rdbuf();
  std::string tir = text.str();

  const double mm = 1e3, kn = 1e-3, ms = 1e3; // SI value to file value
  const std::map<std::string, double> scale = {
    {"UNLOADED_RADIUS", mm},           {"WIDTH", mm},
    {"VERTICAL_STIFFNESS", kn / mm},   {"VERTICAL_DAMPING", kn / (mm / ms)},
    {"FNOMIN", kn},                    {"VXLOW", mm / ms},
    {"LONGVL", mm / ms}};
  for(const auto& [key, factor] : scale) {
    const std::regex line("^(" + key + "\\s*=\\s*)([-+0-9.eE]+)",
                          std::regex::multiline);
    std::smatch match;
    ASSERT_TRUE(std::regex_search(tir, match, line)) << key;
    std::ostringstream value;
    value.precision(17);
    value << std::stod(match[2]) * factor;
    tir.replace(match.position(2), match.length(2), value.str());
  }
  for(const auto& [quantity, unit] :
      std::map<std::string, std::string>{{"LENGTH", "'mm'"},
                                         {"FORCE", "'kN'"},
                                         {"TIME", "'millisecond'"}}) {
    const std::regex line("^(" + quantity + "\\s*=\\s*)'[^']*'",
                          std::regex::multiline);
    ASSERT_TRUE(std::regex_search(tir, line)) << quantity;
    tir = std::regex_replace(tir, line, "$1" + unit);
  }

  const std::filesystem::path file =
    std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "pac2002_mm_kn_ms.tir";
  std::filesystem::create_directories(file.parent_path());
  std::ofstream(file) << tir;
  const Pac2002Coefficients p = Pac2002Coefficients::Read(file.string());

  const auto& si = coefficients_;
  EXPECT_NEAR(p.unloaded_radius, si.unloaded_radius, 1e-12);
  EXPECT_NEAR(p.width, si.width, 1e-12);
  EXPECT_NEAR(p.vertical_stiffness, si.vertical_stiffness,
              1e-9 * si.vertical_stiffness);
  EXPECT_NEAR(p.vertical_damping, si.vertical_damping,
              1e-9 * si.vertical_damping);
  EXPECT_NEAR(p.fnomin, si.fnomin, 1e-9 * si.fnomin);
  EXPECT_NEAR(p.vxlow, si.vxlow, 1e-12);
  EXPECT_NEAR(p.longvl, si.longvl, 1e-12);
  EXPECT_NEAR(p.qfz1, si.qfz1, 1e-9 * si.qfz1);
  // dimensionless coefficients are not scaled
  EXPECT_EQ(p.pky1, si.pky1);
  EXPECT_EQ(p.pkx1, si.pkx1);
}

TEST_F(TirePac2002, RejectsUnknownUnit) {
  const std::filesystem::path file =
    std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "pac2002_cubit.tir";
  std::filesystem::create_directories(file.parent_path());
  std::ofstream(file) << "[UNITS]\nLENGTH = 'cubit'\n";
  EXPECT_THROW(Pac2002Coefficients::Read(file.string()), shift::core::Exception);
}

TEST_F(TirePac2002, RelaxedSlipsInEquilibriumMatchSteadyState) {
  // at the steady state slips the transient model has no rates and the
  // forces of the steady state model
  Pac2002 steady(coefficients_, 9.81);
  Pac2002 transient(coefficients_, 9.81, true);
  ASSERT_EQ(transient.get_state_count(), 4u);
  const Contact contact = MakeContact(20.0, 0.4, 0.03, 0.01);
  double steady_states[2] = {0.0, 0.0}, steady_rates[2];
  const ksl_coscrew_t f =
    Evaluate(steady, contact, steady_states, steady_rates);
  double states[4] = {0.0, 0.0, 0.0, 0.0}, rates[4];
  SteadySlips(contact, states);
  const ksl_coscrew_t t = Evaluate(transient, contact, states, rates);
  EXPECT_NEAR(rates[2], 0.0, 1e-9);
  EXPECT_NEAR(rates[3], 0.0, 1e-9);
  const double tol = 1e-9 * coefficients_.fnomin;
  EXPECT_NEAR(t.lin.x, f.lin.x, tol);
  EXPECT_NEAR(t.lin.y, f.lin.y, tol);
  EXPECT_NEAR(t.lin.z, f.lin.z, tol);
  EXPECT_NEAR(t.ang.x, f.ang.x, tol);
  EXPECT_NEAR(t.ang.y, f.ang.y, tol);
  EXPECT_NEAR(t.ang.z, f.ang.z, tol);
}

TEST_F(TirePac2002, RelaxedSlipsLagKinematicSlips) {
  // from zero relaxed slips a step in slip produces no slip force at first,
  // and the relaxed slips move toward the kinematic slips with the time
  // constant sigma / Vx
  Pac2002 transient(coefficients_, 9.81, true);
  const Contact contact = MakeContact(20.0, 0.4, 0.03);
  double states[4] = {0.0, 0.0, 0.0, 0.0}, rates[4];
  const ksl_coscrew_t t = Evaluate(transient, contact, states, rates);
  EXPECT_LT(std::fabs(t.lin.y), 0.05 * coefficients_.fnomin);
  double target[4];
  SteadySlips(contact, target);
  const double fz = coefficients_.fnomin;
  const double sigma_kappa = transient.RelaxationLengthKappa(fz);
  const double sigma_alpha = transient.RelaxationLengthAlpha(fz, 0.0);
  EXPECT_GT(target[2], 0.0); // driving
  EXPECT_LT(target[3], 0.0); // sliding right: alpha < 0 in ISO
  EXPECT_NEAR(rates[2], 20.1 * target[2] / sigma_kappa,
              1e-6 * std::fabs(rates[2]));
  EXPECT_NEAR(rates[3], 20.1 * target[3] / sigma_alpha,
              1e-6 * std::fabs(rates[3]));
  // relaxation lengths at the nominal load: PTX1 R0 and PTY1 sin(2 atan(1 /
  // PTY2)) R0
  const double r0 = coefficients_.unloaded_radius;
  EXPECT_NEAR(sigma_kappa, coefficients_.ptx1 * r0, 1e-12);
  EXPECT_NEAR(sigma_alpha,
              coefficients_.pty1 *
                std::sin(2.0 * std::atan(1.0 / coefficients_.pty2)) * r0,
              1e-12);
}
