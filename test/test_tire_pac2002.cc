/* Sign and magnitude checks of the PAC2002 tire model in the SAE contact
frame of tire::Contact (x heading, y right, z down) */

#include "gtest/gtest.h"

#include <cmath>
#include <string>

#include "shift/config.h"
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
    double states[2] = {0.0, 0.0};
    double rates[2];
    ksl_coscrew_t force;
    model.Evaluate(contact, states, rates, &force);
    return force;
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
