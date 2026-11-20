/*
 * Moving coordinates inside a kinematic loop while the dependent coordinates
 * keep the loop closed: a single coordinate with
 * Substructure::IncrementSpecifiedCoordinate, and the rotation of a spherical
 * or float joint, with all three rotation coordinates held independent, with
 * Substructure::SetSpecifiedRotation. In the abiflexx model the joints follow
 * spherical joints, so their q and dq indices differ.
 */
#include "gtest/gtest.h"

#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "shift/config.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/joint/base.h"
#include "shift/core/substructure.h"

#include "ksl/mat3x3.h"
#include "ksl/quaternion.h"

/* the quaternion with the roll of q incremented by delta, R = Rz(yaw)
 * Ry(pitch) Rx(roll) */
static ksl_quaternion_t IncrementRoll(const double* q, double delta) {
  ksl_quaternion_t current;
  for(int i = 0; i < 4; i++) {
    current.at[i] = q[i];
  }
  ksl_mat3x3_t r;
  ksl_quaternion_toMat3x3(&current, &r);
  ksl_vec3_t angles; /* [yaw, pitch, roll] */
  ksl_vec3_t reference = ksl_vec3(0.0, 0.0, 0.0);
  ksl_mat3x3_getEulerAnglesWithReference(&r, KSL_AXIS_ZYX, &angles,
                                         &reference);
  angles.at[2] += delta;
  ksl_mat3x3_setFromEulerAngles(&r, KSL_AXIS_ZYX, &angles);
  ksl_quaternion_t target;
  ksl_mat3x3_toQuaternion(&r, &target);
  return target;
}

class SpecifiedCoordinates : public testing::Test {
protected:
  void SetUp() override {
    // run in the build tree so the model's log file stays out of examples/
    const std::filesystem::path dir =
      std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "specified_coordinates";
    std::filesystem::create_directories(dir);
    previous_dir_ = std::filesystem::current_path();
    std::filesystem::current_path(dir);
    context_.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                      "/abiflexx_d4_1300/abiflexx_d4_1300.json");
    q0_.assign(context_.get_q().begin(), context_.get_q().end());
  }
  void TearDown() override {
    std::filesystem::current_path(previous_dir_);
  }

  void Reset() {
    std::copy(q0_.begin(), q0_.end(), context_.get_q().begin());
    context_.UpdateDH();
  }

  shift::core::Context context_;
  std::vector<double> q0_;
  std::filesystem::path previous_dir_;
};

TEST_F(SpecifiedCoordinates, SphericalJointsInLoopFollowRoll) {
  int moved = 0;
  for(auto* joint : context_.get_active_joints()) {
    if(joint->get_type() != "spherical" || !joint->get_substructure()) {
      continue;
    }
    SCOPED_TRACE(joint->get_name());
    Reset();
    shift::core::Substructure* substructure = joint->get_substructure();
    int q_index, dq_index;
    shift::core::Substructure::RotationOffsets(joint, q_index, dq_index);

    substructure->SetupNewtonRaphsonSpecifiedRotation(context_.get_dhva(),
                                                      joint);
    const ksl_quaternion_t target =
      IncrementRoll(&context_.get_q()[q_index], 0.02);
    ASSERT_NO_THROW(substructure->SetSpecifiedRotation(context_.get_dhva(),
                                                       joint, target.at));

    // the joint holds the target rotation exactly (up to the sign of q) ...
    double dot = 0.0;
    for(int i = 0; i < 4; i++) {
      dot += context_.get_q()[q_index + i] * target.at[i];
    }
    EXPECT_NEAR(std::fabs(dot), 1.0, 1e-12);

    // ... the loops are closed ...
    context_.UpdateDH();
    EXPECT_LT(substructure->EvaluateConstraints(context_.get_dhva()), 1e-9);

    // ... and the dependent coordinates moved to keep them closed
    double change = 0.0;
    for(size_t i = 0; i < q0_.size(); i++) {
      if(static_cast<int>(i) < q_index || static_cast<int>(i) >= q_index + 4) {
        change = std::fmax(change, std::fabs(context_.get_q()[i] - q0_[i]));
      }
    }
    EXPECT_GT(change, 1e-3);
    moved++;
  }
  EXPECT_EQ(moved, 12);
}

TEST_F(SpecifiedCoordinates, FixedPlatformOrientationIsRejected) {
  // the legs of the parallel mechanism fix the orientation of its platform,
  // so a roll of the platform's float joint is not possible
  shift::core::joint::Base* platform = nullptr;
  for(auto* joint : context_.get_active_joints()) {
    if(joint->get_type() == "float" && joint->get_substructure()) {
      platform = joint;
    }
  }
  ASSERT_NE(platform, nullptr);
  Reset();
  shift::core::Substructure* substructure = platform->get_substructure();
  int q_index, dq_index;
  shift::core::Substructure::RotationOffsets(platform, q_index, dq_index);
  substructure->SetupNewtonRaphsonSpecifiedRotation(context_.get_dhva(),
                                                    platform);
  const std::vector<double> before(context_.get_q().begin(),
                                   context_.get_q().end());
  const ksl_quaternion_t target =
    IncrementRoll(&context_.get_q()[q_index], 0.02);

  EXPECT_THROW(substructure->SetSpecifiedRotation(context_.get_dhva(),
                                                  platform, target.at),
               shift::core::Exception);

  // the coordinates are restored
  for(size_t i = 0; i < before.size(); i++) {
    EXPECT_EQ(context_.get_q()[i], before[i]) << "q[" << i << "]";
  }
}

TEST_F(SpecifiedCoordinates, RevoluteJointsInLoopFollowIncrementAndTarget) {
  int moved = 0;
  int q_dq_differ = 0;
  for(auto* joint : context_.get_active_joints()) {
    if(joint->get_type() != "revolute" || !joint->get_substructure()) {
      continue;
    }
    SCOPED_TRACE(joint->get_name());
    Reset();
    shift::core::Substructure* substructure = joint->get_substructure();
    const int q_index = joint->get_q_index();
    const int dq_index = joint->get_dq_index();
    q_dq_differ += q_index != dq_index;

    substructure->SetupNewtonRaphsonSpecifiedCoordinate(context_.get_dhva(),
                                                        dq_index);
    const double start = context_.get_q()[q_index];
    ASSERT_NO_THROW(substructure->IncrementSpecifiedCoordinate(
      context_.get_dhva(), dq_index, 0.02,
      shift::core::Substructure::INCREMENT));
    EXPECT_NEAR(context_.get_q()[q_index], start + 0.02, 1e-14);
    context_.UpdateDH();
    EXPECT_LT(substructure->EvaluateConstraints(context_.get_dhva()), 1e-9);

    // every quaternion of the loop stays a unit quaternion
    for(auto* other : context_.get_active_joints()) {
      if(other->get_substructure() != substructure ||
         (other->get_type() != "spherical" && other->get_type() != "float")) {
        continue;
      }
      int rq, rdq;
      shift::core::Substructure::RotationOffsets(other, rq, rdq);
      double norm = 0.0;
      for(int i = 0; i < 4; i++) {
        norm += context_.get_q()[rq + i] * context_.get_q()[rq + i];
      }
      EXPECT_NEAR(norm, 1.0, 1e-12) << other->get_name();
    }

    ASSERT_NO_THROW(substructure->IncrementSpecifiedCoordinate(
      context_.get_dhva(), dq_index, start,
      shift::core::Substructure::TARGET));
    EXPECT_NEAR(context_.get_q()[q_index], start, 1e-14);
    context_.UpdateDH();
    EXPECT_LT(substructure->EvaluateConstraints(context_.get_dhva()), 1e-9);
    moved++;
  }
  EXPECT_GT(moved, 0);
  EXPECT_EQ(q_dq_differ, moved);
}

/* the slider-crank of examples/slider_crank, the worked example of the
 * Topology page: a crank of 0.6 m (joint A, angle theta) and a connecting rod
 * of 1.5 m (joint D, from the crank) drive a piston along y (joint B,
 * displacement s); joint C, from the rod to the piston, is the chord. One
 * loop of four coordinates has rank 3 and one degree of freedom. */
class SliderCrank : public testing::Test {
protected:
  void SetUp() override {
    const std::filesystem::path dir =
      std::filesystem::path(SHIFT_BINARY_DIR) / "test" / "slider_crank_partition";
    std::filesystem::create_directories(dir);
    previous_dir_ = std::filesystem::current_path();
    std::filesystem::current_path(dir);
    context_.LoadJSON(std::string(SHIFT_EXAMPLES_DIR) +
                      "/slider_crank/slider_crank.json");
    for(auto* joint : context_.get_active_joints()) {
      joints_[joint->get_name().substr(0, 1)] = joint;
    }
    substructure_ = joints_["A"]->get_substructure();
  }
  void TearDown() override {
    std::filesystem::current_path(previous_dir_);
  }

  double q(const std::string& joint) {
    return context_.get_q()[joints_[joint]->get_q_index()];
  }

  /* the joint (A, B, C or D) of each column of the substructure in the
   * pivot order of complete pivoting: dependent first, then independent */
  std::string PivotOrder() {
    std::string order;
    for(int column : substructure_->get_pc12()) {
      const int dq = int(substructure_->get_dq_offset()) + column;
      for(auto& [name, joint] : joints_) {
        if(joint->get_dq_index() == dq) {
          order += name;
        }
      }
    }
    return order;
  }

  shift::core::Context context_;
  std::map<std::string, shift::core::joint::Base*> joints_;
  shift::core::Substructure* substructure_ = nullptr;
  std::filesystem::path previous_dir_;
};

TEST_F(SliderCrank, PartitionHasRankThreeAndOneDegreeOfFreedom) {
  ASSERT_NE(substructure_, nullptr);
  EXPECT_EQ(substructure_->get_rank(), 3);
  EXPECT_EQ(substructure_->get_dq_count(), 4u);
  /* complete pivoting leaves the crank angle independent */
  EXPECT_EQ(PivotOrder().back(), 'A');
  EXPECT_NEAR(substructure_->InfinityNormB(), 1.245, 1e-3);
}

/* the crank angle specified: the piston follows the closed form
 * s = 0.6 cos(theta) + sqrt(1.5^2 - (0.6 sin(theta))^2) */
TEST_F(SliderCrank, CrankAngleGivesThePistonPosition) {
  const int dq = joints_["A"]->get_dq_index();
  substructure_->SetupNewtonRaphsonSpecifiedCoordinate(context_.get_dhva(), dq);
  EXPECT_EQ(PivotOrder().back(), 'A');
  for(double theta : {5.6, 6.0, 6.5, 7.0}) {
    ASSERT_NO_THROW(substructure_->IncrementSpecifiedCoordinate(
      context_.get_dhva(), dq, theta, shift::core::Substructure::TARGET));
    const double s =
      0.6 * std::cos(theta) +
      std::sqrt(1.5 * 1.5 - std::pow(0.6 * std::sin(theta), 2));
    EXPECT_LT(substructure_->InfinityNormB(), 1.4);
    EXPECT_NEAR(q("A"), theta, 1e-14);
    EXPECT_NEAR(q("B"), s, 1e-8);
    context_.UpdateDH();
    EXPECT_LT(substructure_->EvaluateConstraints(context_.get_dhva()), 1e-9);
  }
}

/* the piston specified: near the outer dead centre, s = 2.1, the crank turns
 * ever faster per unit of piston travel, and beyond it no configuration
 * exists */
TEST_F(SliderCrank, PistonCannotDriveThroughDeadCentre) {
  const int dq = joints_["B"]->get_dq_index();
  substructure_->SetupNewtonRaphsonSpecifiedCoordinate(context_.get_dhva(), dq);
  EXPECT_EQ(PivotOrder().back(), 'B');
  /* |B|inf grows as gap^(-1/2), by sqrt(10) per decade */
  const double expected[4] = {3.10, 6.37, 19.68, 62.12};
  int k = 0;
  for(double gap : {1e-1, 1e-2, 1e-3, 1e-4}) {
    ASSERT_NO_THROW(substructure_->IncrementSpecifiedCoordinate(
      context_.get_dhva(), dq, 2.1 - gap, shift::core::Substructure::TARGET));
    EXPECT_NEAR(q("B"), 2.1 - gap, 1e-14);
    EXPECT_NEAR(substructure_->InfinityNormB(), expected[k++], 0.01) << gap;
  }
  const std::vector<double> before(context_.get_q().begin(),
                                   context_.get_q().end());
  EXPECT_THROW(substructure_->IncrementSpecifiedCoordinate(
                 context_.get_dhva(), dq, 2.11,
                 shift::core::Substructure::TARGET),
               shift::core::Exception);
  for(size_t i = 0; i < before.size(); i++) {
    EXPECT_EQ(context_.get_q()[i], before[i]) << "q[" << i << "]";
  }
}
