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
