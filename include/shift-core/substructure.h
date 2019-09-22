#ifndef SHIFT_SUBSTRUCTURE_H_
#define SHIFT_SUBSTRUCTURE_H_

#include "shift-core/array2d.h"
#include "shift-core/base.h"
#include "shift-core/body/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/topology.h"

#include <assert.h>
#include <string>
#include <vector>

namespace Shift {

class Context;

/*!
@brief Shift Assembly datastructure
*/
class Substructure : Base {

public:
  Substructure()
    : Base("") {}
  ~Substructure() {}

  void setContext(Shift::Context*);

  void setChordConstraintCount(unsigned int count) {
    chord_constraint_count_ = count;
    constraint_equation_count_ = 6 * count;
    pr_12_.clear();
    pr_21_.clear();
    pr_12_.resize(constraint_equation_count_);
    pr_21_.resize(constraint_equation_count_);
  }

  void setqCount(unsigned int count) {
    q_count_ = count;
    pc_12_.clear();
    pc_21_.clear();
    pc_12_.resize(count);
    pc_21_.resize(count);
  }

  void setdqCount(unsigned int count) {
    dq_count_ = count;
  }

  void setqOffset(unsigned int offset, double* q) {
    q_offset_ = offset;
    q_ = q + q_offset_;
  }

  void setdqOffset(unsigned int offset, double* dq, double* delta_q) {
    dq_offset_ = offset;
    dq_ = dq + dq_offset_;
    delta_q_ = delta_q + dq_offset_;
  }

  void setJointCount(unsigned int count) {
    joint_count_ = count;
  }

  void setJointOffset(unsigned int offset) {
    joint_offset_ = offset;
  }

  void setcs(double* cs) {
    cs_ = cs;
  }

  void setdcs(double* dcs) {
    dcs_ = dcs;
  }

  void setJ(double* J) {
    J_ = J;
  }

  void setJxx(int* Jxx) {
    Jxx_ = Jxx;
  }

  void setJb() {
    Jb_.resize(dq_count_ * constraint_equation_count_);
    std::fill(Jb_.begin(), Jb_.end(), 0);
  }

  void setIndex(unsigned int index) {
    index_ = index;
  }

  unsigned int constraintEquationCount() const {
    return constraint_equation_count_;
  }

  unsigned int qCount() const {
    return q_count_;
  }

  unsigned int dqCount() const {
    return dq_count_;
  }

  unsigned int dqOffset() const {
    return dq_offset_;
  }

  unsigned int jointCount() const {
    return joint_count_;
  }

  void setLoopOffset(unsigned int offset) {
    loop_offset_ = offset;
  }

  int bOffset() const {
    return b_offset_;
  }

  void clearConstraintOffsets() {
    constraints_.clear();
  }

  /*!
  \brief set ij mapping array and initialize to -1
  */
  void setijMapping(int* ij_mapping) {
    assert(ij_mapping);
    ij_mapping_ = ij_mapping;
    for(unsigned int i = 0; i < dq_count_ * constraint_equation_count_; i++) {
      ij_mapping_[i] = -1;
    }
  }

  unsigned int chordConstraintCount() const {
    return chord_constraint_count_;
  }

  void setBOffset(int offset) {
    b_offset_ = offset;
  }

  int rank() const {
    return rank_;
  }

  void print();

  void setChordConstraintOffset(unsigned int offset) {
    chord_constraint_offset_ = offset;
  }

  void
  setPositiveLoopMembersCompressed(std::vector<std::vector<int>>* members) {
    positive_loop_members_compressed_ = members;
  }

  void
  setNegativeLoopMembersCompressed(std::vector<std::vector<int>>* members) {
    negative_loop_members_compressed_ = members;
  }

  void setPO1(const Topology::DHVA& dhva);

  void setPO2(const Topology::DHVA& dhva);

  void setBooleanPO2();

  void factorPO2();

  void setBMatrixSparse(double* __restrict__ b3);

  void evaluateSparsity(Shift::Array2d<int>& rb3, std::vector<int>& cb3,
                        std::vector<int>& pv_23);

  const std::vector<int>& pc_12() const {
    return pc_12_;
  }

  const std::vector<int>& pr_12() const {
    return pr_12_;
  }

  const std::vector<int>& pc_21() const {
    return pc_21_;
  }

  const std::vector<int>& pr_21() const {
    return pr_21_;
  }

  enum MoveType { TARGET, INCREMENT, TARGET_SPHERICAL, INCREMENT_SPHERICAL };

  struct Constraint {
    struct {
      int d =
        0; /*!< offset into dhva.d for constraint's parent body displacement */
      int v =
        0; /*!< offset into dhva.v for constraint's parent body velocity */
    } parent;
    struct {
      int d =
        0; /*!< offset into dhva.d for constraint's child body displacement */
      int v = 0; /*!< offset into dhva.v for constraint's child body velocity */
    } child;
  };

  const std::vector<Constraint>& constraints() const {
    return constraints_;
  }

  void appendConstraintOffset(Shift::Substructure::Constraint constraint) {
    constraints_.push_back(constraint);
  }

  void newtonRaphsonSolver();
  void updateq();
  double qError();
  void firstOrderUpdate();
  int removeConstraintViolations();
  int removeConstraintViolationsFast(bool);
  void printError(double initial_error, double final_error, double* q_copy);
  void incrementSpecifiedCoordinate(Shift::Topology::DHVA& dhva,
                                    int specified_coordinate_index,
                                    double value, MoveType move_type);
  void setBMatrix();
  void setPO1Specified(const Shift::Topology::DHVA& dhva, int dq_index);
  double infinityNormB();
  double evaluateConstraints(const Shift::Topology::DHVA& dhva);
  void setRightNullSpace(double* right_null_space) {
    right_null_space_ = right_null_space;
  }
  void setMobilityNumbers(double* mobility_numbers) {
    mobility_numbers_ = mobility_numbers;
  }
  void computeMobility();
  void copyNullSpace();
  void setupNewtonRaphsonSpecifiedCoordinate(Shift::Topology::DHVA& dhva,
                                             int specified_coordinate_index);
  bool infinityNormCheck();

  const double* cs() const {
    return cs_;
  }
  const std::vector<int>& pr12() const {
    return pr_12_;
  }

  const std::vector<int>& pc12() const {
    return pc_12_;
  }

  const std::vector<int>& pr21() const {
    return pr_21_;
  }

  const std::vector<int>& pc21() const {
    return pc_21_;
  }

private:
  std::vector<Constraint> constraints_;

  double* cs_ = nullptr; /*!< pointer to beginning of constraint array */
  double* dcs_ =
    nullptr; /*!< pointer to beginning of constraint time derivative array */
  double* J_ = nullptr;
  int* Jxx_ = nullptr;
  double* dq_ = nullptr;
  double* q_ = nullptr;
  double* delta_q_ = nullptr;
  double* right_null_space_ = nullptr;
  double* mobility_numbers_ = nullptr;
  std::vector<bool> Jb_;

  double epsilon_rank_determination_ = 1e-9;
  double infinity_norm_multiplier_ = 2.0;
  double scaled_initial_infinity_norm_b_ =
    0.0; //<! scaled infinity norm after an initial full factorization
  double infinity_norm_b_ = 0.0; //!< current infinity norm
  int* ij_mapping_ = nullptr;

  std::vector<std::vector<int>>* positive_loop_members_compressed_ = nullptr;
  std::vector<std::vector<int>>* negative_loop_members_compressed_ = nullptr;

  Shift::Context* context_ = nullptr;
  unsigned int chord_constraint_offset_ = 0;
  unsigned int chord_constraint_count_ = 0;
  unsigned int constraint_equation_count_ = 0;
  unsigned int q_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int joint_count_ = 0;
  unsigned int joint_offset_ = 0;
  unsigned int q_offset_ = 0;
  unsigned int dq_offset_ = 0;
  unsigned int loop_offset_ = 0;

  std::vector<int> pr_12_;
  std::vector<int> pc_12_;
  std::vector<int> pr_21_;
  std::vector<int> pc_21_;

  int b_offset_ = 0;
  unsigned int index_ = -1;
  int rank_ = -1;
};

} // namespace Shift

#endif
