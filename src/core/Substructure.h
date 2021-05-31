#ifndef SHIFT_SUBSTRUCTURE_H_
#define SHIFT_SUBSTRUCTURE_H_

#include "shift/core/Array2d.h"
#include "shift/core/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/joint/Base.h"
#include "shift/core/Topology.h"

#include <assert.h>
#include <string>
#include <vector>

namespace shift {
namespace core {

class Context;

/*!
@brief Shift Assembly datastructure
*/
class Substructure : Base {

public:
  Substructure()
    : Base("") {}
  ~Substructure() {}

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

  unsigned int get_constraint_equation_count() const {
    return constraint_equation_count_;
  }

  unsigned int get_q_count() const {
    return q_count_;
  }

  unsigned int get_dq_count() const {
    return dq_count_;
  }

  unsigned int get_dq_offset() const {
    return dq_offset_;
  }

  unsigned int get_joint_count() const {
    return joint_count_;
  }

  int get_b_offset() const {
    return b_offset_;
  }

  unsigned int get_chord_constraint_count() const {
    return chord_constraint_count_;
  }

  int get_rank() const {
    return rank_;
  }

  unsigned int get_index() const {
    return index_;
  }

  const std::vector<Constraint>& get_constraint_indices() const {
    return constraints_;
  }

  const std::vector<int>& get_pc12() const {
    return pc_12_;
  }

  const std::vector<int>& get_pr12() const {
    return pr_12_;
  }

  const std::vector<int>& get_pc21() const {
    return pc_21_;
  }

  const std::vector<int>& get_pr21() const {
    return pr_21_;
  }

  /*! Obtain pointer to start of substructure block sub jacobian matrix */
  const double* get_jacobian_matrix() const {
    return J_;
  }

  const double* get_cs() const {
    return cs_;
  }

  const double* get_constraints() const {
    return cs_;
  }


  void set_context(shift::core::Context*);

  void set_chord_constraint_count(unsigned int count) {
    chord_constraint_count_ = count;
    constraint_equation_count_ = 6 * count;
    pr_12_.clear();
    pr_21_.clear();
    pr_12_.resize(constraint_equation_count_);
    pr_21_.resize(constraint_equation_count_);
  }

  void set_q_count(unsigned int count) {
    q_count_ = count;
    pc_12_.clear();
    pc_21_.clear();
    pc_12_.resize(count);
    pc_21_.resize(count);
  }

  void set_dq_count(unsigned int count) {
    dq_count_ = count;
  }

  void set_q_offset(unsigned int offset, double* q) {
    q_offset_ = offset;
    q_ = q + q_offset_;
  }

  void set_dq_offset(unsigned int offset, double* dq, double* delta_q) {
    dq_offset_ = offset;
    dq_ = dq + dq_offset_;
    delta_q_ = delta_q + dq_offset_;
  }

  void set_joint_count(unsigned int count) {
    joint_count_ = count;
  }

  void set_joint_offset(unsigned int offset) {
    joint_offset_ = offset;
  }

  void set_cs(double* cs) {
    cs_ = cs;
  }

  void set_dcs(double* dcs) {
    dcs_ = dcs;
  }

  void set_J(double* J) {
    J_ = J;
  }

  void set_Jxx(int* Jxx) {
    Jxx_ = Jxx;
  }

  void set_Jb() {
    Jb_.resize(dq_count_ * constraint_equation_count_);
    std::fill(Jb_.begin(), Jb_.end(), 0);
  }

  void set_index(unsigned int index) {
    index_ = index;
  }
  void set_loop_offset(unsigned int offset) {
    loop_offset_ = offset;
  }

  /*!
  \brief set ij mapping array and initialize to -1
  */
  void set_ij_mapping(int* ij_mapping) {
    assert(ij_mapping);
    ij_mapping_ = ij_mapping;
    for(unsigned int i = 0; i < dq_count_ * constraint_equation_count_; i++) {
      ij_mapping_[i] = -1;
    }
  }

  
  void set_b_offset(int offset) {
    b_offset_ = offset;
  }

  void set_right_null_space(double* right_null_space) {
    right_null_space_ = right_null_space;
  }
  
  void set_mobility_numbers(double* mobility_numbers) {
    mobility_numbers_ = mobility_numbers;
  }

  void set_chord_constraint_offset(unsigned int offset) {
    chord_constraint_offset_ = offset;
  }

  void
  set_positive_loop_members_compressed(std::vector<std::vector<int>>* members) {
    positive_loop_members_compressed_ = members;
  }

  void
  set_negative_loop_members_compressed(std::vector<std::vector<int>>* members) {
    negative_loop_members_compressed_ = members;
  }

  void Print() const;
  
  void ClearConstraintOffsets() {
    constraints_.clear();
  }

  void SetupPO1(const topology::DHVA& dhva);

  void SetupPO2(const topology::DHVA& dhva);

  void SetupBooleanPO2();

  void FactorPO2();

  void EvaluateSparsity(shift::core::Array2d<int>& rb3, std::vector<int>& cb3,
                        std::vector<int>& pv_23);

  void AppendConstraintOffset(shift::core::Substructure::Constraint constraint) {
    constraints_.push_back(constraint);
  }

  void NewtonRaphsonSolver();
  void Updateq();
  double ComputeDeltaqError();
  void FirstOrderUpdate();
  int RemoveConstraintViolations();
  int RemoveConstraintViolationsFast(bool);
  void PrintError(double initial_error, double final_error, double* q_copy);
  void IncrementSpecifiedCoordinate(shift::core::topology::DHVA& dhva,
                                    int specified_coordinate_index,
                                    double value, MoveType move_type);
  void SetupBMatrix();
  void SetupBMatrixSparse(double* __restrict__ b3);
  void SetupPO1Specified(const shift::core::topology::DHVA& dhva, int dq_index);
  double InfinityNormB();
  double EvaluateConstraints(const shift::core::topology::DHVA& dhva);
  void ComputeMobility();
  void CopyNullSpace();
  void SetupNewtonRaphsonSpecifiedCoordinate(shift::core::topology::DHVA& dhva,
                                             int specified_coordinate_index);
  /*!
  @brief call full factor again if infinity norm of B matrix
  has changed a specified threshold from initial starting value

  @param q generalized coordinates
  @param dq generalized coordinate time derivatives

  @return
  0: infinity norm check passed
  1: infinity norm check failed and newton raphson was successfully reapplied to
  determine the correct rank, the simulation can continue
  2: infinity norm check failed and newton raphson
  did not converge, the simulation should terminate
  */
  unsigned int InfinityNormCheck(double* q, const double* dq);

  /*!
  @brief check pivot ratios in block jacobian matrix associated with
  substructure and call full factor if pivot ratio has changed a specified
  threshold from initial starting value

  @param infinity_norm_multiplier override the default infinity norm multiplier
  with a custom value
  @param q generalized coordinates
  @param dq generalized coordinate time derivatives

  @return
  0: infinity norm check passed
  1: infinity norm check failed and newton raphson was successfully reapplied to
  determine the correct rank, the simulation can continue
  2: infinity norm check failed and newton raphson
  did not converge, the simulation should terminate
  */
  unsigned int InfinityNormCheck(double* q, const double* dq,
                                 const double infinity_norm_multiplier);

  void Connect();

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
  double initial_infinity_norm_b_ =
    0.0; //<! infinity norm after an initial full factorization
  double infinity_norm_b_ = 0.0; //!< current infinity norm
  int* ij_mapping_ = nullptr;

  std::vector<std::vector<int>>* positive_loop_members_compressed_ = nullptr;
  std::vector<std::vector<int>>* negative_loop_members_compressed_ = nullptr;

  shift::core::Context* context_ = nullptr;
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

  bool print_constraint_error_ = false;
};

}
} // namespace shift

#endif
