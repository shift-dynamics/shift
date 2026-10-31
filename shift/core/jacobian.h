#ifndef SHIFT_JACOBIAN_H_
#define SHIFT_JACOBIAN_H_

#include "shift/core/array2d.h"
#include "shift/core/base.h"
#include "shift/core/substructure.h"
#include "shift/core/topology.h"
#include "shift/core/parameter/list.h"

#include "ksl/memory.h"

#include <iostream>
#include <string>

namespace shift {
namespace core {

class Context;

/*!
@brief Shift Jacobian class
*/
class Jacobian
  : public Base
  , public parameter::List {

public:
  Jacobian(Context* context)
    : Base("") {
    context_ = context;
  }
  ~Jacobian() {}

public:
  enum Failure {
    OK = 0,
    INFINITY_NORM_B = 1,
    PIVOT_RATIO = 2,
    NEWTON_RAPHSON = 4
  };
  
  struct InfinityNormFailure {
    double time;
    std::vector<unsigned int> substructure_index;
  };

  bool get_newton_raphson_failure() {
    return newton_raphson_failure_;
  }

  const std::vector<Substructure>& get_substructures() const {
    return substructures_;
  }

  std::vector<Substructure>& get_substructures() {
    return substructures_;
  }

  const std::vector<bool>& get_is_dependent_po3() const {
    return is_dependent_po3_;
  }

  const std::vector<bool>& get_is_dependent_po1() const {
    return is_dependent_po1_;
  }

  const Array2d<int>& get_rb3() const {
    return rb3_;
  }

  const std::vector<int> get_cb3() const {
    return cb3_;
  }

  const std::vector<double>& get_b3() const {
    return b3_;
  }

  const std::vector<int>& get_pv31() const {
    return pv_31_;
  }

  const std::vector<int>& get_pv13() const {
    return pv_13_;
  }

  const std::vector<int>& get_pv23() const {
    return pv_23_;
  }

  const std::vector<double>& get_constraints() const {
    return e_;
  }

  const std::vector<double>& get_de() const {
    return de_;
  }

  const std::vector<InfinityNormFailure>& get_infinity_norm_failures() const {
    return infinity_norm_failures_;
  }

  virtual void DeclareParameters() override{};

  /*! \brief allocate memory for Jacobian after substructures have been
  identified

  Identifies size of various data structures, allocates memory and performs an
  initial full-factor to determine split of dependent and independent variables
  */
  void Initialize();

  /*! \brief perform full factor to set mapping from PO1 to PO2 */
  void Analyze();

  double EvaluateConstraints();
  double EvaluateConstraintsNewtonRaphson();

  void InitializeMobility();

  /*! save a copy of the Jacobian to Js for comparison */
  void Save() {
    memcpy(Js_.data(), J_.data(), J_.size() * sizeof(double));
  }

  void SetupPO1(const topology::DHVA& dhva);
  void SetupPO1();
  void TranslatePermutations();
  void SetupBooleanPO2();
  void EvaluateSparsity();
  void SetupPO2(const topology::DHVA& dhva);
  void SetupPO2();
  void FactorPO2();
  void SetupBMatrixSparse();
  void SetupConstraintPointers();
  void SetupSubstructurePointers();
  unsigned int RemoveConstraintViolations();
  void RemoveConstraintViolationsFast(bool);

  /*! check if infinity norm of B matrix is greater than a threshold times the
   * initial starting value
   * \return bool true if infinity norm is greater than initial starting value
   */
  unsigned int CheckCondition(double time, double* q, const double* dq);

  /*! check if infinity norm of B matrix is greater than a threshold times the
   * initial starting value
   * \return bool true if infinity norm is greater than initial starting value
   */
  unsigned int CheckCondition(double time, double* q, const double* dq,
                                 const double infinity_norm_multiplier);

  unsigned int CheckCondition(const double);

private:
  void SetupLookup();
  void MallocEquations();
  void CompressLoopMembers();

  bool newton_raphson_failure_ = false;

  std::vector<double> J_;  /*!< a linear vector containing all block submatrices
                              for each substructure */
  std::vector<double> Js_; /*!< saved copy of Jacobian matrix */
  std::vector<int> Jxx_;
  std::vector<bool>
    Jb_; /*!< boolean array indicating fills, used to set up RNCA algorithm */
  std::vector<double>
    e_; /*!< global array of constraints \f$\underline{e}\f$ */
  std::vector<double>
    de_; /*!< time derivative of constraints \f$\underline{\dot{e}}\f$ */
  std::vector<double> b3_;
  std::vector<int> ij_mapping_;
  Array2d<int> rb3_;
  std::vector<int> cb3_;
  std::vector<double>
    right_null_space_; /*!< right null space of Jacobian matrix */

  unsigned int jacobian_size_ =
    0; /*!< size of all block sub Jacobian matrices */
  unsigned int constraint_equation_count_ =
    0; /*!< total number of constraint equations for all block sub-Jacobian
          matrices */

  std::vector<bool> is_dependent_po1_;
  std::vector<bool> is_dependent_po3_;

  std::vector<int> pv_12_;
  std::vector<int> pv_21_;
  std::vector<int> pv_13_;
  std::vector<int> pv_31_;
  std::vector<int> pv_23_;
  std::vector<int> pv_32_;

  std::vector<std::vector<int>> positive_loop_members_compressed_;
  std::vector<std::vector<int>> negative_loop_members_compressed_;

  std::vector<Substructure> substructures_;

  unsigned int iterations_since_last_infinity_norm_failure_ = 0;

  std::vector<InfinityNormFailure> infinity_norm_failures_;
};

} // namespace core
} // namespace shift

#endif
