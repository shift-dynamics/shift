#ifndef SHIFT_JACOBIAN_H_
#define SHIFT_JACOBIAN_H_

#include "shift/core/array2d.h"
#include "shift/core/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/substructure.h"
#include "shift/core/topology.h"

#include "ksl/memory.h"

#include <iostream>
#include <string>

namespace shift {

class Context;

/*!
@brief Shift Jacobian class
*/
class Jacobian
  : public Base
  , public Parameter::List {

public:
  Jacobian(shift::Context* context)
    : Base("") {
    context_ = context;
  }
  ~Jacobian() {}

public:
  virtual void addParameters() override{};

  /*! \brief allocate memory for Jacobian after substructures have been
  identified

  Identifies size of various data structures, allocates memory and performs an
  initial full-factor to determine split of dependent and independent variables
*/
  void initialize();

  /*! \brief perform full factor to set mapping from PO1 to PO2 */
  void analyze();

  const std::vector<shift::Substructure>& substructures() const {
    return substructures_;
  }
  double evaluateConstraints();

  void initializeMobility();

  bool newtonRaphsonFailure() {
    return newton_raphson_failure_;
  }

  /*! save a copy of the Jacobian to Js for comparison */
  void save() {
    memcpy(Js_.data(), J_.data(), J_.size() * sizeof(double));
  }

  void setPO1(const shift::Topology::DHVA& dhva);
  void setPO1();
  void translatePermutations();
  void setBooleanPO2();
  void evaluateSparsity();
  void setPO2(const shift::Topology::DHVA& dhva);
  void setPO2();
  void factorPO2();
  void setBMatrixSparse();
  void setConstraintPointers();
  void setSubstructurePointers();
  int removeConstraintViolations();
  void removeConstraintViolationsFast(bool);
  const std::vector<bool>& isDependentPO3() const {
    return is_dependent_po3_;
  }
  const std::vector<bool>& isDependentPO1() const {
    return is_dependent_po1_;
  }
  const shift::Array2d<int>& rb3() const {
    return rb3_;
  }
  const std::vector<int> cb3() const {
    return cb3_;
  }

  /*! check if infinity norm of B matrix is greater than a threshold times the
   * initial starting value
   * \return bool true if infinity norm is greater than initial starting value
   */
  unsigned int infinityNormCheck(double time, double* q, const double* dq);

  /*! check if infinity norm of B matrix is greater than a threshold times the
   * initial starting value
   * \return bool true if infinity norm is greater than initial starting value
   */
  unsigned int infinityNormCheck(double time, double* q, const double* dq,
                                 const double infinity_norm_multiplier);

  const std::vector<double>& b3() const {
    return b3_;
  }

  const std::vector<int>& pv31() const {
    return pv_31_;
  }

  const std::vector<int>& pv13() const {
    return pv_13_;
  }

  const std::vector<int>& pv23() const {
    return pv_23_;
  }

  const std::vector<double>& constraints() const {
    return cs_;
  }

  bool infinityNormCheck(const double);

  struct InfinityNormFailure {
    double time;
    std::vector<unsigned int> substructure_index;
  };

  const std::vector<InfinityNormFailure>& infinityNormFailures() const {
    return infinity_norm_failures_;
  }

private:
  bool newton_raphson_failure_ = false;

  std::vector<double> J_;
  std::vector<double> Js_;
  std::vector<int> Jxx_;
  std::vector<bool> Jb_;
  std::vector<double> cs_;
  std::vector<double> dcs_;
  std::vector<double> b3_;
  std::vector<int> ij_mapping_;
  shift::Array2d<int> rb3_;
  std::vector<int> cb3_;
  std::vector<double> right_null_space_;

  unsigned int jacobian_size_ =
    0; /*!< size of all block sub Jacobian matrices */
  unsigned int constraint_equation_count_ =
    0; /*!< total number of constraint equations in all block sub-Jacobian
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

  std::vector<shift::Substructure> substructures_;

  void setLookup();
  void mallocEquations();
  void compressLoopMembers();

  unsigned int iterations_since_last_infinity_norm_failure_ = 0;

  std::vector<InfinityNormFailure> infinity_norm_failures_;
};

} // namespace shift

#endif
