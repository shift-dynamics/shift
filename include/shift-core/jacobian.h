#ifndef SHIFT_JACOBIAN_H_
#define SHIFT_JACOBIAN_H_

#include <iostream>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"
#include "shift-core/substructure.h"
#include "shift-core/topology.h"

#include "ksl/memory.h"

namespace Shift {

class Context;

/*!
@brief Shift Jacobian class
*/
class Jacobian
  : public Base
  , public Serialize {

public:
  Jacobian(Shift::Context* context)
    : Base("") {
    context_ = context;
  }
  ~Jacobian() {
    if(J_) {
      delete[] J_;
      J_ = NULL;
    }

    if(Jxx_) {
      delete[] Jxx_;
      Jxx_ = NULL;
    }

    if(Jb_) {
      delete[] Jb_;
      Jb_ = NULL;
    }

    if(cs_) {
      delete[] cs_;
      cs_ = NULL;
    }

    if(dcs_) {
      delete[] dcs_;
      dcs_ = NULL;
    }

    if(b3_) {
      delete[] b3_;
      b3_ = NULL;
    }
    if(ijMapping_) {
      delete[] ijMapping_;
      ijMapping_ = NULL;
    }
    if(cb3_) {
      delete[] cb3_;
      cb3_ = NULL;
    }
    if(rightNullSpace_) {
      delete[] rightNullSpace_;
    }

    ksl_array2Di_free(rb3_);
  }

public:
  /*! \brief allocate memory for Jacobian after substructures have been
  identified

  Identifies size of various data structures, allocates memory and performs an
  initial full-factor to determine split of dependent and independent variables
*/
  void initialize();

  /*! \brief perform full factor to set mapping from PO1 to PO2 */
  void analyze();

  const std::vector<Shift::Substructure>& substructures() const {
    return substructures_;
  }

  void initializeMobility();

private:
  Shift::Context* context_ = NULL;

  double* J_ = NULL; /*! array to hold entries of Jacobian */
  int* Jxx_ = NULL;  /*!< integer array */
  bool* Jb_ = NULL;
  double* cs_ = NULL; /*!< constraints */
  double* dcs_ = NULL;
  double* b3_ = NULL;
  int* ijMapping_ = NULL;
  int** rb3_ = NULL;
  int* cb3_ = NULL;
  double* rightNullSpace_ = NULL;

  unsigned int jacobianSize_ =
    0; /*!< size of all block sub Jacobian matrices */
  unsigned int constraintEquationCount_ =
    0; /*!< total number of constraint equations in all block sub-Jacobian
          matrices */

  std::vector<bool> isDependentPO1_;
  std::vector<bool> isDependentPO3_;

  std::vector<int> pv_12_;
  std::vector<int> pv_21_;
  std::vector<int> pv_13_;
  std::vector<int> pv_31_;
  std::vector<int> pv_23_;
  std::vector<int> pv_32_;

  std::vector<std::vector<int>> positiveLoopMembersCompressed_;
  std::vector<std::vector<int>> negativeLoopMembersCompressed_;

  std::vector<Shift::Substructure> substructures_;

  void setLookup();
  void mallocEquations();
  void compressLoopMembers();
  void setPO1(const Shift::Topology::DHVA& dhva);
  void translatePermutations();
  void setBooleanPO2();
  void evaluateSparsity();
  void setPO2(const Shift::Topology::DHVA& dhva);
  void factorPO2();
  void setBMatrixSparse();
  void setConstraintPointers();
  void setSubstructurePointers();
  int removeConstraintViolations();
  void removeConstraintViolationsFast(bool);
};

} // namespace Shift

#endif
