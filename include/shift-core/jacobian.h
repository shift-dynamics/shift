#ifndef SHIFT_JACOBIAN_H_
#define SHIFT_JACOBIAN_H_

#include <iostream>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"
#include "shift-core/substructure.h"

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

private:
  Shift::Context* context_ = NULL;
  /*! array to hold entries of Jacobian */
  double* J_ = NULL;
  /*! integer array */
  int* Jxx_ = NULL;
  bool* Jb_ = NULL;
  double* cs_ = NULL; /*!< constraints */
  double* dcs_ = NULL;

  unsigned int jacobianSize_ =
    0; /*!< size of all block sub Jacobian matrices */
  unsigned int constraintEquationCount_ =
    0; /*!< total number of constraint equations in all block sub-Jacobian
          matrices */
  std::vector<int> jointLookup_;
  std::vector<int> dqLookup_;
  std::vector<int> qLookup_;
  std::vector<int> dqLookupCompressed_;

  std::vector<bool> isDependentPO1_;
  std::vector<bool> isDependentPO3_;

  std::vector<int> pv_12_;
  std::vector<int> pv_21_;
  std::vector<int> pv_13_;
  std::vector<int> pv_31_;
  std::vector<int> pv_23_;
  std::vector<int> pv_32_;

  std::vector<Shift::Substructure> substructures_;

  void setLookup();
  void mallocEquations();
  void compressLoopMembers();
  void setPO1();
  void translatePermutations();
  void setBooleanPO2();
  void evaluateSparsity();
  void setPO2();
  void factorPO2();
  void setBMatrixSparse();
  void setConstraintPointers();
  void setSubstructurePointers();
};

} // namespace Shift

#endif
