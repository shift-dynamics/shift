#ifndef SHIFT_JACOBIAN_H_
#define SHIFT_JACOBIAN_H_

#include <iostream>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"
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
  , public Parameter::List {

public:
  Jacobian(Shift::Context* context)
    : Base("") {
    context_ = context;
  }
  ~Jacobian() {
    if(J_) {
      delete[] J_;
      J_ = nullptr;
    }

    if(Jxx_) {
      delete[] Jxx_;
      Jxx_ = nullptr;
    }

    if(Jb_) {
      delete[] Jb_;
      Jb_ = nullptr;
    }

    if(cs_) {
      delete[] cs_;
      cs_ = nullptr;
    }

    if(dcs_) {
      delete[] dcs_;
      dcs_ = nullptr;
    }

    if(b3_) {
      delete[] b3_;
      b3_ = nullptr;
    }
    if(ij_mapping_) {
      delete[] ij_mapping_;
      ij_mapping_ = nullptr;
    }
    if(cb3_) {
      delete[] cb3_;
      cb3_ = nullptr;
    }
    if(right_null_space_) {
      delete[] right_null_space_;
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
  double* J_ = nullptr; /*! array to hold entries of Jacobian */
  int* Jxx_ = nullptr;  /*!< integer array */
  bool* Jb_ = nullptr;
  double* cs_ = nullptr; /*!< constraints */
  double* dcs_ = nullptr;
  double* b3_ = nullptr;
  int* ij_mapping_ = nullptr;
  int** rb3_ = nullptr;
  int* cb3_ = nullptr;
  double* right_null_space_ = nullptr;

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
