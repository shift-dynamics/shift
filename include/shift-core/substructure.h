#ifndef SHIFT_SUBSTRUCTURE_H_
#define SHIFT_SUBSTRUCTURE_H_

#include <assert.h>
#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/body/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/topology.h"

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
    chordConstraintCount_ = count;
    constraintEquationCount_ = 6 * count;
    pr_12_.clear();
    pr_21_.clear();
    pr_12_.resize(constraintEquationCount_);
    pr_21_.resize(constraintEquationCount_);
  }

  void setqCount(unsigned int count) {
    qCount_ = count;
    pc_12_.clear();
    pc_21_.clear();
    pc_12_.resize(count);
    pc_21_.resize(count);
  }

  void setdqCount(unsigned int count) {
    dqCount_ = count;
  }

  void setqOffset(unsigned int offset, double* q) {
    qOffset_ = offset;
    q_ = q + qOffset_;
  }

  void setdqOffset(unsigned int offset, double* dq, double* deltaq) {
    dqOffset_ = offset;
    dq_ = dq + dqOffset_;
    deltaq_ = deltaq + dqOffset_;
  }

  void setJointCount(unsigned int count) {
    jointCount_ = count;
  }

  void setJointOffset(unsigned int offset) {
    jointOffset_ = offset;
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

  void setJb(bool* Jb) {
    Jb_ = Jb;
  }

  void setIndex(unsigned int index) {
    index_ = index;
  }

  unsigned int getConstraintEquationCount() {
    return constraintEquationCount_;
  }

  unsigned int getqCount() {
    return qCount_;
  }

  unsigned int getdqCount() {
    return dqCount_;
  }

  unsigned int getdqOffset() {
    return dqOffset_;
  }

  unsigned int getJointCount() {
    return jointCount_;
  }

  void setLoopOffset(unsigned int offset) {
    loopOffset_ = offset;
  }

  int getBOffset() {
    return bOffset_;
  }

  /*!
  \brief set ij mapping array and initialize to -1
  */
  void setijMapping(int* ijMapping) {
    assert(ijMapping);
    ijMapping_ = ijMapping;
    for(unsigned int i = 0; i < dqCount_ * constraintEquationCount_; i++) {
      ijMapping_[i] = -1;
    }
  }

  unsigned int getChordConstraintCount() {
    return chordConstraintCount_;
  }

  void setBOffset(int offset) {
    bOffset_ = offset;
  }

  int getRank() {
    return rank_;
  }

  int getNonZeroElementsInB();

  void setPositiveLoopMembersCompressed(
    std::vector<std::vector<int>>::iterator begin,
    std::vector<std::vector<int>>::iterator end) {
    positiveLoopMembersCompressedBegin_ = begin;
    positiveLoopMembersCompressedEnd_ = end;
  }

  void setNegativeLoopMembersCompressed(
    std::vector<std::vector<int>>::iterator begin,
    std::vector<std::vector<int>>::iterator end) {
    negativeLoopMembersCompressedBegin_ = begin;
    negativeLoopMembersCompressedEnd_ = end;
  }

  void setPO1(const Topology::DHVA& dhva);

  void setPO2(const Topology::DHVA& dhva);

  void setBooleanPO2();

  void factorPO2();

  void setBMatrixSparse();

  void evaluateSparsity(int** __restrict__ rb3, int* __restrict__ cb3,
                        std::vector<int>& pv_23);

  const std::vector<int>& getpc_12() const {
    return pc_12_;
  }

  const std::vector<int>& getpr_12() const {
    return pr_12_;
  }

  const std::vector<int>& getpc_21() const {
    return pc_21_;
  }

  const std::vector<int>& getpr_21() const {
    return pr_21_;
  }

private:
  double* cs_ = NULL; /*!< pointer to beginning of constraint array */
  double* dcs_ =
    NULL; /*!< pointer to beginning of constraint time derivative array */
  double* J_ = NULL;
  int* Jxx_ = NULL;
  bool* Jb_ = NULL;
  double* b3_;
  double* dq_ = NULL;
  double* q_ = NULL;
  double* deltaq_ = NULL;
  double epsilonRankDetermination_ = 1e-9;
  double infinityNormMultiplier_ = 2.0;
  double scaledInitialInfinityNormB_ =
    0.0; //<! scaled infinity norm after an initial full factorization
  double infinityNormB_ = 0.0; //!< current infinity norm
  int* ijMapping_ = NULL;

  std::vector<std::vector<int>>::iterator positiveLoopMembersCompressedBegin_;
  std::vector<std::vector<int>>::iterator positiveLoopMembersCompressedEnd_;

  std::vector<std::vector<int>>::iterator negativeLoopMembersCompressedBegin_;
  std::vector<std::vector<int>>::iterator negativeLoopMembersCompressedEnd_;

  Shift::Context* context_;
  unsigned int chordConstraintCount_ = 0;
  unsigned int constraintEquationCount_ = 0;
  unsigned int qCount_ = 0;
  unsigned int dqCount_ = 0;
  unsigned int jointCount_ = 0;
  unsigned int jointOffset_ = 0;
  unsigned int qOffset_ = 0;
  unsigned int dqOffset_ = 0;
  unsigned int loopOffset_ = 0;

  std::vector<int> pr_12_;
  std::vector<int> pc_12_;
  std::vector<int> pr_21_;
  std::vector<int> pc_21_;

  int bOffset_ = 0;
  unsigned int index_ = -1;
  int rank_ = -1;
};

} // namespace Shift

#endif
