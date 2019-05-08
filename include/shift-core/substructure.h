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

  unsigned int constraintEquationCount() const {
    return constraintEquationCount_;
  }

  unsigned int qCount() const {
    return qCount_;
  }

  unsigned int dqCount() const {
    return dqCount_;
  }

  unsigned int dqOffset() const {
    return dqOffset_;
  }

  unsigned int jointCount() const {
    return jointCount_;
  }

  void setLoopOffset(unsigned int offset) {
    loopOffset_ = offset;
  }

  int bOffset() const {
    return bOffset_;
  }

  void clearConstraintOffsets() {
    constraints_.clear();
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

  unsigned int chordConstraintCount() {
    return chordConstraintCount_;
  }

  void setBOffset(int offset) {
    bOffset_ = offset;
  }

  int rank() {
    return rank_;
  }

  void print();

  void setChordConstraintOffset(unsigned int offset) {
    chordConstraintOffset_ = offset;
  }

  void
  setPositiveLoopMembersCompressed(std::vector<std::vector<int>>* members) {
    positiveLoopMembersCompressed_ = members;
  }

  void
  setNegativeLoopMembersCompressed(std::vector<std::vector<int>>* members) {
    negativeLoopMembersCompressed_ = members;
  }

  void setPO1(const Topology::DHVA& dhva);

  void setPO2(const Topology::DHVA& dhva);

  void setBooleanPO2();

  void factorPO2();

  void setBMatrixSparse(double* __restrict__ b3);

  void evaluateSparsity(int** __restrict__ rb3, int* __restrict__ cb3,
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

  std::vector<Constraint> constraints_;

  void appendConstraintOffset(Shift::Substructure::Constraint constraint) {
    constraints_.push_back(constraint);
  }

  void newtonRaphsonSolver();
  void updateq();
  double qError();
  double constraintError();
  void firstOrderUpdate();
  int removeConstraintViolations();
  int removeConstraintViolationsFast(bool);
  void printError(double initialError, double finalError, double* qCopy);
  void incrementSpecifiedCoordinate(Shift::Topology::DHVA& dhva,
                                    int specifiedCoordinateIndex, double value,
                                    MoveType moveType);
  void setBMatrix();
  void setPO1Specified(const Shift::Topology::DHVA& dhva, int dqIndex);
  double infinityNormB();
  void evaluateConstraints(const Shift::Topology::DHVA& dhva);
  void setRightNullSpace(double* rightNullSpace) {
    rightNullSpace_ = rightNullSpace;
  }
  void setMobilityNumbers(double* mobilityNumbers) {
    mobilityNumbers_ = mobilityNumbers;
  }
  void computeMobility();
  void copyNullSpace();
  void setupNewtonRaphsonSpecifiedCoordinate(Shift::Topology::DHVA& dhva,
                                             int specifiedCoordinateIndex);

private:
  double* cs_ = NULL; /*!< pointer to beginning of constraint array */
  double* dcs_ =
    NULL; /*!< pointer to beginning of constraint time derivative array */
  double* J_ = NULL;
  int* Jxx_ = NULL;
  bool* Jb_ = NULL;
  double* dq_ = NULL;
  double* q_ = NULL;
  double* deltaq_ = NULL;
  double* rightNullSpace_ = NULL;
  double* mobilityNumbers_ = NULL;
  double epsilonRankDetermination_ = 1e-9;
  double infinityNormMultiplier_ = 2.0;
  double scaledInitialInfinityNormB_ =
    0.0; //<! scaled infinity norm after an initial full factorization
  double infinityNormB_ = 0.0; //!< current infinity norm
  int* ijMapping_ = NULL;

  std::vector<std::vector<int>>* positiveLoopMembersCompressed_ = NULL;
  std::vector<std::vector<int>>* negativeLoopMembersCompressed_ = NULL;

  Shift::Context* context_ = NULL;
  unsigned int chordConstraintOffset_ = 0;
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
