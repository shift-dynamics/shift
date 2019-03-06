#ifndef SHIFT_SUBSTRUCTURE_H_
#define SHIFT_SUBSTRUCTURE_H_

#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/body/base.h"
#include "shift-core/joint/base.h"

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

  void setContext(Shift::Context* context) {
    context_ = context;
  }

  void setChordConstraintCount(unsigned int count) {
    constraintEquationCount_ = 6 * count;
  }

  void setqCount(unsigned int count) {
    qCount_ = count;
  }

  void setdqCount(unsigned int count) {
    dqCount_ = count;
  }

  void setqOffset(unsigned int offset) {
    qOffset_ = offset;
  }

  void setdqOffset(unsigned int offset) {
    dqOffset_ = offset;
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

  unsigned int getJointCount() {
    return jointCount_;
  }

private:
  double* cs_ = NULL; /*!< pointer to beginning of constraint array */
  double* dcs_ =
    NULL; /*!< pointer to beginning of constraint time derivative array */
  double* J_ = NULL;
  int* Jxx_ = NULL;
  bool* Jb_ = NULL;

  Shift::Context* context_;
  unsigned int constraintEquationCount_ = 0;
  unsigned int qCount_ = 0;
  unsigned int dqCount_ = 0;
  unsigned int jointCount_ = 0;
  unsigned int jointOffset_ = 0;
  unsigned int qOffset_ = 0;
  unsigned int dqOffset_ = 0;

  std::vector<int> pr_12_;
  std::vector<int> pc_12_;
  std::vector<int> pr_21_;
  std::vector<int> pc_21_;

  unsigned int index_ = -1;
};

} // namespace Shift

#endif
