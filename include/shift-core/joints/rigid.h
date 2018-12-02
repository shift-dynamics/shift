#ifndef _SHIFT_RIGIDJOINT_H_
#define _SHIFT_RIGIDJOINT_H_

#include "joint.h"

namespace Shift {

namespace Joint {

class Rigid : public JointBase {
public:
  Rigid()
    : JointBase() {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }

  Rigid(std::string name)
    : JointBase(name) {
    dof_ = 0;
    q_0_ = NULL;
    dq_0_ = NULL;
  }

  ~Rigid() {}

  void addKeys();
  void updateDH();
  void updateDHVA();

private:
};

} // namespace Joint

} // namespace Shift

#endif
