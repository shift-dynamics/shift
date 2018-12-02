#ifndef _SHIFT_CONTACTDETECTIONBEHAVIOR_H_
#define _SHIFT_CONTACTDETECTIONBEHAVIOR_H_

#include "shift-core/behaviors/behavior.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Behavior {

class ContactDetection : public BehaviorBase {
public:
  ContactDetection()
    : BehaviorBase() {
    addKeys();
  }
  ContactDetection(std::string name)
    : BehaviorBase(name) {
    addKeys();
  }
  ~ContactDetection() {}
  static ContactDetection* create();

private:
  void addKeys();
};

} // namespace Behavior

} // namespace Shift

#endif
