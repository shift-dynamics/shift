#ifndef _SHIFT_CONTACTDETECTIONBEHAVIOR_H_
#define _SHIFT_CONTACTDETECTIONBEHAVIOR_H_

#include <memory>

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
  static std::unique_ptr<ContactDetection> create();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  void addKeys();
};

} // namespace Behavior

} // namespace Shift

#endif
