#ifndef SHIFT_BEHAVIOR_CONTACTDETECTION_H_
#define SHIFT_BEHAVIOR_CONTACTDETECTION_H_

#include <memory>

#include "shift/core/behavior/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace Behavior {

class ContactDetection : public Base {
public:
  ContactDetection()
    : Base() {
    addParameters();
  }
  ContactDetection(std::string name)
    : Base(name) {
    addParameters();
  }
  ~ContactDetection() {}
  static std::unique_ptr<ContactDetection> create();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  void addParameters() override;
};

} // namespace Behavior

} // namespace Shift

#endif
