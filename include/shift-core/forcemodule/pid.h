#ifndef SHIFT_FORCEMODULE_PID_H_
#define SHIFT_FORCEMODULE_PID_H_

#include <memory>

#include "shift-core/forcemodule/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace ForceModule {

class PID : public Base {
public:
  PID()
    : Base() {
    addKeys();
  }
  PID(std::string name)
    : Base(name) {
    addKeys();
  }
  ~PID() {}
  static std::unique_ptr<PID> create();
  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace ForceModule

} // namespace Shift

#endif
