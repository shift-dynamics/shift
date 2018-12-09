#ifndef _SHIFT_PIDFORCEMODULE_H_
#define _SHIFT_PIDFORCEMODULE_H_

#include "shift-core/forcemodules/forcemodule.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace ForceModule {

class PID : public ForceModuleBase {
public:
  PID()
    : ForceModuleBase() {
    addKeys();
  }
  PID(std::string name)
    : ForceModuleBase(name) {
    addKeys();
  }
  ~PID() {}
  static PID* create();
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
