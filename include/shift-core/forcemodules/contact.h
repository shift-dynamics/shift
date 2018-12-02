#ifndef _SHIFT_CONTACTFORCEMODULE_H_
#define _SHIFT_CONTACTFORCEMODULE_H_

#include "shift-core/forcemodules/forcemodule.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace ForceModule {

class Contact : public ForceModuleBase {
public:
  Contact()
    : ForceModuleBase() {
    addKeys();
  }
  Contact(std::string name)
    : ForceModuleBase(name) {
    addKeys();
  }
  ~Contact() {}
  static Contact* create();
  void addKeys();

private:
};

} // namespace ForceModule

} // namespace Shift

#endif
