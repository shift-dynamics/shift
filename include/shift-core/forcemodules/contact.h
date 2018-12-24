#ifndef _SHIFT_CONTACTFORCEMODULE_H_
#define _SHIFT_CONTACTFORCEMODULE_H_

#include <memory>

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
  static std::unique_ptr<Contact> create();
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
