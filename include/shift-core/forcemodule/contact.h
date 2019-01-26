#ifndef SHIFT_FORCEMODULE_CONTACT_H_
#define SHIFT_FORCEMODULE_CONTACT_H_

#include <memory>

#include "shift-core/forcemodule/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace ForceModule {

class Contact : public Base {
public:
  Contact()
    : Base() {
    addKeys();
  }
  Contact(std::string name)
    : Base(name) {
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
