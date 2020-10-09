#ifndef SHIFT_FORCEMODULE_CONTACT_H_
#define SHIFT_FORCEMODULE_CONTACT_H_

#include <memory>

#include "shift/core/forcemodule/base.h"

namespace Shift {

namespace Joint {
class JointBase;
}

namespace ForceModule {

class Contact : public Base {
public:
  Contact()
    : Base() {
    addParameters();
  }
  Contact(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Contact() override {}
  static std::unique_ptr<Contact> create();
  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

  virtual void service(const double*, double*) override {}
  virtual void connect() override {}

private:
  static bool registered_;
};

} // namespace ForceModule

} // namespace Shift

#endif
