#ifndef SHIFT_FORCEMODULE_CONTACT_H_
#define SHIFT_FORCEMODULE_CONTACT_H_

#include <memory>

#include "shift/core/forcemodule/Base.h"

namespace shift {
namespace core {

namespace joint {
class JointBase;
}

namespace forcemodule {

class Contact : public Base {
public:
  Contact()
    : Base() {
    DeclareParameters();
  }
  Contact(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Contact() override {}
  static std::unique_ptr<Contact> Create();
  void DeclareParameters() override;

  static bool is_registered() {
    return registered_;
  }

  virtual void Service(const double*, double*) override {}
  virtual void Connect() override {}

private:
  static bool registered_;
};

} // namespace forcemodule
}
} // namespace shift

#endif
