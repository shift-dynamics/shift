#ifndef SHIFT_BEHAVIOR_CONTACTDETECTION_H_
#define SHIFT_BEHAVIOR_CONTACTDETECTION_H_

#include <memory>

#include "shift/core/behavior/Base.h"

namespace shift {
namespace core {
  
namespace joint {
class Base;
}

namespace behavior {

class ContactDetection : public Base {
public:
  ContactDetection()
    : Base() {
    DeclareParameters();
  }
  ContactDetection(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  ~ContactDetection() {}
  static std::unique_ptr<ContactDetection> Create();

  static bool is_registered() {
    return registered_;
  }

private:
  static bool registered_;
  void DeclareParameters() override;
};

} // namespace behavior
}
} // namespace shift

#endif
