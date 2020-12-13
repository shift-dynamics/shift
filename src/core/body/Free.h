#ifndef SHIFT_BODY_FREE_H_
#define SHIFT_BODY_FREE_H_

#include "shift/core/body/Base.h"

#include <memory>

namespace shift {
namespace core {
namespace body {

class Free : public Base {
public:
  Free()
    : Base() {
    addParameters();
    is_articulated_ = false;
  }

  Free(std::string name)
    : Base(name) {
    addParameters();
    is_articulated_ = false;
  }

  virtual ~Free() override {}

  static std::unique_ptr<Free> create();

  void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
