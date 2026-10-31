#ifndef SHIFT_BODY_FREE_H_
#define SHIFT_BODY_FREE_H_

#include "shift/core/body/base.h"

#include <memory>

namespace shift {
namespace core {
namespace body {

class Free : public Base {
public:
  Free()
    : Base() {
    DeclareParameters();
    is_articulated_ = false;
  }

  Free(std::string name)
    : Base(name) {
    DeclareParameters();
    is_articulated_ = false;
  }

  virtual ~Free() override {}

  static std::unique_ptr<Free> create();

  void DeclareParameters() override;

  static bool is_registered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
