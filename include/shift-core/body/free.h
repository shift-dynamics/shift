#ifndef SHIFT_BODY_FREE_H_
#define SHIFT_BODY_FREE_H_

#include "shift-core/body/base.h"

#include <memory>

namespace Shift {

namespace Body {

class Free : public Base {
public:
  Free()
    : Base() {
    addKeys();
  }

  Free(std::string name)
    : Base(name) {
    addKeys();
  }

  ~Free() {}

  static std::unique_ptr<Free> create();

  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Body

} // namespace Shift

#endif
