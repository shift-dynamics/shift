#ifndef _SHIFT_FREEBODY_
#define _SHIFT_FREEBODY_

#include "body.h"

#include <memory>

namespace Shift {

namespace Body {

class Free : public BodyBase {
public:
  Free()
    : BodyBase() {
    addKeys();
  }

  Free(std::string name)
    : BodyBase(name) {
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
