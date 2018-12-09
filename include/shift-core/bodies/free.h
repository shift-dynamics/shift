#ifndef _SHIFT_FREEBODY_
#define _SHIFT_FREEBODY_

#include "body.h"

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

  static Free* create();

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
