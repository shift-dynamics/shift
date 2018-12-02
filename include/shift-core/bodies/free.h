#ifndef _SHIFT_FREEBODY_

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

private:
};

} // namespace Body

} // namespace Shift

#endif
