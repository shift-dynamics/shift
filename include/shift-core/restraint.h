#ifndef _SHIFT_RESTRAINT_H_
#define _SHIFT_RESTRAINT_H_

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Shift Frame class
*/
class Restraint : Base {

public:
  Restraint(std::string name)
    : Base(name) {}
  ~Restraint() {}

private:
};

} // namespace Shift

#endif
