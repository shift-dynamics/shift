#ifndef _SHIFT_JOINT_H_
#define _SHIFT_JOINT_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Abstract base class for Shift C++ objects
*/
class Joint : Base {

public:
  Joint(std::string name)
    : Base(name) {}
  ~Joint() {}

private:
};

} // namespace Shift

#endif
