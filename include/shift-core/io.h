#ifndef _SHIFT_IO_H_
#define _SHIFT_IO_H_

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Shift Frame class
*/
class Io : Base {

public:
  Io(std::string name)
    : Base(name) {}
  ~Io() {}

private:
};

} // namespace Shift

#endif
