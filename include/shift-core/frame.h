#ifndef _SHIFT_FRAME_H_
#define _SHIFT_FRAME_H_

#include <string>

#include "base.h"

namespace Shift {

/*!
@brief Shift Frame class
*/
class Frame : Base {

public:
  Frame(std::string name)
    : Base(name) {}
  ~Frame() {}

private:
};

} // namespace Shift

#endif
