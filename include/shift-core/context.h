#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <string>

#include "shift-core/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , modelparameters_(name) {}
  ~Context() {}

private:
  ModelParameters modelparameters_;
};

} // namespace Shift

#endif
