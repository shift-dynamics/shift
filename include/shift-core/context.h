#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <string>

#include "base.h"
#include "modelparameters.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , m_modelparameters(name) {}
  ~Context() {}

private:
  ModelParameters m_modelparameters;
};

} // namespace Shift

#endif
