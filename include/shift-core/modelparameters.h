#ifndef _SHIFT_MODELPARAMETERS_H_
#define _SHIFT_MODELPARAMETERS_H_

#include <string>

#include "shift-core/base.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class ModelParameters : Base {

public:
  ModelParameters(std::string name)
    : Base(name) {}
  ~ModelParameters() {}

private:
};

} // namespace Shift

#endif
