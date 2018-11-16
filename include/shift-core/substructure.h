#ifndef _SHIFT_SUBSTRUCTURE_H_
#define _SHIFT_SUBSTRUCTURE_H_

#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/body.h"
#include "shift-core/joint.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Substructure : Base {

public:
  Substructure(std::string name)
    : Base(name) {}
  ~Substructure() {}

private:
};

} // namespace Shift

#endif
