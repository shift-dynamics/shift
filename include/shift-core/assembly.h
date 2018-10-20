#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/body.h"
#include "shift-core/joint.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Assembly : Base {

public:
  Assembly(std::string name)
    : Base(name) {}
  ~Assembly() {}

private:
  std::vector<Assembly> subassemblies_;
  std::vector<Body> bodies_;
  std::vector<Joint> joints_;
};

} // namespace Shift

#endif
