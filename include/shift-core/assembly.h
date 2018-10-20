#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

#include <string>
#include <vector>

#include "base.h"
#include "body.h"
#include "joint.h"

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
  std::vector<Assembly> m_subassemblies;
  std::vector<Body> m_bodies;
  std::vector<Joint> m_joints;
};

} // namespace Shift

#endif
