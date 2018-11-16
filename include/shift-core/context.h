#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/modelparameters.h"
#include "shift-core/terrain.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , model_parameters_(name)
    , root_assembly_("fixed") {}
  ~Context() {}

private:
  ModelParameters model_parameters_;
  Assembly root_assembly_;
  std::vector<Terrain> terrains_;
};

} // namespace Shift

#endif
