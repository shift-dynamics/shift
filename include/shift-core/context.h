#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

class Terrain;

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
  void readJSON(const std::string filename);
  void writeJSON();
  void writeJSON(const std::string filename);

private:
  ModelParameters model_parameters_;
  Assembly root_assembly_;
  std::vector<Terrain*> terrains_;
  std::string filename_;
};

} // namespace Shift

#endif
