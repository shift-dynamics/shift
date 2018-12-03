#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <json/json.h>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

class Terrain;

/*!
@brief Shift Context datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , modelParameters_(name)
    , fixed_("fixed") {}
  ~Context() {}
  void readJSON(const std::string filename);
  void writeJSON();
  void writeJSON(const std::string filename);

private:
  ModelParameters modelParameters_;
  Assembly fixed_;
  std::vector<Terrain*> terrains_;
  std::string filename_;
  Json::Value root_;
};

} // namespace Shift

#endif
