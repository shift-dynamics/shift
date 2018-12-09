#ifndef _SHIFT_FORCEMODULE_H_
#define _SHIFT_FORCEMODULE_H_

#include <functional>
#include <map>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace ForceModule {

/*!
@brief Shift::Frame abstract base class
*/
class ForceModuleBase
  : public Base
  , public Serialize {

public:
  ForceModuleBase(std::string name)
    : Base(name)
    , Serialize() {
    addKeys();
  }
  ForceModuleBase()
    : Base("")
    , Serialize() {
    addKeys();
  }
  ~ForceModuleBase() {}

  void readJSON(Json::Value);

protected:
  void addKeys();

  std::string type_;
};

class Modules {
public:
  static bool initialized();
};

} // namespace ForceModule

} // namespace Shift

#endif
