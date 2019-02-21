#ifndef SHIFT_FORCEMODULE_BASE_H_
#define SHIFT_FORCEMODULE_BASE_H_

#include <functional>
#include <map>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Context;

namespace ForceModule {

/*!
@brief Shift::Frame abstract base class
*/
class Base
  : public Shift::Base
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name)
    , Serialize() {
    addKeys();
  }
  Base()
    : Shift::Base("")
    , Serialize() {
    addKeys();
  }
  virtual ~Base() {}

  void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();
  void setContext(Context* context) {
    context_ = context;
  }

protected:
  Context* context_ = NULL;
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
