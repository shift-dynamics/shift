#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;
class Context;

namespace Terrain {

/*!
@brief Abstract TerrainBase base class
*/
class Base
  : public Shift::Base
  , public Serialize {

public:
  Base(std::string name)
    : Shift::Base(name) {
    addKeys();
  }
  Base()
    : Shift::Base("") {
    addKeys();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  void setContext(Context* context) {
    context_ = context;
  }

  virtual void readJSON(Json::Value&);
  virtual void load();
  virtual void connect();

protected:
  Context* context_;
  void addKeys();

  std::string type_; //!< descriptive name of TerrainBase type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Terrain

} // namespace Shift

#endif
