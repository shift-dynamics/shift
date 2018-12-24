#ifndef _SHIFT_TERRAIN_H_
#define _SHIFT_TERRAIN_H_

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
class TerrainBase
  : public Base
  , public Serialize {

public:
  TerrainBase(std::string name)
    : Base(name) {
    addKeys();
  }
  TerrainBase()
    : Base("") {
    addKeys();
  }
  ~TerrainBase() {}

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
