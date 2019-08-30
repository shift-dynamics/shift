#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/parameter.h"

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
  , public Shift::Parameter::List {

public:
  Base(std::string name)
    : Shift::Base(name) {
    addParameters();
  }
  Base()
    : Shift::Base("") {
    addParameters();
  }
  virtual ~Base() {}

  const std::string& getType() {
    return type_;
  }

  virtual void addParameters() override;

  void setContext(Context* context) {
    context_ = context;
  }

  static const std::string arrayKey() {
    return "terrain";
  }

  static const std::string defaultType() {
    return "";
  }

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
