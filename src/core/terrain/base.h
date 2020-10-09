#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {

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
  : public shift::Base
  , public shift::Parameter::List {

public:
  Base(std::string name)
    : shift::Base(name) {
    addParameters();
  }
  Base()
    : shift::Base("") {
    addParameters();
  }
  virtual ~Base() override {}

  const std::string& getType() {
    return type_;
  }

  static const std::string arrayKey() {
    return "terrain";
  }

  static const std::string defaultType() {
    return "";
  }

  virtual void load();
  virtual void connect();
  virtual void setActiveObjects(bool parent_is_active) override;

protected:
  virtual void addParameters() override;

  std::string type_; //!< descriptive name of TerrainBase type
  std::string filename_;
  std::vector<std::vector<std::string>> parent_;
};

class Modules {
public:
  static bool initialized();
};

} // namespace Terrain

} // namespace shift

#endif
