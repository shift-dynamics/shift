#ifndef SHIFT_TERRAIN_BASE_H_
#define SHIFT_TERRAIN_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace body {
class Base;
}

class Frame;
class Context;

namespace terrain {

/*!
@brief Abstract TerrainBase base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {
    DeclareParameters();
  }
  Base()
    : shift::core::Base("") {
    DeclareParameters();
  }
  virtual ~Base() override {}

  const std::string& get_type() {
    return type_;
  }

  static const std::string get_array_key() {
    return "terrain";
  }

  static const std::string get_default_type() {
    return "";
  }

  virtual void ActivateObjects(bool parent_is_active) override;
  
  virtual void Load();
  virtual void Connect();

protected:
  virtual void DeclareParameters() override;

  std::string type_; //!< descriptive name of TerrainBase type
  std::string filename_;
  std::vector<std::vector<std::string>> parent_;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace terrain
}
} // namespace shift

#endif
