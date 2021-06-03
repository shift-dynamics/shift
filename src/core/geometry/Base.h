#ifndef SHIFT_GEOMETRY_BASE_H_
#define SHIFT_GEOMETRY_BASE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

class Frame;
class Context;

namespace geometry {

/*!
@brief Abstract Geometry base class
*/
class Base
  : public shift::core::Base
  , public shift::core::parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {}
  Base()
    : shift::core::Base("") {}
  virtual ~Base() {}

  static const std::string get_array_key() {
    return "geometry";
  }

  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  virtual void ActivateObjects(bool) override;
  
  virtual void Load();
  virtual void Connect();

protected:
  virtual void DeclareParameters() override;
  std::string type_; //!< descriptive name of Geometry type
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace geometry
}
} // namespace shift

#endif
