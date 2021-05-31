#ifndef SHIFT_LIGHT_BASE_H_
#define SHIFT_LIGHT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

namespace body {
class BodyBase;
}

class Frame;
class Context;

namespace light {

/*!
@brief Abstract shift::Light base class
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {}
  Base()
    : shift::core::Base("") {}
  virtual ~Base() override {}

  static const std::string get_array_key() {
    return "lights";
  }
  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() {
    return type_;
  }

  void set_objects_active(bool parent_is_active) override;
  
  virtual void Load();
  virtual void Connect();

protected:
  virtual void DeclareParameters() override;
  std::vector<std::string> parent_;
  std::string type_; //!< descriptive name of Light type
  ksl_vec3_t Ka_ = ksl_vec3(0.2, 0.2, 0.2);
  ksl_vec3_t Kd_ = ksl_vec3(1.0, 1.0, 1.0);
  ksl_vec3_t Ks_ = ksl_vec3(1.0, 1.0, 1.0);
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace light
}
} // namespace shift

#endif
