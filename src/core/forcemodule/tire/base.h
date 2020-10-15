#ifndef SHIFT_FORCEMODULE_TIRE_BASE_H_
#define SHIFT_FORCEMODULE_TIRE_BASE_H_

#include <memory>

#include "shift/core/forcemodule/base.h"

namespace shift {
namespace core {
namespace forcemodule {
namespace tire {

class Base : public forcemodule::Base {
public:
  Base()
    : forcemodule::Base() {
    addParameters();
  }
  Base(std::string name)
    : forcemodule::Base(name) {
    addParameters();
  }
  virtual ~Base() override {}
  static std::unique_ptr<Base> create();
  virtual void addParameters() override;

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
  std::string filename_;
  double r0_ = 0.25;                    /*!< unloaded radius */
  std::vector<std::string> wheel_body_; /*!< fully qualified wheel name */

  /*! internal quantities for updating model*/
  shift::core::body::Base* wheel_body_ptr; /*!< pointer to wheel body */
};

} // namespace Tire

} // namespace forcemodule
}
} // namespace shift

#endif
