#ifndef SHIFT_FORCEMODULE_TIRE_BASE_H_
#define SHIFT_FORCEMODULE_TIRE_BASE_H_

#include <memory>

#include "shift/core/force_module/base.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

class Base : public force_module::Base {
public:
  Base()
    : force_module::Base() {
    DeclareParameters();
  }
  Base(std::string name)
    : force_module::Base(name) {
    DeclareParameters();
  }
  virtual ~Base() override {}
  static std::unique_ptr<Base> create();
  virtual void DeclareParameters() override;

  static bool is_registered() {
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

} // namespace force_module
}
} // namespace shift

#endif
