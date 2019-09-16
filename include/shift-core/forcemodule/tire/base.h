#ifndef SHIFT_FORCEMODULE_TIRE_BASE_H_
#define SHIFT_FORCEMODULE_TIRE_BASE_H_

#include <memory>

#include "shift-core/forcemodule/base.h"

namespace Shift {

namespace ForceModule {

namespace Tire {

class Base : public ForceModule::Base {
public:
  Base()
    : ForceModule::Base() {
    addParameters();
  }
  Base(std::string name)
    : ForceModule::Base(name) {
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
  Shift::Body::Base* wheel_body_ptr; /*!< pointer to wheel body */
};

} // namespace Tire

} // namespace ForceModule

} // namespace Shift

#endif
