#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift/core/body/Base.h"
#include "shift/core/Factory.h"

#include <memory>

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace terrain {
class Base;
}

namespace body {

class Fixed : public Base {
public:
  Fixed()
    : Base() {
    name_ = "fixed";
    addParameters();
  }
  Fixed(std::string name)
    : Base(name) {
    name_ = "fixed";
    addParameters();
  }
  virtual ~Fixed() override {}
  static std::unique_ptr<Fixed> create();
  virtual void addParameters() override;
  static bool isRegistered() {
    return registered_;
  }

private:
  /*! \brief User defined frames attached to fixed inertial reference frame */
  shift::core::parameter::Array<shift::core::Frame> frames_;

  /*! \brief User defined lights attached to fixed inertial reference frame */
  shift::core::parameter::FactoryArray<shift::core::light::Base> lights_;

  /*! \brief User defined terrain attached to fixed inertial reference frame */
  shift::core::parameter::FactoryArray<shift::core::terrain::Base> terrain_;

  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
