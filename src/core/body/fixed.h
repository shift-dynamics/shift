#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift/core/body/base.h"
#include "shift/core/factory.h"

#include <memory>

namespace shift {

namespace Joint {
class Base;
}

namespace Terrain {
class Base;
}

namespace Body {

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
  shift::Parameter::Array<shift::Frame> frames_;

  /*! \brief User defined lights attached to fixed inertial reference frame */
  shift::Parameter::FactoryArray<shift::Light::Base> lights_;

  /*! \brief User defined terrain attached to fixed inertial reference frame */
  shift::Parameter::FactoryArray<shift::Terrain::Base> terrain_;

  static bool registered_;
};

} // namespace Body

} // namespace shift

#endif
