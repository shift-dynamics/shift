#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift-core/body/base.h"
#include "shift-core/factory.h"

#include <memory>

namespace Shift {

namespace Joint {
class Base;
}

namespace Terrain {
class Base;
}

namespace Body {

class Fixed : public Base {
public:
  Fixed() {
    name_ = "fixed";
    addParameters();
  }
  Fixed(std::string) {
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
  Shift::Parameter::Array<Shift::Frame> frames_;

  /*! \brief User defined lights attached to fixed inertial reference frame */
  Shift::Parameter::FactoryArray<Shift::Light::Base> lights_;

  /*! \brief User defined terrain attached to fixed inertial reference frame */
  Shift::Parameter::FactoryArray<Shift::Terrain::Base> terrain_;

  static bool registered_;
};

} // namespace Body

} // namespace Shift

#endif
