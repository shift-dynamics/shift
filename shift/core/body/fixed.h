#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift/core/body/base.h"
#include "shift/core/factory.h"

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
    DeclareParameters();
  }
  Fixed(std::string name)
    : Base(name) {
    name_ = "fixed";
    DeclareParameters();
  }
  virtual ~Fixed() override {}

  static bool is_registered() {
    return registered_;
  }

  static std::unique_ptr<Fixed> Create();

  virtual void DeclareParameters() override;

  std::vector<std::unique_ptr<terrain::Base>>& get_terrain() {
    return terrain_.get_data();
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
