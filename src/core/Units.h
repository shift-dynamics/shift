#ifndef SHIFT_UNITS_H_
#define SHIFT_UNITS_H_

#include <string>

#include "shift/core/Base.h"
#include "shift/core/parameter/List.h"

namespace shift {
namespace core {

/*!
\brief Shift Assembly datastructure
*/
class Units
  : public Base
  , public shift::core::parameter::List {

public:
  Units(std::string name)
    : Base(name)
    , parameter::List() {
    DeclareParameters();
  }
  Units()
    : Base("")
    , parameter::List() {
    DeclareParameters();
  }
  ~Units() {}

  /*! \brief retrieve angle units */
  const std::string& get_units_angle() {
    return angle_;	  
  }

  /*! \brief retrieve length units */
  const std::string& get_units_length() {
    return length_;
  }

  /*! \brief retrieve mass units */
  const std::string& get_units_mass() {
    return mass_;
  }

  /*! \brief retrieve time units */
  const std::string& get_units_time() {
    return time_;
  }

  /*! \brief set angle units */
  void set_units_angle(const std::string angle) {
    angle_ = angle;
  }

  /*! \brief set length units */
  void set_units_length(const std::string length) {
    length_ = length;
  }

  /*! \brief set mass units */
  void set_units_mass(const std::string mass) {
    mass_ = mass;
  }

  /*! \brief set time units */
  void set_units_time(const std::string time) {
    time_ = time;
  }

private:
  virtual void DeclareParameters() override;

  std::string angle_ = "radian";
  std::string length_ = "meter";
  std::string mass_ = "kilogram";
  std::string time_ = "second";
};
}
} // namespace shift

#endif
