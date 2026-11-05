#ifndef SHIFT_LIGHT_SPOT_H_
#define SHIFT_LIGHT_SPOT_H_

#include "shift/core/factory.h"
#include "shift/core/light/base.h"

namespace shift {
namespace core {
namespace light {

/*!
@brief A light shining in a cone around the -z axis of its frame

The light is full inside inner_cone_angle and fades smoothly to nothing at
cone_angle, both half angles from the axis (radians).
*/
class Spot : public Base {
public:
  Spot()
    : Base() {
    DeclareParameters();
  }
  Spot(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~Spot() override {}
  static std::unique_ptr<Spot> Create();
  static bool is_registered() {
    return registered_;
  }

  void Load() override;

  /*! \brief half angle of the cone (radians) */
  double get_cone_angle() const {
    return cone_angle_;
  }
  /*! \brief half angle of the full brightness part of the cone (radians) */
  double get_inner_cone_angle() const {
    return inner_cone_angle_;
  }

private:
  virtual void DeclareParameters() override;
  double cone_angle_ = 0.5235987755982988; // 30 degrees
  double inner_cone_angle_ = -1.0;         // set by Load if not given
  static bool registered_;
};

} // namespace light
}
} // namespace shift

#endif
