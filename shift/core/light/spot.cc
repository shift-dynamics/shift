#include "shift/core/light/spot.h"

#include <cmath>

#include "shift/core/exception.h"

bool shift::core::light::Spot::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "spot", shift::core::light::Spot::Create);

std::unique_ptr<shift::core::light::Spot> shift::core::light::Spot::Create() {
  return std::make_unique<shift::core::light::Spot>();
}

void shift::core::light::Spot::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
  DeclareParameter("cone_angle", &cone_angle_,
               "Half angle of the cone of light from the -z axis of the "
               "light's frame (radians, optional, default 30 degrees).",
               false);
  DeclareParameter("inner_cone_angle", &inner_cone_angle_,
               "Half angle inside which the light is at full brightness; it "
               "fades smoothly to nothing between this angle and cone_angle "
               "(radians, optional, default 0.8 cone_angle).",
               false);
}

void shift::core::light::Spot::Load() {
  const std::string prefix = "Light " + get_name() + ": ";
  if(!(cone_angle_ > 0.0) || !(cone_angle_ < M_PI)) {
    throw shift::core::Exception(prefix + "cone_angle must be between 0 "
                                 "and pi");
  }
  if(inner_cone_angle_ < 0.0) {
    inner_cone_angle_ = 0.8 * cone_angle_;
  }
  if(inner_cone_angle_ > cone_angle_) {
    throw shift::core::Exception(prefix + "inner_cone_angle must not be "
                                 "larger than cone_angle");
  }
}
