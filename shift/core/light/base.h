#ifndef SHIFT_LIGHT_BASE_H_
#define SHIFT_LIGHT_BASE_H_

#include <functional>
#include <map>
#include <string>

#include "shift/core/base.h"
#include "shift/core/parameter/list.h"

namespace shift {
namespace core {

namespace body {
class BodyBase;
}

class Frame;
class Context;

namespace light {

/*!
@brief Abstract shift::Light base class

A light is attached to a frame of a body (parent: ["body", "frame"]) and
moves with it. Spot, directional and IES lights shine along the -z axis of
their frame (the frame's z axis points back towards the light, as the view
direction of a camera).

Colors, as RGB from 0 to 1:
- Kd: diffuse color. A white, matte surface facing the light at unit
  attenuation shows this color.
- Ks: color of the specular highlights.
- Ka: ambient color, added to the light from the sky on every surface.

intensity multiplies Kd and Ks. Point, spot and IES lights fall off with
the distance d to the light as 1 / (c + l d + q d^2), with attenuation
[c, l, q]; the default [1, 0, 0] does not fall off.

The viewer draws the lights of a model in place of its default sun.
*/
class Base
  : public shift::core::Base
  , public parameter::List {

public:
  Base(std::string name)
    : shift::core::Base(name) {}
  Base()
    : shift::core::Base("") {}
  virtual ~Base() override {}

  static const std::string get_array_key() {
    return "lights";
  }
  static const std::string get_default_type() {
    return "";
  }

  const std::string& get_type() const {
    return type_;
  }

  void ActivateObjects(bool parent_is_active) override;
  virtual void Load();
  /*! \brief finds the parent body and frame; a light whose frame does not
   * exist is left without one (get_frame() is null) and not drawn */
  virtual void Connect();

  /*! \brief the parent body and frame, set by Connect; null if not found */
  const body::Base* get_parent_body() const {
    return parent_body_;
  }
  const Frame* get_frame() const {
    return frame_;
  }

  const ksl_vec3_t& get_Ka() const {
    return Ka_;
  }
  const ksl_vec3_t& get_Kd() const {
    return Kd_;
  }
  const ksl_vec3_t& get_Ks() const {
    return Ks_;
  }
  double get_intensity() const {
    return intensity_;
  }
  /*! \brief [c, l, q]: the light falls off as 1 / (c + l d + q d^2) */
  const ksl_vec3_t& get_attenuation() const {
    return attenuation_;
  }

protected:
  virtual void DeclareParameters() override;
  std::vector<std::string> parent_;
  std::string type_; //!< descriptive name of Light type
  ksl_vec3_t Ka_ = ksl_vec3(0.2, 0.2, 0.2);
  ksl_vec3_t Kd_ = ksl_vec3(1.0, 1.0, 1.0);
  ksl_vec3_t Ks_ = ksl_vec3(1.0, 1.0, 1.0);
  double intensity_ = 1.0;
  ksl_vec3_t attenuation_ = ksl_vec3(1.0, 0.0, 0.0);

  const body::Base* parent_body_ = nullptr;
  const Frame* frame_ = nullptr;
};

class Modules {
public:
  static bool is_initialized();
};

} // namespace light
}
} // namespace shift

#endif
