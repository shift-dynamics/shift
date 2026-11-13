#include "shift/core/light/base.h"
#include "shift/core/context.h"
#include "shift/core/light/directional.h"
#include "shift/core/light/ies.h"
#include "shift/core/light/point.h"
#include "shift/core/light/spot.h"

#include "shift/core/body/base.h"
#include "shift/core/frame.h"
#include "shift/core/geometry/base.h"
#include "shift/core/log.h"

bool shift::core::light::Modules::is_initialized() {
  if(!shift::core::light::Point::is_registered()) {
    return false;
  }
  if(!shift::core::light::Spot::is_registered()) {
    return false;
  }
  if(!shift::core::light::Directional::is_registered()) {
    return false;
  }
  if(!shift::core::light::IES::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::light::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "User defined descriptive light name.", true);
  DeclareParameter(
    "active", &active_,
    "Indicates whether light is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
  DeclareParameter("type", &type_, "Indicates type of light", true);
  DeclareParameter(
    "parent", &parent_,
    "Indicates light's parent body and frame using the syntax [\"Body\", "
    "\"Frame\"]",
    true);
  DeclareParameter("Ka", &Ka_,
               "Indicates light's ambient color component in RGB format "
               "using values from 0 to 1.",
               false);
  DeclareParameter("Kd", &Kd_,
               "Indicates light's diffuse color component in RGB format "
               "using values from 0 to 1.",
               false);
  DeclareParameter("Ks", &Ks_,
               "Indicates light's specular color component in RGB format "
               "using values from 0 to 1.",
               false);
  DeclareInput("intensity", &intensity_,
               "Factor of the diffuse and specular colors (optional, "
               "default 1). An input: a controller, such as a map from a "
               "brake pedal, can change it while the model runs, to switch "
               "a lamp on and off.");
  DeclareParameter("glow", &glow_,
               "names of geometries of the parent body that glow with the "
               "light, as the lens of a lamp: the viewer adds to their "
               "shading their own color times Kd and intensity, so that "
               "they light up as the light is switched on (optional)",
               false)
    .set_row_dim(-1);
  DeclareParameter("attenuation", &attenuation_,
               "[c, l, q]: point, spot and IES lights fall off with the "
               "distance d to the light as 1 / (c + l d + q d^2). Use "
               "[0, 0, 1] for the inverse square law of a physical light "
               "(optional, default [1, 0, 0]: no fall off).",
               false);
}

void shift::core::light::Base::Load() {}

void shift::core::light::Base::Connect() {
  parent_body_ = nullptr;
  frame_ = nullptr;
  const std::string prefix = "Light " + get_name() + ": ";
  if(parent_.size() != 2) {
    SHIFT_WARNING(prefix + "parent must be [\"body\", \"frame\"]; the "
                  "light is not drawn");
    return;
  }
  body::Base* body =
    context_->FindActiveBody(parent_[0], get_parent_assembly());
  if(!body) {
    SHIFT_WARNING(prefix + "body " + parent_[0] + " not found among the "
                  "active bodies; the light is not drawn");
    return;
  }
  for(auto& frame : body->get_frames()) {
    if(frame->get_name() == parent_[1] ||
       (parent_[1] == "origin" && frame->is_origin())) {
      frame_ = frame.get();
    }
  }
  if(!frame_) {
    SHIFT_WARNING(prefix + "frame " + parent_[1] + " not found on body " +
                  parent_[0] + "; the light is not drawn");
    return;
  }
  for(const auto& name : glow_) {
    bool found = false;
    for(const auto& geometry : body->get_geometry()) {
      found = found || geometry->get_name() == name;
    }
    if(!found) {
      SHIFT_WARNING(prefix + "glow: no geometry named " + name +
                    " on body " + parent_[0]);
    }
  }
  parent_body_ = body;
}

void shift::core::light::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
