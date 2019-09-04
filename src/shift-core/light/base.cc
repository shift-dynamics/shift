#include "shift-core/light/base.h"
#include "shift-core/context.h"
#include "shift-core/light/directional.h"
#include "shift-core/light/ies.h"
#include "shift-core/light/point.h"
#include "shift-core/light/spot.h"

#include "ksl/print.h"

bool Shift::Light::Modules::initialized() {
  if(!Shift::Light::Point::isRegistered()) {
    return false;
  }
  if(!Shift::Light::Spot::isRegistered()) {
    return false;
  }
  if(!Shift::Light::Directional::isRegistered()) {
    return false;
  }
  if(!Shift::Light::IES::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Light::Base::addParameters() {
  addParameter("name", &name_, "User defined descriptive light name.", true);
  addParameter(
    "active", &active_,
    "Indicates whether light is active in simulation: If the value is set to "
    "false, the data will be loaded into computer working memory, but will not "
    "be processed when setting up and solving the equations of motion. Use "
    "this key to selectively enable or disable portions of the model.",
    false);
  addParameter("type", &type_, "Indicates type of light", true);
  addParameter(
    "parent", &parent_,
    "Indicates light's parent body and frame using the syntax [\"Body\", "
    "\"Frame\"]",
    true);
  addParameter("Ka", &Ka_,
               "Indicates light's ambient color component in RGB format "
               "using values from 0 to 1.",
               false);
  addParameter("Kd", &Kd_,
               "Indicates light's diffuse color component in RGB format "
               "using values from 0 to 1.",
               false);
  addParameter("Ks", &Ks_,
               "Indicates light's specular color component in RGB format "
               "using values from 0 to 1.",
               false);
}

void Shift::Light::Base::load() {}

void Shift::Light::Base::connect() {}

void Shift::Light::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
