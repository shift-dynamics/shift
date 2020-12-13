#include "shift/core/light/Base.h"
#include "shift/core/Context.h"
#include "shift/core/light/Directional.h"
#include "shift/core/light/IES.h"
#include "shift/core/light/Point.h"
#include "shift/core/light/Spot.h"

#include "ksl/print.h"

bool shift::core::light::Modules::initialized() {
  if(!shift::core::light::Point::isRegistered()) {
    return false;
  }
  if(!shift::core::light::Spot::isRegistered()) {
    return false;
  }
  if(!shift::core::light::Directional::isRegistered()) {
    return false;
  }
  if(!shift::core::light::IES::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::light::Base::addParameters() {
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

void shift::core::light::Base::load() {}

void shift::core::light::Base::connect() {}

void shift::core::light::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
