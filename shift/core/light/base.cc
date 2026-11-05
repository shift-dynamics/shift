#include "shift/core/light/base.h"
#include "shift/core/context.h"
#include "shift/core/light/directional.h"
#include "shift/core/light/ies.h"
#include "shift/core/light/point.h"
#include "shift/core/light/spot.h"

#include "ksl/print.h"

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
}

void shift::core::light::Base::Load() {}

void shift::core::light::Base::Connect() {}

void shift::core::light::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
