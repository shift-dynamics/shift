#include "shift/core/sensor/base.h"
#include "shift/core/context.h"
#include "shift/core/sensor/body_load.h"
#include "shift/core/sensor/frame.h"
#include "shift/core/sensor/joint.h"
#include "shift/core/sensor/marker.h"

/* referencing each module's registration keeps its object file, and so its
registration, in the static library */
bool shift::core::sensor::Modules::is_initialized() {
  return shift::core::sensor::BodyLoad::is_registered() &&
         shift::core::sensor::Frame::is_registered() &&
         shift::core::sensor::Joint::is_registered() &&
         shift::core::sensor::Marker::is_registered();
}

void shift::core::sensor::Base::Load() {}

void shift::core::sensor::Base::Connect() {}

void shift::core::sensor::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "User defined descriptive sensor name",
                   true);
  DeclareParameter("type", &type_, "sensor type", true);
  DeclareParameter("active", &active_,
                   "false loads the sensor without evaluating it", false);
}

void shift::core::sensor::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
