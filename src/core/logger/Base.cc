#include "shift/core/logger/Base.h"
#include "shift/core/Context.h"
#include "shift/core/logger/CSV.h"
#include "shift/core/logger/HDF5.h"

bool shift::core::logger::Modules::is_initialized() {
  if(!shift::core::logger::CSV::is_registered()) {
    return false;
  }
  if(!shift::core::logger::HDF5::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::logger::Base::DeclareParameters() {
  DeclareParameter("log_all_channels", &log_all_channels_,
               "indicates to log all generalized coordinates, defaults to true "
               "if not specified.",
               false);
}

void shift::core::logger::Base::Load() {}

void shift::core::logger::Base::Connect() {}

void shift::core::logger::Base::set_objects_active(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
