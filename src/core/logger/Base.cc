#include "shift/core/logger/Base.h"
#include "shift/core/Context.h"
#include "shift/core/logger/CSV.h"
#include "shift/core/logger/HDF5.h"

bool shift::core::logger::Modules::initialized() {
  if(!shift::core::logger::CSV::isRegistered()) {
    return false;
  }
  if(!shift::core::logger::HDF5::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::logger::Base::addParameters() {
  addParameter("log_all_channels", &log_all_channels_,
               "indicates to log all generalized coordinates, defaults to true "
               "if not specified.",
               false);
}

void shift::core::logger::Base::load() {}

void shift::core::logger::Base::connect() {}

void shift::core::logger::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
