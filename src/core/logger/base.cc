#include "shift/core/logger/base.h"
#include "shift/core/context.h"
#include "shift/core/logger/csv.h"
#include "shift/core/logger/hdf5.h"

bool shift::Logger::Modules::initialized() {
  if(!shift::Logger::CSV::isRegistered()) {
    return false;
  }
  // if(!shift::Logger::HDF5::isRegistered()) {
  //   return false;
  // }
  return true;
}

void shift::Logger::Base::addParameters() {
  addParameter("log_all_channels", &log_all_channels_,
               "indicates to log all generalized coordinates, defaults to true "
               "if not specified.",
               false);
}

void shift::Logger::Base::load() {}

void shift::Logger::Base::connect() {}

void shift::Logger::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
