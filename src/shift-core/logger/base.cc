#include "shift-core/logger/base.h"
#include "shift-core/context.h"
#include "shift-core/logger/csv.h"
#include "shift-core/logger/hdf5.h"

bool Shift::Logger::Modules::initialized() {
  if(!Shift::Logger::CSV::isRegistered()) {
    return false;
  }
  // if(!Shift::Logger::HDF5::isRegistered()) {
  //   return false;
  // }
  return true;
}

void Shift::Logger::Base::addParameters() {}

void Shift::Logger::Base::load() {}

void Shift::Logger::Base::connect() {}
