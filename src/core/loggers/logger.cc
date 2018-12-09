#include "shift-core/loggers/logger.h"

#include "shift-core/loggers/csv.h"
#include "shift-core/loggers/hdf5.h"

bool Shift::Logger::Modules::initialized() {
  if(!Shift::Logger::CSV::isRegistered()) {
    return false;
  }
  // if(!Shift::Logger::HDF5::isRegistered()) {
  //   return false;
  // }
  return true;
}

void Shift::Logger::LoggerBase::addKeys() {}
