#include "shift-core/loggers/logger.h"
#include "shift-core/context.h"
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

void Shift::Logger::LoggerBase::load() {}

void Shift::Logger::LoggerBase::connect() {}

void Shift::Logger::LoggerBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    // context_->setActive(this);
  }
}
