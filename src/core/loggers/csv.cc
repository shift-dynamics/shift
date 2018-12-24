#include "shift-core/loggers/csv.h"
#include "shift-core/factory.h"
#include "shift-core/loggers/logger.h"

bool Shift::Logger::CSV::registered_ =
  Shift::Factory<Shift::Logger::LoggerBase>::get()->registerModule(
    "csv", Shift::Logger::CSV::create);

std::unique_ptr<Shift::Logger::CSV> Shift::Logger::CSV::create() {
  return std::make_unique<Shift::Logger::CSV>();
}

void Shift::Logger::CSV::addKeys() {
  Shift::Logger::LoggerBase::addKeys();
}

void Shift::Logger::CSV::readJSON(Json::Value& value) {
  Shift::Logger::LoggerBase::readJSON(value);
}
