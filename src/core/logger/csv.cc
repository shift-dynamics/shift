#include "shift-core/logger/csv.h"
#include "shift-core/factory.h"
#include "shift-core/logger/base.h"

bool Shift::Logger::CSV::registered_ =
  Shift::Factory<Shift::Logger::Base>::get()->registerModule(
    "csv", Shift::Logger::CSV::create);

std::unique_ptr<Shift::Logger::CSV> Shift::Logger::CSV::create() {
  return std::make_unique<Shift::Logger::CSV>();
}

void Shift::Logger::CSV::addKeys() {
  Shift::Logger::Base::addKeys();
}

void Shift::Logger::CSV::readJSON(Json::Value& value) {
  Shift::Logger::Base::readJSON(value);
}
