#include "shift-core/logger/csv.h"
#include "shift-core/factory.h"
#include "shift-core/logger/base.h"

bool Shift::Logger::CSV::registered_ =
  Shift::Factory<Shift::Logger::Base>::get()->registerModule(
    "csv", Shift::Logger::CSV::create);

std::unique_ptr<Shift::Logger::CSV> Shift::Logger::CSV::create() {
  return std::make_unique<Shift::Logger::CSV>();
}

void Shift::Logger::CSV::addParameters() {
  Shift::Logger::Base::addParameters();
}
