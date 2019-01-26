#include "shift-core/sensor/base.h"
#include "shift-core/context.h"

bool Shift::Sensor::Modules::initialized() {
  return true;
}

void Shift::Sensor::Base::load() {}

void Shift::Sensor::Base::connect() {}

void Shift::Sensor::Base::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
}
