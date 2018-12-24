#include "shift-core/sensors/sensor.h"
#include "shift-core/context.h"

bool Shift::Sensor::Modules::initialized() {
  return true;
}

void Shift::Sensor::SensorBase::load() {}

void Shift::Sensor::SensorBase::connect() {}

void Shift::Sensor::SensorBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    context_->setActive(this);
  }
}
