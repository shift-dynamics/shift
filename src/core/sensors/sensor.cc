#include "shift-core/sensors/sensor.h"

void Shift::Sensor::Factory::registerModule(const std::string& name,
                                            CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
