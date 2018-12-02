#include "shift-core/lights/light.h"

void Shift::Light::Factory::registerModule(const std::string& name,
                                           CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
