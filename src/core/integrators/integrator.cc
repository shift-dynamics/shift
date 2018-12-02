#include "shift-core/integrators/integrator.h"

void Shift::Integrator::Factory::registerModule(const std::string& name,
                                                CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
