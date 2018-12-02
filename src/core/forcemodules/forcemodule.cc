#include "shift-core/forcemodules/forcemodule.h"

void Shift::ForceModule::Factory::registerModule(const std::string& name,
                                                 CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
