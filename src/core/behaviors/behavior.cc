#include "shift-core/behaviors/behavior.h"

void Shift::Behavior::Factory::registerModule(const std::string& name,
                                              CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
