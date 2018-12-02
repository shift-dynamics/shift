#include "shift-core/terrain/terrain.h"

void Shift::Terrain::Factory::registerModule(const std::string& name,
                                             CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
