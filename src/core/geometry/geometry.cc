#include "shift-core/geometry/geometry.h"

void Shift::Geometry::Factory::registerModule(const std::string& name,
                                              CreateFunction pfnCreate) {
  factoryMap_.insert(std::make_pair(name, pfnCreate));
}
