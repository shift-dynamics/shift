#include "shift-core/terrain/basic.h"

class Shift::Terrain::Registration registration("basic",
                                                Shift::Terrain::Basic::create);

void Shift::Terrain::Basic::addKeys() {
  Shift::Terrain::TerrainBase::addKeys();
}
