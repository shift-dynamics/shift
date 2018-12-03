#include "shift-core/terrain/basic.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Terrain::TerrainBase>
  registration("basic", Shift::Terrain::Basic::create);

Shift::Terrain::Basic* Shift::Terrain::Basic::create() {
  return new Shift::Terrain::Basic();
}

void Shift::Terrain::Basic::addKeys() {
  Shift::Terrain::TerrainBase::addKeys();
}
