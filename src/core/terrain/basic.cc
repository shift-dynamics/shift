#include "shift-core/terrain/basic.h"
#include "shift-core/factory.h"

bool Shift::Terrain::Basic::registered_ =
  Shift::Factory<Shift::Terrain::TerrainBase>::get()->registerModule(
    "basic", Shift::Terrain::Basic::create);

std::unique_ptr<Shift::Terrain::Basic> Shift::Terrain::Basic::create() {
  return std::make_unique<Shift::Terrain::Basic>();
}

void Shift::Terrain::Basic::addKeys() {
  Shift::Terrain::TerrainBase::addKeys();
}

void Shift::Terrain::TerrainBase::load() {}

void Shift::Terrain::TerrainBase::connect() {}
