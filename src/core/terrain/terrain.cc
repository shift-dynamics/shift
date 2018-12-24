#include "shift-core/terrain/terrain.h"
#include "shift-core/context.h"
#include "shift-core/terrain/basic.h"

bool Shift::Terrain::Modules::initialized() {
  if(!Shift::Terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Terrain::TerrainBase::addKeys() {}

void Shift::Terrain::TerrainBase::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
  if(context_) {
    context_->setActive(this);
  }
}
