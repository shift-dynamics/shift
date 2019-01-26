#include "shift-core/terrain/base.h"
#include "shift-core/context.h"
#include "shift-core/terrain/basic.h"

bool Shift::Terrain::Modules::initialized() {
  if(!Shift::Terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Terrain::Base::addKeys() {}

void Shift::Terrain::Base::readJSON(Json::Value& value) {
  Shift::Serialize::readJSON(value);
}
