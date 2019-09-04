#include "shift-core/terrain/base.h"
#include "shift-core/context.h"
#include "shift-core/terrain/basic.h"

bool Shift::Terrain::Modules::initialized() {
  if(!Shift::Terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Terrain::Base::addParameters() {}

void Shift::Terrain::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
