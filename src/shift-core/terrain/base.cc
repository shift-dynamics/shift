#include "shift-core/terrain/base.h"
#include "shift-core/context.h"
#include "shift-core/terrain/basic.h"

bool Shift::Terrain::Modules::initialized() {
  if(!Shift::Terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void Shift::Terrain::Base::addParameters() {
  addParameter(
    "name", &name_,
    "terrain descriptive name, each terrain must have a unique name identified",
    true);
  addParameter("parent", &parent_,
               "terrain's parent body / frame, defaults to [[\"fixed\"], "
               "[\"origin\"]] if not specified",
               false)
    .setRowDim(2)
    .setColumnDim(-1);
  addParameter("filename", &filename_, "path to filename that defines terrain",
               true);
}

void Shift::Terrain::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
