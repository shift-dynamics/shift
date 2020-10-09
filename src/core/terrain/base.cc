#include "shift/core/terrain/base.h"
#include "shift/core/context.h"
#include "shift/core/terrain/basic.h"

bool shift::Terrain::Modules::initialized() {
  if(!shift::Terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void shift::Terrain::Base::addParameters() {
  addParameter(
    "name", &name_,
    "terrain descriptive name, each terrain must have a unique name identifier",
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

void shift::Terrain::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
