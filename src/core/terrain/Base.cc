#include "shift/core/terrain/Base.h"
#include "shift/core/Context.h"
#include "shift/core/terrain/Basic.h"

bool shift::core::terrain::Modules::initialized() {
  if(!shift::core::terrain::Basic::isRegistered()) {
    return false;
  }
  return true;
}

void shift::core::terrain::Base::addParameters() {
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

void shift::core::terrain::Base::setActiveObjects(bool parent_is_active) {
  assert(context_);
  context_->setActive(this, parent_is_active && active_);
}
