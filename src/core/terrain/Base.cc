#include "shift/core/terrain/Base.h"
#include "shift/core/Context.h"
#include "shift/core/terrain/Basic.h"

bool shift::core::terrain::Modules::is_initialized() {
  return shift::core::terrain::Basic::is_registered();
}

void shift::core::terrain::Base::DeclareParameters() {
  DeclareParameter(
    "name", &name_,
    "terrain descriptive name, each terrain must have a unique name identifier",
    true);
  DeclareParameter("parent", &parent_,
               "terrain's parent body / frame, defaults to [[\"fixed\"], "
               "[\"origin\"]] if not specified",
               false)
    .set_row_dim(2)
    .set_column_dim(-1);
  DeclareParameter("filename", &filename_, "path to filename that defines terrain",
               true);
}

void shift::core::terrain::Base::set_objects_active(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
