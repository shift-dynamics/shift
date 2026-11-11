#include "shift/core/terrain/base.h"

#include <algorithm>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/terrain/basic.h"
#include "shift/core/terrain/scm.h"
#include "shift/core/terrain/sph.h"

bool shift::core::terrain::Modules::is_initialized() {
  return shift::core::terrain::Basic::is_registered() &&
         shift::core::terrain::Scm::is_registered() &&
         shift::core::terrain::Sph::is_registered();
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
  DeclareParameter("filename", &filename_,
               "the elevation map, a single channel TIFF: a file next to the "
               "model or in an include directory; required by \"basic\", "
               "optional for \"scm\" (flat without it)",
               false);
  DeclareParameter("x", &x_range_,
                   "[first, last] x of the extent of the terrain in the "
                   "terrain frame (m): required by \"scm\", \"sph\" and "
                   "a region, the extent it covers",
                   false)
    .set_row_dim(2);
  DeclareParameter("y", &y_range_,
                   "[first, last] y of the extent of the terrain in the "
                   "terrain frame (m), as x",
                   false)
    .set_row_dim(2);
  DeclareParameter("regions", &region_array_,
                   "terrains inside this one, each over its extent x, y in "
                   "the frame of this terrain, e.g. an \"sph\" soil inside "
                   "an \"scm\" terrain; optional",
                   false);
}

void shift::core::terrain::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  const bool active = parent_is_active && active_;
  /* only the terrains of the fixed body are the context's; this one holds
  its regions */
  if(!parent_terrain_) {
    context_->ActivateObject(this, active);
  }
  regions_.clear();
  for(auto& region : region_array_.get_data()) {
    region->parent_terrain_ = this;
    region->ActivateObjects(active);
    if(active && region->is_active()) {
      regions_.push_back(region.get());
    }
  }
}

bool shift::core::terrain::Base::Contains(double x, double y) const {
  if(!has_extent()) {
    return true;
  }
  return x >= x_range_[0] && x <= x_range_[1] && y >= y_range_[0] &&
         y <= y_range_[1];
}

shift::core::terrain::Base* shift::core::terrain::Base::RegionAt(
  double x, double y) const {
  for(Base* region : regions_) {
    if(region->Contains(x, y)) {
      return region;
    }
  }
  return nullptr;
}

void shift::core::terrain::Base::CheckRegions() const {
  const std::string prefix = "Terrain " + get_name() + ": ";
  for(size_t a = 0; a < regions_.size(); a++) {
    const Base& region = *regions_[a];
    if(!region.has_extent() || !(region.x_range_[1] > region.x_range_[0]) ||
       !(region.y_range_[1] > region.y_range_[0])) {
      throw shift::core::Exception(prefix + "region " + region.get_name() +
                                   " needs x and y, each [first, last] with "
                                   "last > first");
    }
    if(has_extent() &&
       (region.x_range_[0] < x_range_[0] || region.x_range_[1] > x_range_[1] ||
        region.y_range_[0] < y_range_[0] || region.y_range_[1] > y_range_[1])) {
      throw shift::core::Exception(prefix + "region " + region.get_name() +
                                   " is not inside the terrain's x and y");
    }
    for(size_t b = 0; b < a; b++) {
      const Base& other = *regions_[b];
      if(region.x_range_[0] < other.x_range_[1] &&
         other.x_range_[0] < region.x_range_[1] &&
         region.y_range_[0] < other.y_range_[1] &&
         other.y_range_[0] < region.y_range_[1]) {
        throw shift::core::Exception(prefix + "regions " + other.get_name() +
                                     " and " + region.get_name() +
                                     " overlap");
      }
    }
  }
}

void shift::core::terrain::Base::LoadAll() {
  Load();
  CheckRegions();
  for(Base* region : regions_) {
    region->LoadAll();
  }
}

void shift::core::terrain::Base::ConnectAll() {
  Connect();
  for(Base* region : regions_) {
    region->ConnectAll();
  }
}

void shift::core::terrain::Base::AcceptStepAll() {
  AcceptStep();
  for(Base* region : regions_) {
    region->AcceptStepAll();
  }
}

void shift::core::terrain::Base::UpdateOutputAll() {
  UpdateOutput();
  for(Base* region : regions_) {
    region->UpdateOutputAll();
  }
}
