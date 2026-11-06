#include "shift/core/force_module/base.h"
#include "shift/core/context.h"
#include "shift/core/force_module/beam.h"
#include "shift/core/force_module/belt.h"
#include "shift/core/force_module/bushing.h"
#include "shift/core/force_module/contact.h"
#include "shift/core/force_module/hydraulic_cylinder.h"
#include "shift/core/force_module/rubber_mount.h"
#include "shift/core/force_module/tire/base.h"
#include "shift/core/force_module/gear_mesh.h"
#include "shift/core/force_module/journal_bearing.h"
#include "shift/core/force_module/applied_load.h"
#include "shift/core/force_module/joint_force.h"
#include "shift/core/force_module/lead_screw.h"
#include "shift/core/force_module/powertrain.h"
#include "shift/core/force_module/soil_tool.h"
#include "shift/core/force_module/spline.h"
#include "shift/core/force_module/spring_damper.h"
#include "shift/core/force_module/thrust_collar.h"
#include "shift/core/force_module/terrain_contact.h"
#include "shift/core/force_module/track.h"
#include "shift/core/force_module/track_soil.h"

bool shift::core::force_module::Modules::is_initialized() {
  if(!shift::core::force_module::Contact::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::tire::Base::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Beam::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Belt::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Bushing::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::HydraulicCylinder::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::RubberMount::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::GearMesh::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::JournalBearing::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::LeadScrew::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Powertrain::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::JointFriction::is_registered() ||
     !shift::core::force_module::JointMap::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::AppliedLoad::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::TerrainContact::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::SoilTool::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Spline::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::SpringDamper::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::ThrustCollar::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::Track::is_registered()) {
    return false;
  }
  if(!shift::core::force_module::TrackSoil::is_registered()) {
    return false;
  }
  return true;
}
void shift::core::force_module::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}

void shift::core::force_module::Base::DeclareParameters() {
  DeclareParameter("name", &name_, "User defined descriptive force module name",
                   true);
  DeclareParameter("type", &type_, "force module type", true);
  DeclareParameter("active", &active_,
                   "false loads the force module without evaluating it",
                   false);
}

void shift::core::force_module::Base::Load() {}

void shift::core::force_module::Base::Connect() {}
