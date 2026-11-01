#include "shift/core/force_module/tire/base.h"

#include <cmath>
#include <vector>

#include "ksl/coscrew.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"

#include "shift/core/assembly.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/log.h"
#include "shift/core/utility.h"

bool shift::core::force_module::tire::Base::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "tire", shift::core::force_module::tire::Base::Create);

std::unique_ptr<shift::core::force_module::tire::Base>
shift::core::force_module::tire::Base::Create() {
  return std::make_unique<shift::core::force_module::tire::Base>();
}

void shift::core::force_module::tire::Base::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("tire_type", &tire_type_, "tire model: \"fiala\" or \"pac2002\"", true);
  DeclareParameter("wheel_body", &wheel_body_name_,
                   "name of the wheel body, in the same assembly; the wheel "
                   "spins about the y axis of its origin frame",
                   true);

  FialaParameters& f = fiala_;
  DeclareParameter("unloaded_radius", &f.unloaded_radius,
                   "Fiala: unloaded radius, defaults to 0.344", false);
  DeclareParameter("vertical_stiffness", &f.vertical_stiffness,
                   "Fiala: vertical stiffness, defaults to 310000", false);
  DeclareParameter("vertical_damping", &f.vertical_damping,
                   "Fiala: vertical damping, defaults to 3100", false);
  DeclareParameter("rolling_resistance", &f.rolling_resistance,
                   "Fiala: rolling resistance coefficient, defaults to 0.001",
                   false);
  DeclareParameter("width", &f.width, "Fiala: tire width, defaults to 0.235",
                   false);
  DeclareParameter("cslip", &f.cslip,
                   "Fiala: longitudinal slip stiffness, defaults to 1000000",
                   false);
  DeclareParameter("calpha", &f.calpha,
                   "Fiala: cornering stiffness, defaults to 45836.6236", false);
  DeclareParameter("u_max", &f.u_max,
                   "Fiala: friction coefficient without sliding, defaults to "
                   "1.0",
                   false);
  DeclareParameter("u_min", &f.u_min,
                   "Fiala: friction coefficient when sliding, defaults to 0.9",
                   false);
  DeclareParameter("x_relaxation_length", &f.x_relaxation_length,
                   "Fiala: longitudinal relaxation length, defaults to 0.05",
                   false);
  DeclareParameter("y_relaxation_length", &f.y_relaxation_length,
                   "Fiala: lateral relaxation length, defaults to 0.15", false);

  DeclareParameter("definition", &definition_,
                   "PAC2002: .tir property file, next to the model file or "
                   "in an include directory",
                   false);
  DeclareParameter("mounted_side", &mounted_side_,
                   "PAC2002: \"left\" or \"right\"; the tire is mirrored if "
                   "this differs from TYRESIDE of the property file",
                   false);
  DeclareParameter("use_transient_slip", &use_transient_slip_,
                   "PAC2002: must be false, the model is steady state", false);
}

std::string shift::core::force_module::tire::Base::FindFile(
  const std::string& name) const {
  if(!name.empty() && name[0] == '/') {
    return name;
  }
  const std::string file =
    shift::core::utility::GetParentPath(context_->get_filename()) + "/" + name;
  if(shift::core::utility::FileExists(file)) {
    return file;
  }
  for(const auto& directory :
      (*context_)["model_parameters"]["include_directories"]
        .get<std::vector<std::string>>()) {
    const std::string candidate = directory + "/" + name;
    if(shift::core::utility::FileExists(candidate)) {
      return candidate;
    }
  }
  throw shift::core::Exception("Tire " + get_name() + ": file " + name +
                               " not found");
}

void shift::core::force_module::tire::Base::Load() {
  if(tire_type_ == "fiala") {
    model_ = std::make_unique<Fiala>(fiala_);
  } else if(tire_type_ == "pac2002") {
    const std::string prefix = "Tire " + get_name() + ": ";
    if(definition_.empty()) {
      throw shift::core::Exception(prefix + "pac2002 requires a definition "
                                            "(.tir property file)");
    }
    if(use_transient_slip_) {
      throw shift::core::Exception(prefix + "pac2002 is steady state, "
                                            "use_transient_slip is not "
                                            "supported");
    }
    Pac2002Coefficients coefficients =
      Pac2002Coefficients::Read(FindFile(definition_));
    bool mirror = coefficients.mirror;
    if(!mounted_side_.empty()) {
      if(mounted_side_ != "left" && mounted_side_ != "right") {
        throw shift::core::Exception(prefix + "mounted_side must be "
                                              "\"left\" or \"right\"");
      }
      mirror = mirror != ((mounted_side_ == "left") !=
                          coefficients.measured_left);
    }
    if(mirror) {
      coefficients.Mirror();
    }
    if(coefficients.relaxation) {
      SHIFT_INFO(prefix + "the property file enables relaxation; pac2002 is "
                          "steady state");
    }
    model_ = std::make_unique<Pac2002>(
      coefficients,
      ksl_vec3_l2norm(context_->get_model_parameters().get_gravity()));
  } else {
    throw shift::core::Exception("Tire " + get_name() + ": tire_type \"" +
                                 tire_type_ + "\" is not supported");
  }
  equation_count_ = model_->get_state_count();
  q_0_.assign(equation_count_, 0.0);
  force_w_ = ksl_coscrew(0, 0, 0, 0, 0, 0);
}

void shift::core::force_module::tire::Base::Connect() {
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for tire " +
                                 get_name());
  }
  try {
    wheel_body_ = (*parent_assembly_ptr_)["bodies"][wheel_body_name_]
                    .get_data<shift::core::body::Base>();
  } catch(...) {
    wheel_body_ = nullptr;
  }
  if(wheel_body_ == nullptr) {
    throw shift::core::Exception("Tire " + get_name() + ": wheel body '" +
                                 wheel_body_name_ +
                                 "' not found in its assembly");
  }
  wheel_origin_ = nullptr;
  for(auto& frame : wheel_body_->get_frames()) {
    if(frame->is_origin()) {
      wheel_origin_ = frame.get();
    }
  }
  if(wheel_origin_ == nullptr) {
    throw shift::core::Exception("Tire " + get_name() +
                                 ": wheel body has no origin frame");
  }
  load_index_ = -2;
}

/* port of the contact search of the original Fiala implementation, steps
numbered as in the source */
void shift::core::force_module::tire::Base::UpdateContact(
  const ksl_SE3_t& hub, const ksl_screw_t& velocity) {

  contact_.in_contact = false;
  contact_.depth = 0.0;
  const double r0 = model_->get_unloaded_radius();

  /* flat terrain at z = 0, normal +z (into the terrain, z down) */
  const double terrain_height = 0.0;
  const ksl_vec3_t terrain_normal = {{0.0, 0.0, 1.0}};

  const ksl_vec3_t& center = hub.t;
  const ksl_vec3_t& wheel_normal = hub.R.v1; // spin axis: hub y axis

  /* no contact if the disc center is below the terrain or farther above it
  than the radius */
  if(center.z >= terrain_height || center.z <= terrain_height - r0) {
    return;
  }

  /* 4: heading in the terrain plane; a horizontal wheel has no contact */
  ksl_vec3_t x_direction;
  ksl_cross_vv(&wheel_normal, &terrain_normal, &x_direction);
  const double sin_tilt = ksl_vec3_l2norm(&x_direction);
  if(sin_tilt < 0.01) {
    return;
  }

  /* 6: lowest point of the disc */
  ksl_vec3_t down;
  ksl_cross_vv(&x_direction, &wheel_normal, &down);
  ksl_vec3_t lowest = center;
  for(int i = 0; i < 3; i++) {
    lowest.at[i] += r0 / sin_tilt * down.at[i];
  }
  if(lowest.z <= terrain_height) {
    return;
  }

  /* 15 */
  contact_.depth = std::fabs(terrain_height - lowest.z) * terrain_normal.z;
  contact_.in_contact = true;

  /* 10 - 14: contact frame, x heading, z terrain normal, y = z x x */
  ksl_cross_vv(&wheel_normal, &terrain_normal, &x_direction);
  ksl_vec3_normalize(&x_direction);
  ksl_vec3_t y_direction;
  ksl_cross_vv(&terrain_normal, &x_direction, &y_direction);
  ksl_SE3_t& w = contact_.w_frame;
  w.R.v0 = x_direction;
  w.R.v1 = y_direction;
  w.R.v2 = terrain_normal;
  w.t = {{lowest.x, lowest.y, terrain_height}};

  /* 16 - 17: velocity of the wheel's material point at the contact point,
  in the w frame; the spatial velocity is referred to the fixed origin, so
  the velocity of a point p is v + w x p */
  ksl_vec3_t w_cross_p;
  ksl_cross_vv(&velocity.ang, &w.t, &w_cross_p);
  ksl_vec3_t contact_velocity = {{velocity.lin.x + w_cross_p.x,
                                  velocity.lin.y + w_cross_p.y,
                                  velocity.lin.z + w_cross_p.z}};
  contact_.contact_velocity_w = {{ksl_dot_vv(&w.R.v0, &contact_velocity),
                                  ksl_dot_vv(&w.R.v1, &contact_velocity),
                                  ksl_dot_vv(&w.R.v2, &contact_velocity)}};

  /* 20: hub velocity along the heading */
  ksl_vec3_t w_cross_c;
  ksl_cross_vv(&velocity.ang, &center, &w_cross_c);
  ksl_vec3_t hub_velocity = {{velocity.lin.x + w_cross_c.x,
                              velocity.lin.y + w_cross_c.y,
                              velocity.lin.z + w_cross_c.z}};
  contact_.vx = ksl_dot_vv(&hub_velocity, &x_direction);

  /* velocity of the contact center: the material velocity at the w frame
  origin less the spin, (w . a) a x (p - c) */
  const double spin = ksl_dot_vv(&velocity.ang, &wheel_normal);
  ksl_vec3_t arm = {{w.t.x - center.x, w.t.y - center.y, w.t.z - center.z}};
  ksl_vec3_t spin_velocity;
  ksl_cross_vv(&wheel_normal, &arm, &spin_velocity);
  ksl_vec3_t center_velocity = {{contact_velocity.x - spin * spin_velocity.x,
                                 contact_velocity.y - spin * spin_velocity.y,
                                 contact_velocity.z - spin * spin_velocity.z}};
  contact_.center_velocity_w = {{ksl_dot_vv(&w.R.v0, &center_velocity),
                                 ksl_dot_vv(&w.R.v1, &center_velocity),
                                 ksl_dot_vv(&w.R.v2, &center_velocity)}};

  /* spin axis in the w frame */
  contact_.spin_axis_w = {{ksl_dot_vv(&w.R.v0, &wheel_normal),
                           ksl_dot_vv(&w.R.v1, &wheel_normal),
                           ksl_dot_vv(&w.R.v2, &wheel_normal)}};
}

void shift::core::force_module::tire::Base::Service(const double* y,
                                                    double* dy) {
  assert(context_ && model_ && wheel_origin_);
  if(load_index_ == -2) {
    load_index_ = context_->GetBodyLoadIndex(wheel_body_);
  }
  const ksl_SE3_t& hub = context_->get_dhva().d[wheel_origin_->get_index()];
  const ksl_screw_t velocity = context_->GetBodyVelocity(load_index_);

  UpdateContact(hub, velocity);
  /* 35: spin rate about the wheel axis */
  contact_.omega = ksl_dot_vv(&hub.R.v1, &velocity.ang);

  model_->Evaluate(contact_, y + state_offset_, dy + state_offset_, &force_w_);
  if(!contact_.in_contact) {
    return;
  }

  /* force and moment at the contact point, in the fixed frame and referred
  to its origin: (R f, R m + p x R f) */
  const ksl_SE3_t& w = contact_.w_frame;
  ksl_coscrew_t load;
  for(int i = 0; i < 3; i++) {
    load.lin.at[i] = w.R.v0.at[i] * force_w_.lin.x +
                     w.R.v1.at[i] * force_w_.lin.y +
                     w.R.v2.at[i] * force_w_.lin.z;
    load.ang.at[i] = w.R.v0.at[i] * force_w_.ang.x +
                     w.R.v1.at[i] * force_w_.ang.y +
                     w.R.v2.at[i] * force_w_.ang.z;
  }
  ksl_vec3_t p_cross_f;
  ksl_cross_vv(&w.t, &load.lin, &p_cross_f);
  for(int i = 0; i < 3; i++) {
    load.ang.at[i] += p_cross_f.at[i];
  }
  context_->AddBodyLoad(load_index_, load);
}
