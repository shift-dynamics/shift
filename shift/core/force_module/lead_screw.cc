#include "shift/core/force_module/lead_screw.h"

#include <algorithm>
#include <cmath>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::LeadScrew::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "lead_screw", shift::core::force_module::LeadScrew::Create);

std::unique_ptr<shift::core::force_module::LeadScrew>
shift::core::force_module::LeadScrew::Create() {
  return std::make_unique<shift::core::force_module::LeadScrew>();
}

void shift::core::force_module::LeadScrew::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("screw", &screw_names_,
                   "[body, frame] of the screw; the frame is on the axis of "
                   "the thread",
                   true)
    .set_row_dim(2);
  DeclareParameter("nut", &nut_names_,
                   "[body, frame] of the nut; the z axis of the frame is the "
                   "axis of the thread",
                   true)
    .set_row_dim(2);
  DeclareParameter("lead", &lead_,
                   "advance of the screw per turn, negative for a left "
                   "handed thread",
                   true);
  DeclareParameter("mean_radius", &mean_radius_,
                   "radius at which the flank of the thread carries the load",
                   true);
  DeclareParameter("thread_angle", &thread_angle_,
                   "half angle of the thread in a plane through the axis, "
                   "radians; optional, default 0 (a square thread)",
                   false);
  DeclareParameter("friction", &friction_,
                   "coefficient of friction on the thread", true)
    .set_runtime();
  DeclareParameter("stiffness", &stiffness_,
                   "stiffness of the contact of the thread along its normal",
                   true)
    .set_runtime();
  DeclareParameter("damping", &damping_,
                   "damping of the contact of the thread along its normal; "
                   "optional, default 0",
                   false)
    .set_runtime();
  DeclareParameter("backlash", &backlash_,
                   "axial play of the thread; optional, default 0", false);
  DeclareParameter("slip_speed", &slip_speed_,
                   "sliding speed below which the friction goes smoothly to "
                   "zero; optional, default 1e-3",
                   false)
    .set_runtime();
  DeclareParameter("left_handed", &left_handed_,
                   "the thread is left handed; optional, default false",
                   false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness_,
                   "sigma_0, the stiffness of the bristles per unit of normal "
                   "force (1/m), which makes the friction hold at rest; "
                   "optional, default 0 (smoothed Coulomb friction)",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping_,
                   "sigma_1, the damping of the bristles per unit of normal "
                   "force (s/m). With 0 the screw rings on the bristles after "
                   "a load comes on, with nothing to settle it; optional, "
                   "default 5",
                   false);
  DeclareParameter("viscous_friction", &viscous_friction_,
                   "sigma_2, friction in proportion to the sliding speed, per "
                   "unit of normal force (s/m); optional, default 0",
                   false);
  DeclareParameter("static_friction", &static_friction_,
                   "mu_s, the friction coefficient at rest, at least friction; "
                   "optional, default friction",
                   false);
  DeclareParameter("stribeck_velocity", &stribeck_velocity_,
                   "sliding speed over which the friction falls from the "
                   "static to friction; optional, default 0.01",
                   false);
}

shift::core::force_module::LeadScrew::Attachment
shift::core::force_module::LeadScrew::Resolve(
  const std::vector<std::string>& names, const std::string& what) const {
  const std::string prefix = "Lead screw " + get_name() + ": ";
  if(names.size() != 2) {
    throw shift::core::Exception(prefix + what + " must be [body, frame]");
  }
  Attachment a;
  a.body = context_->FindActiveBody(names[0], get_parent_assembly());
  if(!a.body) {
    throw shift::core::Exception(prefix + "body " + names[0] +
                                 " not found among the active bodies");
  }
  for(auto& frame : a.body->get_frames()) {
    if(frame->get_name() == names[1] ||
       (names[1] == "origin" && frame->is_origin())) {
      a.frame = frame.get();
    }
  }
  if(!a.frame) {
    throw shift::core::Exception(prefix + "frame " + names[1] +
                                 " not found on body " + names[0]);
  }
  return a;
}

void shift::core::force_module::LeadScrew::Connect() {
  screw_ = Resolve(screw_names_, "screw");
  nut_ = Resolve(nut_names_, "nut");
  if(screw_.body == nut_.body) {
    throw shift::core::Exception("Lead screw " + get_name() +
                                 ": the screw and the nut must be on "
                                 "different bodies");
  }
}

ksl_SE3_t shift::core::force_module::LeadScrew::Pose(
  const Attachment& a) const {
  if(a.body == context_->get_fixed_body()) {
    return *a.frame->get_d_wrt_origin();
  }
  const ksl_SE3_t origin =
    context_->get_body_origin_pose(a.body, context_->get_dhva());
  ksl_SE3_t pose;
  ksl_product_dd(&origin, a.frame->get_d_wrt_origin(), &pose);
  return pose;
}

double shift::core::force_module::LeadScrew::get_lead_angle() const {
  return std::atan(std::fabs(lead_) / (2.0 * M_PI * mean_radius_));
}

bool shift::core::force_module::LeadScrew::is_self_locking() const {
  const double mu = bristle_stiffness_ > 0.0 && static_friction_ >= 0.0
                      ? static_friction_
                      : friction_;
  return mu / std::cos(thread_angle_) >= std::tan(get_lead_angle());
}

void shift::core::force_module::LeadScrew::Load() {
  const std::string prefix = "Lead screw " + get_name() + ": ";
  if(!(lead_ != 0.0) || !(mean_radius_ > 0.0) || !(stiffness_ > 0.0) ||
     friction_ < 0.0 || damping_ < 0.0 || backlash_ < 0.0 ||
     !(slip_speed_ > 0.0) || !(std::fabs(thread_angle_) < 0.5 * M_PI)) {
    throw shift::core::Exception(
      prefix + "lead and stiffness, mean_radius and slip_speed must be "
               "positive (the lead may be negative), friction, damping and "
               "backlash not negative, and thread_angle less than 90 "
               "degrees");
  }
  if(bristle_stiffness_ < 0.0 || bristle_damping_ < 0.0 ||
     viscous_friction_ < 0.0 || !(stribeck_velocity_ > 0.0) ||
     (static_friction_ >= 0.0 && static_friction_ < friction_)) {
    throw shift::core::Exception(
      prefix + "bristle_stiffness, bristle_damping and viscous_friction must "
               "not be negative, stribeck_velocity must be positive and "
               "static_friction at least friction");
  }
  if(left_handed_ && lead_ < 0.0) {
    throw shift::core::Exception(prefix + "left_handed and a negative lead "
                                 "both say left handed: give one");
  }
  if(lead_ < 0.0) {
    left_handed_ = true;
    lead_ = -lead_;
  }
  screw_.load_index = nut_.load_index = -2;
  /* the thread is unloaded at the start */
  const ksl_SE3_t nut = Pose(nut_), screw = Pose(screw_);
  ksl_vec3_t axis = nut.R.v2, offset;
  ksl_subtract_vv(&screw.t, &nut.t, &offset);
  advance_0_ = ksl_dot_vv(&offset, &axis);
  /* the relative rotation of the screw about the axis, and the deflection of
  the bristles */
  equation_count_ = bristle_stiffness_ > 0.0 ? 2 : 1;
  q_0_.assign(equation_count_, 0.0);
}

/* 0 inside |x| < h, x less h beyond with a smooth start (a quadratic over the
width w), and its slope */
static double Play(double x, double h, double w, double* slope) {
  const double a = std::fabs(x);
  double value, d;
  if(a <= h) {
    value = 0.0;
    d = 0.0;
  } else if(a < h + w) {
    value = (a - h) * (a - h) / (2.0 * w);
    d = (a - h) / w;
  } else {
    value = a - h - 0.5 * w;
    d = 1.0;
  }
  *slope = d;
  return x < 0.0 ? -value : value;
}

void shift::core::force_module::LeadScrew::Service(const double* y,
                                                   double* dy) {
  Compute(y, dy, true);
}

void shift::core::force_module::LeadScrew::UpdateOutputs(double,
                                                         const double* y) {
  Compute(y, nullptr, false);
}

void shift::core::force_module::LeadScrew::Compute(const double* y, double* dy,
                                                   bool apply) {
  Attachment* ends[2] = {&nut_, &screw_};
  for(Attachment* a : ends) {
    if(a->load_index == -2) {
      a->load_index = context_->GetBodyLoadIndex(a->body);
    }
  }
  const ksl_SE3_t nut = Pose(nut_), screw = Pose(screw_);
  const ksl_screw_t v_nut = context_->GetBodyVelocity(nut_.load_index);
  const ksl_screw_t v_screw = context_->GetBodyVelocity(screw_.load_index);
  const ksl_vec3_t axis = nut.R.v2;

  /* the relative advance and rotation rate of the screw: the velocities of
  the point at the screw's frame as part of each body (the spatial velocities
  are referred to the fixed origin, v(p) = v + w x p) */
  ksl_vec3_t offset, w_nut_p, w_screw_p, dv, dw;
  ksl_subtract_vv(&screw.t, &nut.t, &offset);
  ksl_cross_vv(&v_nut.ang, &screw.t, &w_nut_p);
  ksl_cross_vv(&v_screw.ang, &screw.t, &w_screw_p);
  for(int c = 0; c < 3; c++) {
    dv.at[c] = (v_screw.lin.at[c] + w_screw_p.at[c]) -
               (v_nut.lin.at[c] + w_nut_p.at[c]);
    dw.at[c] = v_screw.ang.at[c] - v_nut.ang.at[c];
  }
  const double advance = ksl_dot_vv(&offset, &axis) - advance_0_;
  const double advance_rate = ksl_dot_vv(&dv, &axis);
  const double omega = ksl_dot_vv(&dw, &axis);
  const double theta = y[state_offset_];
  if(dy) {
    dy[state_offset_] = omega;
  }

  /* the advance beyond the ideal screw's, and its rate */
  const double p = (left_handed_ ? -1.0 : 1.0) * lead_ / (2.0 * M_PI);
  const double r = mean_radius_;
  const double deflection = advance - p * theta;
  const double deflection_rate = advance_rate - p * omega;
  double engaged = 1.0;
  double free_deflection = deflection;
  if(backlash_ > 0.0) {
    free_deflection = Play(deflection, 0.5 * backlash_, 0.1 * backlash_,
                           &engaged);
  }

  /* the normal force of the contact: the deflection along the flank's normal
  is cos(alpha) cos(lambda) of it */
  const double lambda = std::atan(std::fabs(p) / r);
  const double scale = std::cos(thread_angle_) * std::cos(lambda);
  const double normal =
    scale * (stiffness_ * free_deflection + engaged * damping_ * deflection_rate);
  const double axial_part = normal * scale;
  /* friction along the thread, opposing the sliding; then the generalized
  forces on the screw, along the axis and about it */
  double friction, force, torque;
  stick_ = 0.0;
  if(bristle_stiffness_ > 0.0) {
    /* the bristles lie along the thread: the part of the friction along the
    axis has the sign of the advance of a turn */
    const double along = p < 0.0 ? -std::sin(lambda) : std::sin(lambda);
    const double across = std::cos(lambda);
    const double u = r * omega * across + advance_rate * along;
    const double z = y[state_offset_ + 1];
    const double mu_s = static_friction_ >= 0.0 ? static_friction_ : friction_;
    const double ratio = u / stribeck_velocity_;
    const double g = friction_ + (mu_s - friction_) * std::exp(-ratio * ratio);
    const double rate =
      g > 0.0 ? u - bristle_stiffness_ * std::fabs(u) * z / g : u;
    if(dy) {
      dy[state_offset_ + 1] = rate;
    }
    const double along_thread =
      std::fabs(normal) * (bristle_stiffness_ * z + bristle_damping_ * rate +
                           viscous_friction_ * u);
    friction = std::fabs(along_thread);
    force = -axial_part - along_thread * along;
    torque = p * axial_part - along_thread * r * across;
    stick_ = mu_s > 0.0 ? bristle_stiffness_ * std::fabs(z) / mu_s : 0.0;
  } else {
    const double sliding = std::sqrt((r * omega) * (r * omega) +
                                     advance_rate * advance_rate +
                                     slip_speed_ * slip_speed_);
    friction = friction_ * std::fabs(normal);
    force = -axial_part - friction * advance_rate / sliding;
    torque = p * axial_part - friction * r * r * omega / sliding;
  }

  axial_force_ = force;
  torque_ = torque;
  deflection_ = deflection;
  normal_force_ = normal;
  friction_force_ = friction;
  turns_ = theta / (2.0 * M_PI);

  if(!apply) {
    return;
  }
  /* the force and moment on the screw at its frame, and the opposite on the
  nut, as spatial forces in the fixed frame referred to its origin */
  ksl_coscrew_t load;
  for(int c = 0; c < 3; c++) {
    load.lin.at[c] = force * axis.at[c];
    load.ang.at[c] = torque * axis.at[c];
  }
  ksl_vec3_t p_cross_f;
  ksl_cross_vv(&screw.t, &load.lin, &p_cross_f);
  for(int c = 0; c < 3; c++) {
    load.ang.at[c] += p_cross_f.at[c];
  }
  context_->AddBodyLoad(screw_.load_index, load);
  for(int c = 0; c < 6; c++) {
    load.at[c] = -load.at[c];
  }
  context_->AddBodyLoad(nut_.load_index, load);
}

std::vector<std::string>
shift::core::force_module::LeadScrew::get_output_names() const {
  return {"axial_force",  "torque",         "deflection",
          "normal_force", "friction_force", "turns",
          "stick"};
}

void shift::core::force_module::LeadScrew::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), {axial_force_, torque_, deflection_,
                               normal_force_, friction_force_, turns_, stick_});
}
