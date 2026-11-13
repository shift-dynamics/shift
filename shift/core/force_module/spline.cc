#include "shift/core/force_module/spline.h"

#include <algorithm>
#include <cmath>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/table.h"
#include "shift/core/frame.h"

bool shift::core::force_module::Spline::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "spline", shift::core::force_module::Spline::Create);

std::unique_ptr<shift::core::force_module::Spline>
shift::core::force_module::Spline::Create() {
  return std::make_unique<shift::core::force_module::Spline>();
}

void shift::core::force_module::Spline::DeclareParameters() {
  DeclareParameter("frames", &frame_names_,
                   "[[body, frame], [body, frame]]: a frame on each half, on "
                   "the axis; the first frame's z axis is the axis",
                   true)
    .set_row_dim(2)
    .set_column_dim(2);
  DeclareParameter("pitch_radius", &pitch_radius_, "of the teeth", true);
  DeclareParameter("pressure_angle", &pressure_angle_,
                   "of the teeth, radians; optional, default 0.5236 (30 "
                   "degrees)",
                   false);
  DeclareParameter("torsional_stiffness", &torsional_stiffness_,
                   "torque per angle, past the backlash", true);
  DeclareParameter("torsional_damping", &torsional_damping_,
                   "torque per rate of the angle, past the backlash; optional, "
                   "default 10. With 0 the halves ring on the teeth, with "
                   "little to settle them",
                   false);
  DeclareParameter("backlash", &backlash_,
                   "the whole angle the halves turn freely, from one flank to "
                   "the other; optional, default 0",
                   false);
  DeclareParameter("smoothing", &smoothing_,
                   "width in angle of the edge of the backlash, over which the "
                   "torque comes in; optional, default 1e-4",
                   false);
  DeclareParameter("preload", &preload_,
                   "normal force on the teeth without torque; optional, "
                   "default 0",
                   false);
  DeclareParameter("friction", &friction_,
                   "coefficient of sliding friction of the teeth; optional, "
                   "default 0.1",
                   false);
  DeclareParameter("static_friction", &static_friction_,
                   "coefficient that holds the spline at rest; optional, "
                   "default the friction",
                   false);
  DeclareParameter("viscous_friction", &viscous_friction_,
                   "force per axial speed; optional, default 0", false);
  DeclareParameter("bristle_stiffness", &bristle_stiffness_,
                   "sigma_0, per unit of the normal force, 1/m: the friction "
                   "over this is how far the spline gives before it slides; "
                   "optional, default 2e4",
                   false);
  DeclareParameter("bristle_damping", &bristle_damping_,
                   "sigma_1, per unit of the normal force, s/m; optional, "
                   "default 5. With 0 the bristles ring, with little to settle "
                   "them",
                   false);
  DeclareParameter("bristle_time", &bristle_time_,
                   "the shortest time the bristles take to reach their sliding "
                   "deflection; optional, default 1e-4. It changes the friction "
                   "only for that moment, as the spline breaks loose. With 0 "
                   "there is no limit: the plain LuGre model, whose time step "
                   "falls with the sliding speed",
                   false);
  DeclareParameter("stribeck_velocity", &stribeck_velocity_,
                   "speed over which static friction falls to sliding "
                   "friction; optional, default 0.01",
                   false);
  DeclareOutput("angle", &angle_, "twist of the spline");
  DeclareOutput("torque", &torque_, "torque through the spline");
  DeclareOutput("normal_force", &normal_force_, "normal force on the teeth");
  DeclareOutput("friction_force", &friction_force_,
                "friction force of the teeth along the axis");
  DeclareOutput("stick", &stick_,
                "share of the static friction in use, 1 at the limit of "
                "holding");
}

void shift::core::force_module::Spline::Connect() {
  const std::string prefix = "Spline " + get_name() + ": ";
  if(frame_names_.size() != 2) {
    throw shift::core::Exception(prefix + "frames must name two frames");
  }
  for(int k = 0; k < 2; k++) {
    const auto& names = frame_names_[k];
    if(names.size() != 2) {
      throw shift::core::Exception(prefix + "each of frames must be "
                                            "[body, frame]");
    }
    Attachment& a = attachments_[k];
    a.body = context_->FindActiveBody(names[0], get_parent_assembly());
    if(!a.body) {
      throw shift::core::Exception(prefix + "body " + names[0] +
                                   " not found among the active bodies");
    }
    a.frame = nullptr;
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
  }
}

ksl_SE3_t
shift::core::force_module::Spline::Pose(const Attachment& a) const {
  if(a.body == context_->get_fixed_body()) {
    return *a.frame->get_d_wrt_origin();
  }
  const ksl_SE3_t origin =
    context_->get_body_origin_pose(a.body, context_->get_dhva());
  ksl_SE3_t pose;
  ksl_product_dd(&origin, a.frame->get_d_wrt_origin(), &pose);
  return pose;
}

void shift::core::force_module::Spline::Load() {
  const std::string prefix = "Spline " + get_name() + ": ";
  if(!(pitch_radius_ > 0.0) || !(torsional_stiffness_ > 0.0)) {
    throw shift::core::Exception(prefix + "pitch_radius and "
                                          "torsional_stiffness must be "
                                          "positive");
  }
  if(pressure_angle_ < 0.0 || pressure_angle_ >= 0.5 * M_PI) {
    throw shift::core::Exception(prefix + "pressure_angle must be from 0 to "
                                          "below a quarter turn");
  }
  if(static_friction_ < 0.0) {
    static_friction_ = friction_;
  }
  if(torsional_damping_ < 0.0 || backlash_ < 0.0 || preload_ < 0.0 ||
     friction_ < 0.0 || static_friction_ < friction_ ||
     viscous_friction_ < 0.0 || bristle_damping_ < 0.0 || bristle_time_ < 0.0) {
    throw shift::core::Exception(prefix + "torsional_damping, backlash, "
                                          "preload, friction, viscous_friction, "
                                          "bristle_damping and bristle_time "
                                          "must not be negative, nor "
                                          "static_friction below friction");
  }
  if(!(smoothing_ > 0.0) || !(bristle_stiffness_ > 0.0) ||
     !(stribeck_velocity_ > 0.0)) {
    throw shift::core::Exception(prefix + "smoothing, bristle_stiffness and "
                                          "stribeck_velocity must be positive");
  }
  for(auto& attachment : attachments_) {
    attachment.load_index = -2;
  }
  equation_count_ = 2;
  q_0_.assign(equation_count_, 0.0);
}

double shift::core::force_module::Spline::NormalForce(double torque) const {
  return std::fabs(torque) / (pitch_radius_ * std::cos(pressure_angle_)) +
         preload_;
}

void shift::core::force_module::Spline::Compute(const double* y, double* dy) {
  ksl_screw_t velocity[2];
  for(int k = 0; k < 2; k++) {
    Attachment& a = attachments_[k];
    if(a.load_index == -2) {
      a.load_index = context_->GetBodyLoadIndex(a.body);
    }
    poses_[k] = Pose(a);
    velocity[k] = context_->GetBodyVelocity(a.load_index);
  }
  axis_ = {{poses_[0].R.at[2][0], poses_[0].R.at[2][1], poses_[0].R.at[2][2]}};
  /* the velocity of each origin: v + w x p, the spatial velocities being
  referred to the origin of the fixed frame */
  ksl_vec3_t wi, wj, w;
  ksl_cross_vv(&velocity[0].ang, &poses_[0].t, &wi);
  ksl_cross_vv(&velocity[1].ang, &poses_[1].t, &wj);
  ksl_subtract_vv(&velocity[1].ang, &velocity[0].ang, &w);
  double v = 0.0;
  for(int c = 0; c < 3; c++) {
    v += axis_.at[c] * ((velocity[1].lin.at[c] + wj.at[c]) -
                        (velocity[0].lin.at[c] + wi.at[c]));
  }
  const double rate = ksl_dot_vv(&w, &axis_);
  angle_ = y[state_offset_];
  const double z = y[state_offset_ + 1];

  /* the torque, past the backlash on either side */
  const double gap = 0.5 * backlash_;
  double deflection = angle_, weight = 1.0;
  if(backlash_ > 0.0) {
    double ahead, behind, slope_ahead, slope_behind;
    Ramp(angle_ - gap, smoothing_, ahead, slope_ahead);
    Ramp(-angle_ - gap, smoothing_, behind, slope_behind);
    deflection = ahead - behind;
    weight = std::min(slope_ahead + slope_behind, 1.0);
  }
  torque_ = -weight * (torsional_stiffness_ * deflection +
                       torsional_damping_ * rate);
  normal_force_ = NormalForce(torque_);

  const double e = v / stribeck_velocity_;
  const double g =
    friction_ + (static_friction_ - friction_) * std::exp(-e * e);
  /* z relaxes towards its sliding value at this rate. Held to 1 / bristle_time,
  which leaves the sliding friction as it is, or it would set the time step of
  the whole model while the spline slides. A coefficient of zero leaves no
  bristles to bend */
  double dz = 0.0;
  if(g > 0.0) {
    const double relaxation = bristle_stiffness_ * std::fabs(v) / g;
    if(bristle_time_ > 0.0 && relaxation * bristle_time_ > 1.0) {
      const double sliding = std::copysign(g / bristle_stiffness_, v);
      dz = (sliding - z) / bristle_time_;
    } else {
      dz = v - relaxation * z;
    }
  }
  friction_force_ = -normal_force_ * (bristle_stiffness_ * z +
                                      bristle_damping_ * dz) -
                    viscous_friction_ * v;
  stick_ = static_friction_ > 0.0
             ? bristle_stiffness_ * std::fabs(z) / static_friction_
             : 0.0;
  if(dy) {
    dy[state_offset_] = rate;
    dy[state_offset_ + 1] = dz;
  }
}

void shift::core::force_module::Spline::Apply() {
  ksl_coscrew_t load;
  for(int k = 0; k < 2; k++) {
    /* on the second half along the axis, on the first against it, as spatial
    forces in the fixed frame referred to its origin */
    const double sign = k == 0 ? -1.0 : 1.0;
    for(int c = 0; c < 3; c++) {
      load.lin.at[c] = sign * friction_force_ * axis_.at[c];
    }
    ksl_cross_vv(&poses_[k].t, &load.lin, &load.ang);
    for(int c = 0; c < 3; c++) {
      load.ang.at[c] += sign * torque_ * axis_.at[c];
    }
    context_->AddBodyLoad(attachments_[k].load_index, load);
  }
}

void shift::core::force_module::Spline::Service(const double* y, double* dy) {
  Compute(y, dy);
  Apply();
}

void shift::core::force_module::Spline::UpdateOutputs(double t,
                                                      const double* y) {
  (void)t;
  Compute(y, nullptr);
}
