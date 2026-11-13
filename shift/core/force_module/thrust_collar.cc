#include "shift/core/force_module/thrust_collar.h"

#include <algorithm>
#include <cmath>

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::ThrustCollar::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "thrust_collar", shift::core::force_module::ThrustCollar::Create);

std::unique_ptr<shift::core::force_module::ThrustCollar>
shift::core::force_module::ThrustCollar::Create() {
  return std::make_unique<shift::core::force_module::ThrustCollar>();
}

void shift::core::force_module::ThrustCollar::DeclareParameters() {
  DeclareParameter("frames", &frame_names_,
                   "[[body, frame], [body, frame]]: a frame on each face, on "
                   "the axis; the first frame's z axis is the axis, pointing "
                   "from its face to the other",
                   true)
    .set_row_dim(2)
    .set_column_dim(2);
  DeclareParameter("inner_radius", &inner_radius_,
                   "of the ring of the faces; optional, default 0, a disc",
                   false);
  DeclareParameter("outer_radius", &outer_radius_, "of the ring of the faces",
                   true);
  DeclareParameter("viscosity", &viscosity_,
                   "of the oil, Pa s; optional, default 0, dry faces", false);
  DeclareParameter("end_play", &end_play_,
                   "the collar's travel between two shoulders; optional, "
                   "default 0, a single face",
                   false);
  DeclareParameter("roughness", &roughness_,
                   "of the faces together, the film they leave, and half the "
                   "width of the edge of the contact; optional, default 1e-6",
                   false);
  DeclareParameter("pads", &pads_,
                   "number of tapered pads on a face; optional, default 0, "
                   "flat faces, which carry no steady load on the oil",
                   false);
  DeclareParameter("taper", &taper_,
                   "rise of a pad across its length; needed with pads",
                   false);
  DeclareParameter("contact_stiffness", &contact_stiffness_,
                   "force per depth of the contact of the surfaces; optional, "
                   "default 1e9",
                   false);
  DeclareParameter("contact_damping", &contact_damping_,
                   "force per speed in the contact of the surfaces; optional, "
                   "default 1000. With 0 a collar rings on the surfaces, with "
                   "only the film to settle it",
                   false);
  DeclareParameter("friction", &friction_,
                   "coefficient of friction of the surfaces in contact; "
                   "optional, default 0.1",
                   false);
  DeclareParameter("film_damping_limit", &film_damping_limit_,
                   "what the squeeze of a thin film levels off at, force per "
                   "speed: raise it for the true squeeze of a thin film, with "
                   "steps below mass over this limit; optional, default 1e5",
                   false);
  DeclareParameter("slip_speed", &slip_speed_,
                   "closing speed over which the squeeze comes in; optional, "
                   "default 1e-6",
                   false);
  DeclareParameter("slip_rate", &slip_rate_,
                   "speed of rotation below which the friction of the surfaces "
                   "falls to zero with the speed; optional, default 0.1",
                   false);
  DeclareOutput("gap", &gap_, "gap between the faces");
  DeclareOutput("film_force", &film_force_, "force of the oil film");
  DeclareOutput("contact_force", &contact_force_,
                "force of the faces in contact");
  DeclareOutput("friction_torque", &friction_torque_,
                "friction torque about the axis");
}

void shift::core::force_module::ThrustCollar::Connect() {
  const std::string prefix = "Thrust collar " + get_name() + ": ";
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

ksl_SE3_t shift::core::force_module::ThrustCollar::Pose(
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

void shift::core::force_module::ThrustCollar::Load() {
  const std::string prefix = "Thrust collar " + get_name() + ": ";
  if(inner_radius_ < 0.0 || !(outer_radius_ > inner_radius_)) {
    throw shift::core::Exception(prefix + "outer_radius must be greater than "
                                          "inner_radius, which must not be "
                                          "negative");
  }
  if(viscosity_ < 0.0 || end_play_ < 0.0 || contact_stiffness_ < 0.0 ||
     contact_damping_ < 0.0 || friction_ < 0.0) {
    throw shift::core::Exception(prefix + "viscosity, end_play, "
                                          "contact_stiffness, contact_damping "
                                          "and friction must not be negative");
  }
  if(!(roughness_ > 0.0) || !(film_damping_limit_ > 0.0) ||
     !(slip_speed_ > 0.0) || !(slip_rate_ > 0.0)) {
    throw shift::core::Exception(prefix + "roughness, film_damping_limit, "
                                          "slip_speed and slip_rate must be "
                                          "positive");
  }
  if(pads_ < 0 || taper_ < 0.0 || (pads_ > 0) != (taper_ > 0.0)) {
    throw shift::core::Exception(prefix + "give pads and taper together");
  }
  const double a = inner_radius_, b = outer_radius_;
  const double a2 = a * a, b2 = b * b;
  double ring = b2 * b2 - a2 * a2;
  const double quartic = ring;
  if(a > 0.0) {
    ring -= (b2 - a2) * (b2 - a2) / std::log(b / a);
  }
  squeeze_factor_ = 1.5 * M_PI * viscosity_ * ring;
  shear_factor_ = 0.5 * M_PI * viscosity_ * quartic;
  friction_radius_ = 2.0 / 3.0 * (b2 * b - a2 * a) / (b2 - a2);
  for(auto& attachment : attachments_) {
    attachment.load_index = -2;
  }
}

double
shift::core::force_module::ThrustCollar::FilmThickness(double gap) const {
  return roughness_ +
         0.5 * (gap + std::sqrt(gap * gap + roughness_ * roughness_));
}

double shift::core::force_module::ThrustCollar::SqueezeDamping(
  double film) const {
  return squeeze_factor_ / (film * film * film);
}

double shift::core::force_module::ThrustCollar::PadLoad(double film,
                                                        double speed) const {
  if(pads_ == 0) {
    return 0.0;
  }
  const double mean = 0.5 * (inner_radius_ + outer_radius_);
  const double length = 2.0 * M_PI * mean / pads_;
  const double width = outer_radius_ - inner_radius_;
  const double k = taper_ / film;
  /* ln(1 + k) - 2 k / (2 + k) over k^2: k / 12 for a small k, where the
  difference cancels */
  const double shape =
    k < 1e-2 ? k / 12.0 * (1.0 - k)
             : (std::log1p(k) - 2.0 * k / (2.0 + k)) / (k * k);
  return pads_ * 6.0 * viscosity_ * mean * std::fabs(speed) * length * length *
         width * shape / (film * film);
}

void shift::core::force_module::ThrustCollar::Face(double gap, double v,
                                                   double omega, double& film,
                                                   double& contact,
                                                   double& torque) const {
  const double h = FilmThickness(gap);
  const double c = SqueezeDamping(h);
  const double limited = c * film_damping_limit_ / (c + film_damping_limit_);
  /* closing only, from zero at rest: v above slip_speed, a cubic below */
  const double x = std::min(std::max(v / slip_speed_, 0.0), 1.0);
  film = limited * v * x * x * (3.0 - 2.0 * x) + PadLoad(h, omega);
  double depth, slope;
  Ramp(-gap, 2.0 * roughness_, depth, slope);
  contact = std::max(0.0, contact_stiffness_ * depth +
                            contact_damping_ * slope * v);
  torque = -shear_factor_ / (h + 0.5 * taper_) * omega -
           friction_ * contact * friction_radius_ * omega /
             std::sqrt(omega * omega + slip_rate_ * slip_rate_);
}

void shift::core::force_module::ThrustCollar::Compute() {
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
  ksl_vec3_t wi, wj, w, d;
  ksl_cross_vv(&velocity[0].ang, &poses_[0].t, &wi);
  ksl_cross_vv(&velocity[1].ang, &poses_[1].t, &wj);
  ksl_subtract_vv(&velocity[1].ang, &velocity[0].ang, &w);
  ksl_subtract_vv(&poses_[1].t, &poses_[0].t, &d);
  double rate = 0.0;
  for(int c = 0; c < 3; c++) {
    rate += axis_.at[c] * ((velocity[1].lin.at[c] + wj.at[c]) -
                           (velocity[0].lin.at[c] + wi.at[c]));
  }
  gap_ = ksl_dot_vv(&d, &axis_);
  const double omega = ksl_dot_vv(&w, &axis_);
  double film, contact, torque;
  Face(gap_, -rate, omega, film, contact, torque);
  film_force_ = film;
  contact_force_ = contact;
  friction_torque_ = torque;
  axial_force_ = film + contact;
  if(end_play_ > 0.0) {
    Face(end_play_ - gap_, rate, omega, film, contact, torque);
    film_force_ += film;
    contact_force_ += contact;
    friction_torque_ += torque;
    axial_force_ -= film + contact;
  }
}

void shift::core::force_module::ThrustCollar::Apply() {
  ksl_coscrew_t load;
  for(int k = 0; k < 2; k++) {
    /* on the second body along the axis, on the first against it, as spatial
    forces in the fixed frame referred to its origin */
    const double sign = k == 0 ? -1.0 : 1.0;
    for(int c = 0; c < 3; c++) {
      load.lin.at[c] = sign * axial_force_ * axis_.at[c];
    }
    ksl_cross_vv(&poses_[k].t, &load.lin, &load.ang);
    for(int c = 0; c < 3; c++) {
      load.ang.at[c] += sign * friction_torque_ * axis_.at[c];
    }
    context_->AddBodyLoad(attachments_[k].load_index, load);
  }
}

void shift::core::force_module::ThrustCollar::Service(const double* y,
                                                      double* dy) {
  (void)y;
  (void)dy;
  Compute();
  Apply();
}

void shift::core::force_module::ThrustCollar::UpdateOutputs(double t,
                                                            const double* y) {
  (void)t;
  (void)y;
  Compute();
}
