#include "shift/core/force_module/spring_damper.h"

#include <algorithm>
#include <cmath>

#include "ksl/mat3x3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::SpringDamper::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "spring_damper", shift::core::force_module::SpringDamper::Create);

std::unique_ptr<shift::core::force_module::SpringDamper>
shift::core::force_module::SpringDamper::Create() {
  return std::make_unique<shift::core::force_module::SpringDamper>();
}

void shift::core::force_module::BoucWen::DeclareParameters() {
  DeclareParameter("yield_force", &yield_force,
                   "the force the loop levels off at, for beta + gamma = 1",
                   true);
  DeclareParameter("yield_displacement", &yield_displacement,
                   "the displacement of the first slope, yield_force over "
                   "that slope",
                   true);
  DeclareParameter("exponent", &exponent,
                   "the sharpness of the corner of the loop, the larger the "
                   "sharper; optional, default 1",
                   false);
  DeclareParameter("beta", &beta, "shape; optional, default 0.5", false);
  DeclareParameter("gamma", &gamma, "shape; optional, default 0.5", false);
}

void shift::core::force_module::BoucWen::Check(
  const std::string& owner) const {
  if(!(yield_displacement > 0.0) || !(exponent > 0.0) ||
     !(beta + gamma > 0.0) || !(beta - gamma >= 0.0)) {
    throw shift::core::Exception(
      owner + ": hysteresis needs a positive yield_displacement and "
              "exponent, and beta >= |gamma| with beta + gamma > 0");
  }
}

double shift::core::force_module::BoucWen::Rate(
  double z, double displacement_rate) const {
  const double sign = displacement_rate * z >= 0.0 ? 1.0 : -1.0;
  return displacement_rate / yield_displacement *
         (1.0 - std::pow(std::abs(z), exponent) * (beta + gamma * sign));
}

void shift::core::force_module::SpringDamper::DeclareParameters() {
  DeclareParameter("frames", &frame_names_,
                   "[body, frame] of the two frames joined", true)
    .set_row_dim(-1)
    .set_column_dim(2);
  DeclareParameter("axis", &axis_,
                   "for a torsion spring: the axis in frame i's coordinates; "
                   "optional, without it the spring is along the line "
                   "between the frames",
                   false);
  DeclareParameter("free_length", &free_length_,
                   "the length at which the deflection is zero; optional, "
                   "default the length as the model starts",
                   false);
  DeclareParameter("free_angle", &free_angle_,
                   "with axis: the angle from the start at which the "
                   "deflection is zero; optional, default 0",
                   false);
  DeclareParameter("stiffness", &stiffness_,
                   "force per deflection; optional, default 0", false);
  DeclareParameter("preload", &preload_,
                   "force at zero deflection, positive in tension; optional, "
                   "default 0",
                   false);
  DeclareParameter("damping", &damping_,
                   "force per rate of the deflection; optional, default 0",
                   false);
  DeclareParameter("force_curve", &force_curve_values_,
                   "pairs [x0, f0, x1, f1, ...] of deflection and force, "
                   "positive in tension, in increasing x; in place of "
                   "stiffness and preload",
                   false)
    .set_row_dim(-1);
  DeclareParameter("damping_curve", &damping_curve_values_,
                   "pairs [v0, f0, v1, f1, ...] of the rate of the deflection "
                   "and force, in increasing v, negative v in compression; "
                   "in place of damping",
                   false)
    .set_row_dim(-1);
  DeclareParameter("acts", &acts_,
                   "\"both\" (default), \"compression\" or \"tension\": the "
                   "sides on which the spring acts",
                   false);
  DeclareParameter("gap", &gap_,
                   "deflection before the spring acts, on each side that "
                   "acts; optional, default 0",
                   false);
  DeclareParameter("smoothing", &smoothing_,
                   "width of the edge of a stop, over which the force comes "
                   "in; optional, default 0.001",
                   false);
  DeclareParameter("unloading_stiffness", &unloading_stiffness_,
                   "makes the spring crush in compression and come back "
                   "along this slope, staying crushed; optional, default 0 "
                   "(it comes back along its curve)",
                   false);
  DeclareParameter("crush_time", &crush_time_,
                   "time for the crush to follow the compression; optional, "
                   "default 0.001",
                   false);
  DeclareParameter("hysteresis", &hysteresis_,
                   "Bouc-Wen hysteresis of the deflection (see "
                   "force_module::BoucWen); optional",
                   false);
}

void shift::core::force_module::SpringDamper::Connect() {
  const std::string prefix = "Spring damper " + get_name() + ": ";
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

ksl_SE3_t shift::core::force_module::SpringDamper::Pose(
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

void shift::core::force_module::SpringDamper::Load() {
  const std::string prefix = "Spring damper " + get_name();
  const double axis_length = ksl_vec3_l2norm(&axis_);
  torsion_ = axis_length > 0.0;
  if(torsion_) {
    ksl_vec3_scale(&axis_, 1.0 / axis_length);
  }
  if(!force_curve_values_.empty() && (stiffness_ != 0.0 || preload_ != 0.0)) {
    throw shift::core::Exception(prefix + ": give force_curve, or stiffness "
                                          "and preload, not both");
  }
  if(!damping_curve_values_.empty() && damping_ != 0.0) {
    throw shift::core::Exception(prefix + ": give damping_curve or damping, "
                                          "not both");
  }
  if(!force_curve_values_.empty()) {
    force_curve_ = Table::FromPairs(force_curve_values_,
                                    Table::Extrapolation::kLinear,
                                    prefix + ": force_curve");
  }
  if(!damping_curve_values_.empty()) {
    damping_curve_ = Table::FromPairs(damping_curve_values_,
                                      Table::Extrapolation::kLinear,
                                      prefix + ": damping_curve");
  }
  if(acts_ != "both" && acts_ != "compression" && acts_ != "tension") {
    throw shift::core::Exception(prefix + ": acts must be \"both\", "
                                          "\"compression\" or \"tension\", "
                                          "not \"" + acts_ + "\"");
  }
  if(gap_ < 0.0 || !(smoothing_ > 0.0)) {
    throw shift::core::Exception(prefix + ": gap must not be negative, and "
                                          "smoothing must be positive");
  }
  pulls_ = acts_ != "compression";
  pushes_ = acts_ != "tension";
  stop_ = gap_ > 0.0 || acts_ != "both";
  if(hysteresis_.is_used()) {
    hysteresis_.Check(prefix);
  }
  if(unloading_stiffness_ < 0.0 || !(crush_time_ > 0.0)) {
    throw shift::core::Exception(prefix + ": unloading_stiffness must not be "
                                          "negative, and crush_time must be "
                                          "positive");
  }
  for(auto& a : attachments_) {
    a.load_index = -2;
  }
  if(torsion_) {
    free_ = free_angle_;
  } else if(std::isnan(free_length_)) {
    const ksl_SE3_t di = Pose(attachments_[0]);
    const ksl_SE3_t dj = Pose(attachments_[1]);
    ksl_vec3_t d;
    ksl_subtract_vv(&dj.t, &di.t, &d);
    free_ = ksl_vec3_l2norm(&d);
  } else {
    free_ = free_length_;
  }
  equation_count_ = (torsion_ ? 1 : 0) + (hysteresis_.is_used() ? 1 : 0) +
                    (unloading_stiffness_ > 0.0 ? 2 : 0);
  q_0_.assign(equation_count_, 0.0);
}

double shift::core::force_module::SpringDamper::Force(double x, double v,
                                                      double z,
                                                      double crush) const {
  double deflection = x, weight = 1.0;
  if(stop_) {
    deflection = weight = 0.0;
    double value, slope;
    if(pulls_) {
      Ramp(x - gap_, smoothing_, value, slope);
      deflection += value;
      weight += slope;
    }
    if(pushes_) {
      Ramp(-x - gap_, smoothing_, value, slope);
      deflection -= value;
      weight += slope;
    }
    weight = std::min(weight, 1.0);
  }
  auto curve = [this](double e) {
    return force_curve_.empty() ? stiffness_ * e + preload_
                                : force_curve_.Evaluate(e);
  };
  double spring = curve(deflection);
  if(unloading_stiffness_ > 0.0 && crush > 0.0 && deflection > -crush) {
    /* back from the deepest compression along the unloading line, no
    further than zero and never outside the curve */
    spring = std::min(
      0.0, std::max(spring, curve(-crush) +
                              unloading_stiffness_ * (deflection + crush)));
  }
  const double damper =
    damping_curve_.empty() ? damping_ * v : damping_curve_.Evaluate(v);
  double force = weight * (spring + damper + hysteresis_.yield_force * z);
  if(stop_) {
    force = deflection > 0.0   ? std::max(force, 0.0)
            : deflection < 0.0 ? std::min(force, 0.0)
                               : 0.0;
  }
  return force;
}

void shift::core::force_module::SpringDamper::Compute(const double* y,
                                                      double* dy) {
  ksl_screw_t velocity[2];
  for(int k = 0; k < 2; k++) {
    Attachment& a = attachments_[k];
    if(a.load_index == -2) {
      a.load_index = context_->GetBodyLoadIndex(a.body);
    }
    poses_[k] = Pose(a);
    velocity[k] = context_->GetBodyVelocity(a.load_index);
  }
  int state = 0;
  if(torsion_) {
    ksl_product_drv(&poses_[0].R, &axis_, &direction_);
    ksl_vec3_t w;
    ksl_subtract_vv(&velocity[1].ang, &velocity[0].ang, &w);
    rate_ = ksl_dot_vv(&w, &direction_);
    length_ = y[state_offset_];
    if(dy) {
      dy[state_offset_] = rate_;
    }
    state = 1;
  } else {
    ksl_vec3_t d;
    ksl_subtract_vv(&poses_[1].t, &poses_[0].t, &d);
    length_ = ksl_vec3_l2norm(&d);
    /* the line has no direction with the points together: keep the last */
    if(length_ > 1e-12) {
      direction_ = d;
      ksl_vec3_scale(&direction_, 1.0 / length_);
    }
    /* the velocity of each point: v + w x p, the spatial velocities being
    referred to the fixed origin */
    ksl_vec3_t wi, wj;
    ksl_cross_vv(&velocity[0].ang, &poses_[0].t, &wi);
    ksl_cross_vv(&velocity[1].ang, &poses_[1].t, &wj);
    rate_ = 0.0;
    for(int c = 0; c < 3; c++) {
      rate_ += direction_.at[c] * ((velocity[1].lin.at[c] + wj.at[c]) -
                                   (velocity[0].lin.at[c] + wi.at[c]));
    }
  }
  double z = 0.0;
  if(hysteresis_.is_used()) {
    z = y[state_offset_ + state];
    if(dy) {
      dy[state_offset_ + state] = hysteresis_.Rate(z, rate_);
    }
  }
  crush_ = 0.0;
  if(unloading_stiffness_ > 0.0) {
    const int k = state + (hysteresis_.is_used() ? 1 : 0);
    crush_ = y[state_offset_ + k];
    absorbed_ = y[state_offset_ + k + 1];
  }
  const double x = length_ - free_;
  force_ = Force(x, rate_, z, crush_);
  if(dy && unloading_stiffness_ > 0.0) {
    const int k = state + (hysteresis_.is_used() ? 1 : 0);
    dy[state_offset_ + k] = std::max(0.0, -x - crush_) / crush_time_;
    /* the work of the two ends on the structure: the force times the rate */
    dy[state_offset_ + k + 1] = force_ * rate_;
  }
}

void shift::core::force_module::SpringDamper::Apply() {
  ksl_coscrew_t load;
  for(int k = 0; k < 2; k++) {
    /* on i along the direction, on j against it, as spatial forces in the
    fixed frame referred to its origin */
    const double f = k == 0 ? force_ : -force_;
    for(int c = 0; c < 3; c++) {
      (torsion_ ? load.ang : load.lin).at[c] = f * direction_.at[c];
      (torsion_ ? load.lin : load.ang).at[c] = 0.0;
    }
    if(!torsion_) {
      ksl_cross_vv(&poses_[k].t, &load.lin, &load.ang);
    }
    context_->AddBodyLoad(attachments_[k].load_index, load);
  }
}

void shift::core::force_module::SpringDamper::Service(const double* y,
                                                      double* dy) {
  Compute(y, dy);
  Apply();
}

void shift::core::force_module::SpringDamper::UpdateOutputs(double t,
                                                            const double* y) {
  (void)t;
  Compute(y, nullptr);
}

std::vector<std::string>
shift::core::force_module::SpringDamper::get_output_names() const {
  std::vector<std::string> names = {torsion_ ? "angle" : "length", "rate",
                                    torsion_ ? "moment" : "force"};
  if(unloading_stiffness_ > 0.0) {
    names.push_back("crush");
    names.push_back("absorbed");
  }
  return names;
}

void shift::core::force_module::SpringDamper::AppendOutputs(
  std::vector<double>& values) const {
  values.push_back(length_);
  values.push_back(rate_);
  values.push_back(force_);
  if(unloading_stiffness_ > 0.0) {
    values.push_back(crush_);
    values.push_back(absorbed_);
  }
}
