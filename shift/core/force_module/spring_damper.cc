#include "shift/core/force_module/spring_damper.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "ksl/mat3x3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/tangents.h"

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
                   "sides on which the spring acts", false,
                   {"both", "compression", "tension"});
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

  /* the outputs, positive in tension */
  DeclareOutput(torsion_ ? "angle" : "length", &length_,
                torsion_ ? "angle of the torsion spring" : "length");
  DeclareOutput("rate", &rate_, "rate of the length or the angle");
  DeclareOutput(torsion_ ? "moment" : "force", &force_,
                torsion_ ? "moment of the torsion spring" : "force");
  if(unloading_stiffness_ > 0.0) {
    DeclareOutput("crush", &crush_, "deflection crushed in");
    DeclareOutput("absorbed", &absorbed_,
                  "work the force has done on the crushed part");
  }
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

bool shift::core::force_module::SpringDamper::AddTangents(
  const double* y, Tangents& tangents) {
  assert(context_);
  const int body[2] = {attachments_[0].load_index, attachments_[1].load_index};
  if(body[0] == -2 || body[1] == -2) {
    return true; // not yet serviced
  }
  ksl_screw_t velocity[2];
  for(int k = 0; k < 2; k++) {
    velocity[k] = context_->GetBodyVelocity(body[k]);
  }

  /* the states, as Compute reads them */
  int state = torsion_ ? 1 : 0;
  const int angle_state = torsion_ ? static_cast<int>(state_offset_) : -1;
  const int hysteresis_state =
    hysteresis_.is_used() ? static_cast<int>(state_offset_) + state++ : -1;
  const int crush_state =
    unloading_stiffness_ > 0.0 ? static_cast<int>(state_offset_) + state : -1;
  const int absorbed_state = crush_state > -1 ? crush_state + 1 : -1;
  const double z = hysteresis_state > -1 ? y[hysteresis_state] : 0.0;

  /* the derivatives of the law with respect to the deflection and its rate */
  const double x = length_ - free_;
  const double v = rate_;
  const double f = force_;
  const double step_x =
    std::min(1e-7 * std::max(1.0, std::fabs(x)), 1e-3 * smoothing_);
  const double step_v = 1e-7 * std::max(1.0, std::fabs(v));
  const double f_x =
    (Force(x + step_x, v, z, crush_) - Force(x - step_x, v, z, crush_)) /
    (2.0 * step_x);
  const double f_v =
    (Force(x, v + step_v, z, crush_) - Force(x, v - step_v, z, crush_)) /
    (2.0 * step_v);

  /* the rates of the states, a_x dx + a_v dv */
  struct StateRate {
    int state;
    double a_x, a_v;
  };
  std::vector<StateRate> state_rates;
  if(angle_state > -1) {
    state_rates.push_back({angle_state, 0.0, 1.0});
  }
  if(hysteresis_state > -1) {
    const double rate_v =
      (hysteresis_.Rate(z, v + step_v) - hysteresis_.Rate(z, v - step_v)) /
      (2.0 * step_v);
    state_rates.push_back({hysteresis_state, 0.0, rate_v});
  }
  if(crush_state > -1) {
    /* about an axis the deflection is a state, whose terms the integrator
    differences */
    const double a_x =
      !torsion_ && -x - crush_ > 0.0 ? -1.0 / crush_time_ : 0.0;
    state_rates.push_back({crush_state, a_x, 0.0});
    state_rates.push_back(
      {absorbed_state, torsion_ ? 0.0 : f_x * v, f_v * v + f});
  }

  /* the blocks (i, i), (i, j), (j, j), (j, i), and the derivatives of x and
  v with respect to the displacement (x, v) and the twist (v) of each body */
  double stiffness[4][36] = {};
  double damping[4][36] = {};
  double x_row[2][6] = {}, v_row[2][6] = {}, v_twist_row[2][6] = {};
  const ksl_vec3_t& n = direction_;

  if(torsion_) {
    /* the moment f a on i and -f a on j, the axis a turning with body i:
    da = -[a x] dtheta_i, and dv = w . da for the relative angular velocity
    w */
    ksl_vec3_t w;
    ksl_subtract_vv(&velocity[1].ang, &velocity[0].ang, &w);
    const double a_cross[9] = {0.0,      -n.at[2], n.at[1], n.at[2], 0.0,
                               -n.at[0], -n.at[1], n.at[0], 0.0};
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++) {
        double h = 0.0;
        for(int m = 0; m < 3; m++) {
          h += ((r == m ? f : 0.0) + f_v * n.at[r] * w.at[m]) *
               -a_cross[m * 3 + c];
        }
        const double g = f_v * n.at[r] * n.at[c];
        const int e = (r + 3) * 6 + c + 3;
        stiffness[0][e] -= h;
        stiffness[3][e] += h;
        damping[0][e] += g;
        damping[1][e] -= g;
        damping[2][e] += g;
        damping[3][e] -= g;
      }
      double turn = 0.0;
      for(int m = 0; m < 3; m++) {
        turn += w.at[m] * -a_cross[m * 3 + r];
      }
      v_row[0][r + 3] = turn;
      v_twist_row[0][r + 3] = -n.at[r];
      v_twist_row[1][r + 3] = n.at[r];
    }
  } else {
    /* the force f n on point i and -f n on point j, which change with the
    displacement d of j relative to i */
    const ksl_vec3_t& p_i = poses_[0].t;
    const ksl_vec3_t& p_j = poses_[1].t;
    ksl_vec3_t wi, wj, u;
    ksl_cross_vv(&velocity[0].ang, &p_i, &wi);
    ksl_cross_vv(&velocity[1].ang, &p_j, &wj);
    for(int c = 0; c < 3; c++) {
      u.at[c] = (velocity[1].lin.at[c] + wj.at[c]) -
                (velocity[0].lin.at[c] + wi.at[c]);
    }
    const double l = std::max(length_, 1e-12);
    const double u_n = ksl_dot_vv(&u, &n);
    double pu_l[3], projection[9];
    for(int r = 0; r < 3; r++) {
      pu_l[r] = (u.at[r] - u_n * n.at[r]) / l;
      for(int c = 0; c < 3; c++) {
        projection[r * 3 + c] = (r == c ? 1.0 : 0.0) - n.at[r] * n.at[c];
      }
    }
    double g[9], gc[9];
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++) {
        g[r * 3 + c] = n.at[r] * (f_x * n.at[c] + f_v * pu_l[c]) +
                       f * projection[r * 3 + c] / l;
        gc[r * 3 + c] = f_v * n.at[r] * n.at[c];
      }
    }
    Tangents::AddPointBlock(p_i, g, p_i, 1.0, stiffness[0]);
    Tangents::AddPointBlock(p_i, g, p_j, -1.0, stiffness[1]);
    Tangents::AddPointBlock(p_j, g, p_j, 1.0, stiffness[2]);
    Tangents::AddPointBlock(p_j, g, p_i, -1.0, stiffness[3]);
    ksl_vec3_t force_i = n, force_j = n;
    ksl_vec3_scale(&force_i, f);
    ksl_vec3_scale(&force_j, -f);
    Tangents::AddMomentArmBlock(p_i, force_i, 1.0, stiffness[0]);
    Tangents::AddMomentArmBlock(p_j, force_j, 1.0, stiffness[2]);
    Tangents::AddPointBlock(p_i, gc, p_i, 1.0, damping[0]);
    Tangents::AddPointBlock(p_i, gc, p_j, -1.0, damping[1]);
    Tangents::AddPointBlock(p_j, gc, p_j, 1.0, damping[2]);
    Tangents::AddPointBlock(p_j, gc, p_i, -1.0, damping[3]);

    Tangents::AddPointRow(n.at, p_i, -1.0, x_row[0]);
    Tangents::AddPointRow(n.at, p_j, 1.0, x_row[1]);
    Tangents::AddPointRow(pu_l, p_i, -1.0, v_row[0]);
    Tangents::AddPointRow(pu_l, p_j, 1.0, v_row[1]);
    Tangents::AddPointRow(n.at, p_i, -1.0, v_twist_row[0]);
    Tangents::AddPointRow(n.at, p_j, 1.0, v_twist_row[1]);
  }

  const int pairs[4][2] = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  for(int k = 0; k < 4; k++) {
    tangents.AddBodyStiffness(body[pairs[k][0]], body[pairs[k][1]],
                              stiffness[k]);
    tangents.AddBodyDamping(body[pairs[k][0]], body[pairs[k][1]],
                            damping[k]);
  }
  for(const auto& rate : state_rates) {
    for(int k = 0; k < 2; k++) {
      double row_k[6], row_c[6];
      for(int c = 0; c < 6; c++) {
        row_k[c] = rate.a_x * x_row[k][c] + rate.a_v * v_row[k][c];
        row_c[c] = rate.a_v * v_twist_row[k][c];
      }
      tangents.AddStateBodyStiffness(rate.state, body[k], row_k);
      tangents.AddStateBodyDamping(rate.state, body[k], row_c);
    }
  }
  return true;
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
