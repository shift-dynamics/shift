#include "shift/core/force_module/journal_bearing.h"

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

bool shift::core::force_module::JournalBearing::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "journal_bearing", shift::core::force_module::JournalBearing::Create);

std::unique_ptr<shift::core::force_module::JournalBearing>
shift::core::force_module::JournalBearing::Create() {
  return std::make_unique<shift::core::force_module::JournalBearing>();
}

void shift::core::force_module::JournalBearing::DeclareParameters() {
  shift::core::force_module::Base::DeclareParameters();
  DeclareParameter("journal", &journal_names_,
                   "[body, frame] of the journal; the frame is at the center "
                   "of the journal, its z axis along the axis",
                   true)
    .set_row_dim(2);
  DeclareParameter("bearing", &bearing_names_,
                   "[body, frame] of the bearing; the frame is at the center "
                   "of the bearing, its z axis along the axis",
                   true)
    .set_row_dim(2);
  DeclareParameter("radius", &radius_, "radius of the journal", true);
  DeclareParameter("length", &length_, "length of the bearing", true);
  DeclareParameter("clearance", &clearance_,
                   "radial clearance (the bearing radius less the journal "
                   "radius)",
                   true);
  DeclareParameter("viscosity", &viscosity_, "dynamic viscosity of the oil",
                   true)
    .set_runtime();
  DeclareParameter("wall_stiffness", &wall_stiffness_,
                   "stiffness of the contact when the journal touches the "
                   "bearing; optional, default 0 (none)",
                   false)
    .set_runtime();
}

shift::core::force_module::JournalBearing::Attachment
shift::core::force_module::JournalBearing::Resolve(
  const std::vector<std::string>& names, const std::string& what) const {
  const std::string prefix = "Journal bearing " + get_name() + ": ";
  if(names.size() != 2) {
    throw shift::core::Exception(prefix + what + " must be [body, frame]");
  }
  Attachment a;
  a.body = context_->FindActiveBody(names[0]);
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

void shift::core::force_module::JournalBearing::Connect() {
  journal_ = Resolve(journal_names_, "journal");
  bearing_ = Resolve(bearing_names_, "bearing");
  if(journal_.body == bearing_.body) {
    throw shift::core::Exception("Journal bearing " + get_name() +
                                 ": the journal and the bearing must be on "
                                 "different bodies");
  }
}

ksl_SE3_t shift::core::force_module::JournalBearing::Pose(
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

void shift::core::force_module::JournalBearing::Load() {
  const std::string prefix = "Journal bearing " + get_name() + ": ";
  if(!(radius_ > 0.0) || !(length_ > 0.0) || !(clearance_ > 0.0) ||
     !(viscosity_ > 0.0) || wall_stiffness_ < 0.0) {
    throw shift::core::Exception(
      prefix + "radius, length, clearance and viscosity must be positive and "
               "wall_stiffness not negative");
  }
  if(clearance_ >= radius_) {
    throw shift::core::Exception(prefix + "the clearance must be much less "
                                 "than the radius");
  }
  journal_.load_index = bearing_.load_index = -2;
  equation_count_ = 0;
  q_0_.clear();
}

/* the nodes and weights of n point Gauss-Legendre quadrature on [-1, 1], by
Newton's method on the Legendre polynomial P_n from Chebyshev's estimates */
static void GaussLegendre(int n, std::vector<double>& nodes,
                          std::vector<double>& weights) {
  nodes.assign(n, 0.0);
  weights.assign(n, 0.0);
  for(int i = 0; i < n; i++) {
    double x = std::cos(M_PI * (i + 0.75) / (n + 0.5));
    double dp = 1.0;
    for(int iteration = 0; iteration < 100; iteration++) {
      double p0 = 1.0, p1 = x;
      for(int k = 2; k <= n; k++) {
        const double p2 = ((2.0 * k - 1.0) * x * p1 - (k - 1.0) * p0) / k;
        p0 = p1;
        p1 = p2;
      }
      dp = n * (x * p1 - p0) / (x * x - 1.0);
      const double step = p1 / dp;
      x -= step;
      if(std::fabs(step) < 1e-15) {
        break;
      }
    }
    nodes[i] = x;
    weights[i] = 2.0 / ((1.0 - x * x) * dp * dp);
  }
}

void shift::core::force_module::JournalBearing::FilmIntegrals(
  double epsilon, double s, double q, double* i_1, double* i_2) {
  *i_1 = *i_2 = 0.0;
  if(s == 0.0 && q == 0.0) {
    return;
  }
  static std::vector<double> nodes, weights;
  if(nodes.empty()) {
    GaussLegendre(32, nodes, weights);
  }
  /* the film is where s sin(theta) - q cos(theta) = Q sin(theta - theta_0)
  is positive: half a turn from theta_0 */
  const double theta_0 = std::atan2(q, s);
  /* Sommerfeld's substitution cos(psi) = (epsilon + cos(theta)) / (1 +
  epsilon cos(theta)), which makes the integrands smooth even where the film
  is thin */
  const double root = std::sqrt(1.0 - epsilon * epsilon);
  auto psi_of = [&](double theta) {
    return std::atan2(root * std::sin(theta), epsilon + std::cos(theta));
  };
  double psi_a = psi_of(theta_0);
  double psi_b = psi_of(theta_0 + M_PI);
  if(!(psi_b > psi_a)) {
    psi_b += 2.0 * M_PI;
  }
  const double half = 0.5 * (psi_b - psi_a), mid = 0.5 * (psi_b + psi_a);
  double sum_1 = 0.0, sum_2 = 0.0;
  for(size_t k = 0; k < nodes.size(); k++) {
    const double psi = mid + half * nodes[k];
    const double denominator = 1.0 - epsilon * std::cos(psi);
    const double cos_theta = (std::cos(psi) - epsilon) / denominator;
    const double sin_theta = root * std::sin(psi) / denominator;
    /* 1 / (1 + epsilon cos(theta))^3 and dtheta / dpsi */
    const double film = denominator / (1.0 - epsilon * epsilon);
    const double weight = weights[k] * half * root / denominator;
    const double g = (s * sin_theta - q * cos_theta) * film * film * film;
    sum_1 += weight * g * cos_theta;
    sum_2 += weight * g * sin_theta;
  }
  *i_1 = sum_1;
  *i_2 = sum_2;
}

void shift::core::force_module::JournalBearing::Service(const double*,
                                                        double*) {
  Compute(true);
}

void shift::core::force_module::JournalBearing::UpdateOutputs(double,
                                                              const double*) {
  Compute(false);
}

void shift::core::force_module::JournalBearing::Compute(bool apply) {
  Attachment* ends[2] = {&journal_, &bearing_};
  for(Attachment* a : ends) {
    if(a->load_index == -2) {
      a->load_index = context_->GetBodyLoadIndex(a->body);
    }
  }
  const ksl_SE3_t dj = Pose(journal_), db = Pose(bearing_);
  const ksl_screw_t sj = context_->GetBodyVelocity(journal_.load_index);
  const ksl_screw_t sb = context_->GetBodyVelocity(bearing_.load_index);
  const ksl_vec3_t axis = db.R.v2;
  const double c = clearance_, r = radius_, l = length_, mu = viscosity_;

  /* the eccentricity, the velocity of the journal center relative to the
  bearing's, and the rotation of each about the axis (spatial velocities are
  referred to the fixed origin, v(p) = v + w x p) */
  ksl_vec3_t e, wj_p, wb_p, de;
  ksl_subtract_vv(&dj.t, &db.t, &e);
  ksl_axpy_vv(-ksl_dot_vv(&e, &axis), &axis, &e);
  ksl_cross_vv(&sj.ang, &dj.t, &wj_p);
  ksl_cross_vv(&sb.ang, &db.t, &wb_p);
  for(int i = 0; i < 3; i++) {
    de.at[i] = (sj.lin.at[i] + wj_p.at[i]) - (sb.lin.at[i] + wb_p.at[i]);
  }
  ksl_axpy_vv(-ksl_dot_vv(&de, &axis), &axis, &de);
  const double omega_j = ksl_dot_vv(&sj.ang, &axis);
  const double omega_b = ksl_dot_vv(&sb.ang, &axis);

  /* u_1 along the line of centers, u_2 across it (a x u_1) */
  const double size = ksl_vec3_l2norm(&e);
  ksl_vec3_t u_1;
  if(size > 1e-9 * c) {
    u_1 = e;
    ksl_vec3_scale(&u_1, 1.0 / size);
  } else {
    /* concentric: any direction across the axis (the force does not depend
    on it) */
    const ksl_vec3_t trial = std::fabs(axis.x) < 0.9
                               ? ksl_vec3(1.0, 0.0, 0.0)
                               : ksl_vec3(0.0, 1.0, 0.0);
    u_1 = trial;
    ksl_axpy_vv(-ksl_dot_vv(&u_1, &axis), &axis, &u_1);
    ksl_vec3_scale(&u_1, 1.0 / ksl_vec3_l2norm(&u_1));
  }
  ksl_vec3_t u_2;
  ksl_cross_vv(&axis, &u_1, &u_2);
  const double rate_1 = ksl_dot_vv(&de, &u_1); // along the line of centers
  const double rate_2 = ksl_dot_vv(&de, &u_2); // across it

  /* the eccentricity ratio of the film: the true one to 0.9, then smoothly
  towards 0.995 */
  const double epsilon = size / c;
  const double knee = 0.9, top = 0.995;
  const double eps_film =
    epsilon <= knee ? epsilon
                    : knee + (top - knee) * std::tanh((epsilon - knee) /
                                                      (top - knee));
  const double s = eps_film * (omega_j + omega_b) - 2.0 * rate_2 / c;
  const double q = 2.0 * rate_1 / c;

  double i_1, i_2;
  FilmIntegrals(eps_film, s, q, &i_1, &i_2);
  const double k = mu * r * l * l * l / (2.0 * c * c);
  radial_force_ = k * i_1;
  tangential_force_ = k * i_2;
  for(int i = 0; i < 3; i++) {
    force_.at[i] = radial_force_ * u_1.at[i] + tangential_force_ * u_2.at[i];
  }

  /* contact of the journal with the bearing */
  if(wall_stiffness_ > 0.0 && size > c) {
    const double over = size - c;
    const double w = 0.01 * c;
    const double depth = over < w ? over * over / (2.0 * w) : over - 0.5 * w;
    ksl_axpy_vv(-wall_stiffness_ * depth, &u_1, &force_);
  }

  /* the friction torque on the journal about the axis: the shear of the oil
  round the whole bearing, and the work of the pressure force */
  const double root = std::sqrt(1.0 - eps_film * eps_film);
  const double couette =
    -2.0 * M_PI * mu * r * r * r * l * (omega_j - omega_b) / (c * root);
  const double pressure_part = -0.5 * c * eps_film * tangential_force_;
  friction_torque_ = couette + pressure_part;

  /* outputs: the thickness of the film, the greatest pressure over the
  middle of the bearing, the flow out of its ends */
  eccentricity_ratio_ = epsilon;
  min_film_thickness_ = c * (1.0 - epsilon);
  const double strength = std::sqrt(s * s + q * q);
  const double theta_0 = std::atan2(q, s);
  double peak = 0.0;
  for(int i = 1; i < 90; i++) {
    const double theta = theta_0 + M_PI * i / 90.0;
    const double film = 1.0 + eps_film * std::cos(theta);
    peak = std::max(peak, std::sin(theta - theta_0) / (film * film * film));
  }
  peak_pressure_ = 3.0 * mu * (0.25 * l * l) * strength * peak / (c * c);
  side_flow_ = r * c * l * strength;
  load_ = ksl_vec3_l2norm(&force_);

  if(!apply) {
    return;
  }
  /* the force and torque on the journal at its center, and the opposite on the
  bearing, as spatial forces in the fixed frame referred to its origin */
  ksl_coscrew_t load;
  for(int i = 0; i < 3; i++) {
    load.lin.at[i] = force_.at[i];
    load.ang.at[i] = friction_torque_ * axis.at[i];
  }
  ksl_vec3_t p_cross_f;
  ksl_cross_vv(&dj.t, &load.lin, &p_cross_f);
  for(int i = 0; i < 3; i++) {
    load.ang.at[i] += p_cross_f.at[i];
  }
  context_->AddBodyLoad(journal_.load_index, load);
  for(int i = 0; i < 6; i++) {
    load.at[i] = -load.at[i];
  }
  context_->AddBodyLoad(bearing_.load_index, load);
}

std::vector<std::string>
shift::core::force_module::JournalBearing::get_output_names() const {
  return {"eccentricity_ratio", "min_film_thickness", "peak_pressure",
          "friction_torque",    "side_flow",          "radial_force",
          "tangential_force",   "load"};
}

void shift::core::force_module::JournalBearing::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(),
                {eccentricity_ratio_, min_film_thickness_, peak_pressure_,
                 friction_torque_, side_flow_, radial_force_,
                 tangential_force_, load_});
}
