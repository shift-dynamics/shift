#include "shift/core/force_module/belt.h"

#include <algorithm>
#include <cmath>

#include "ksl/coscrew.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"

bool shift::core::force_module::Belt::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "belt", shift::core::force_module::Belt::Create);

std::unique_ptr<shift::core::force_module::Belt>
shift::core::force_module::Belt::Create() {
  return std::make_unique<shift::core::force_module::Belt>();
}

void shift::core::force_module::Belt::DeclareParameters() {
  DeclareParameter("frames", &frame_names_,
                   "[body, frame] of each pulley, two or more, "
                   "counterclockwise about the first pulley's z axis",
                   true)
    .set_row_dim(-1)
    .set_column_dim(2);
  DeclareParameter("radii", &radii_,
                   "radius of each pulley at the belt's pitch line; "
                   "negative for a pulley outside the loop",
                   true)
    .set_row_dim(-1);
  DeclareParameter("axial_stiffness", &axial_stiffness_,
                   "EA of the belt, force per unit strain", true);
  DeclareParameter("damping", &damping_,
                   "stiffness-proportional damping of the stretch, seconds; "
                   "optional, default 0",
                   false);
  DeclareParameter("pretension", &pretension_,
                   "tension of every span as the model starts; optional, "
                   "default 0",
                   false);
  DeclareParameter("friction_coefficient", &friction_coefficient_,
                   "friction coefficient of the belt on each pulley; "
                   "optional, without it the belt does not slip; the "
                   "values can be changed as the model runs (not to "
                   "negative values)",
                   false)
    .set_row_dim(-1)
    .set_runtime();
  DeclareParameter("slip_damping", &slip_damping_,
                   "with friction_coefficient: force difference per unit "
                   "slip speed beyond the friction limit",
                   false);
  DeclareParameter("slack_band", &slack_band_,
                   "force over which a span changes from slack to taut; "
                   "optional, default 1e-6 axial_stiffness",
                   false);
}

void shift::core::force_module::Belt::Connect() {
  const std::string prefix = "Belt " + get_name() + ": ";
  if(frame_names_.size() < 2) {
    throw shift::core::Exception(prefix + "frames must name two or more "
                                 "pulleys");
  }
  pulleys_.clear();
  for(const auto& names : frame_names_) {
    if(names.size() != 2) {
      throw shift::core::Exception(prefix + "each of frames must be "
                                   "[body, frame]");
    }
    Pulley pulley;
    pulley.body = context_->FindActiveBody(names[0]);
    if(!pulley.body) {
      throw shift::core::Exception(prefix + "body " + names[0] +
                                   " not found among the active bodies");
    }
    for(auto& frame : pulley.body->get_frames()) {
      if(frame->get_name() == names[1] ||
         (names[1] == "origin" && frame->is_origin())) {
        pulley.frame = frame.get();
      }
    }
    if(!pulley.frame) {
      throw shift::core::Exception(prefix + "frame " + names[1] +
                                   " not found on body " + names[0]);
    }
    pulleys_.push_back(pulley);
  }
}

ksl_SE3_t shift::core::force_module::Belt::Pose(const Pulley& pulley) const {
  if(pulley.body == context_->get_fixed_body()) {
    return *pulley.frame->get_d_wrt_origin();
  }
  const ksl_SE3_t origin =
    context_->get_body_origin_pose(pulley.body, context_->get_dhva());
  ksl_SE3_t pose;
  ksl_product_dd(&origin, pulley.frame->get_d_wrt_origin(), &pose);
  return pose;
}

double shift::core::force_module::Belt::Ramp(double x, double band) {
  if(x <= 0.0) {
    return 0.0;
  }
  if(x < band) {
    /* f(0) = f'(0) = 0, f(band) = band, f'(band) = 1 */
    const double s = x / band;
    return x * s * (2.0 - s);
  }
  return x;
}

void shift::core::force_module::Belt::Load() {
  const std::string prefix = "Belt " + get_name() + ": ";
  const size_t n = pulleys_.size();
  if(radii_.size() != n) {
    throw shift::core::Exception(prefix + "radii needs one radius for each "
                                 "of the " + std::to_string(n) + " pulleys");
  }
  for(size_t k = 0; k < n; k++) {
    if(!(std::fabs(radii_[k]) > 0.0)) {
      throw shift::core::Exception(prefix + "each radius must be non-zero");
    }
    pulleys_[k].radius = radii_[k];
  }
  if(!(axial_stiffness_ > 0.0)) {
    throw shift::core::Exception(prefix + "axial_stiffness must be "
                                 "positive");
  }
  if(damping_ < 0.0 || pretension_ < 0.0) {
    throw shift::core::Exception(prefix + "damping and pretension must not "
                                 "be negative");
  }
  if(slack_band_ < 0.0) {
    slack_band_ = 1e-6 * axial_stiffness_;
  }
  if(!friction_coefficient_.empty()) {
    if(friction_coefficient_.size() != n) {
      throw shift::core::Exception(prefix + "friction_coefficient needs one "
                                   "value for each of the " +
                                   std::to_string(n) + " pulleys");
    }
    for(double mu : friction_coefficient_) {
      if(mu < 0.0) {
        throw shift::core::Exception(prefix + "friction_coefficient must "
                                     "not be negative");
      }
    }
    if(!(slip_damping_ > 0.0)) {
      throw shift::core::Exception(prefix + "friction_coefficient needs a "
                                   "positive slip_damping");
    }
  }

  angle_0_.assign(n, 0.0);
  offset_.assign(n, 0.0);
  angle_.assign(n, 0.0);
  length_.assign(n, 0.0);
  stretch_.assign(n, 0.0);
  stretch_rate_.assign(n, 0.0);
  tension_.assign(n, 0.0);
  spring_tension_.assign(n, 0.0);
  wrap_.assign(n, 0.0);
  slip_.assign(n, 0.0);
  direction_.assign(n, ksl_vec3_t{});
  start_point_.assign(n, ksl_vec3_t{});
  end_point_.assign(n, ksl_vec3_t{});

  /* the turn of each pulley, then the belt slipped over each */
  equation_count_ = static_cast<unsigned int>(slips() ? 2 * n : n);
  q_0_.assign(equation_count_, 0.0);

  /* the offsets give each span the stretch of the pretension at the start,
  T = EA e / (L - e) */
  Evaluate(q_0_.data(), nullptr, true);
  for(size_t k = 0; k < n; k++) {
    const double e =
      pretension_ * length_[k] / (axial_stiffness_ + pretension_);
    offset_[k] = stretch_[k] - e;
  }
  for(size_t k = 0; k < n; k++) {
    stretch_[k] -= offset_[k];
  }

  /* listed clockwise, the tangents would cross the loop */
  if(n >= 3) {
    double area = 0.0;
    for(size_t k = 0; k < n; k++) {
      const ksl_vec3_t& a = pulleys_[k].pose.t;
      const ksl_vec3_t& b = pulleys_[(k + 1) % n].pose.t;
      const ksl_vec3_t& z = pulleys_[0].pose.R.v[2];
      const ksl_vec3_t ab = {{b.x - a.x, b.y - a.y, b.z - a.z}};
      const ksl_vec3_t a0 = {{a.x - pulleys_[0].pose.t.x,
                              a.y - pulleys_[0].pose.t.y,
                              a.z - pulleys_[0].pose.t.z}};
      area += z.x * (a0.y * ab.z - a0.z * ab.y) +
              z.y * (a0.z * ab.x - a0.x * ab.z) +
              z.z * (a0.x * ab.y - a0.y * ab.x);
    }
    if(!(area > 0.0)) {
      throw shift::core::Exception(prefix + "list the pulleys "
                                   "counterclockwise about the z axis of the "
                                   "first pulley's frame");
    }
  }
}

void shift::core::force_module::Belt::Evaluate(const double* states,
                                               double* rates, bool initial) {
  const std::string prefix = "Belt " + get_name() + ": ";
  const size_t n = pulleys_.size();
  for(auto& pulley : pulleys_) {
    if(pulley.load_index == -2) {
      pulley.load_index = context_->GetBodyLoadIndex(pulley.body);
    }
    pulley.pose = Pose(pulley);
    pulley.velocity = context_->GetBodyVelocity(pulley.load_index);
  }
  auto dot = [](const ksl_vec3_t& a, const ksl_vec3_t& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  };
  auto cross = [](const ksl_vec3_t& a, const ksl_vec3_t& b) {
    return ksl_vec3_t{{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                       a.x * b.y - a.y * b.x}};
  };
  /* velocity of the point p moving with a pulley's body */
  auto point_velocity = [&cross](const Pulley& pulley, const ksl_vec3_t& p) {
    const ksl_vec3_t w_p = cross(pulley.velocity.ang, p);
    return ksl_vec3_t{{pulley.velocity.lin.x + w_p.x,
                       pulley.velocity.lin.y + w_p.y,
                       pulley.velocity.lin.z + w_p.z}};
  };

  /* the belt's plane: normal z along the first pulley's axis, x along the
  line from the first centre to the second, projected */
  const ksl_vec3_t z = pulleys_[0].pose.R.v[2];
  const ksl_vec3_t& c0 = pulleys_[0].pose.t;
  auto in_plane = [&](const ksl_vec3_t& c) {
    const ksl_vec3_t d = {{c.x - c0.x, c.y - c0.y, c.z - c0.z}};
    const double h = dot(d, z);
    return ksl_vec3_t{{d.x - h * z.x, d.y - h * z.y, d.z - h * z.z}};
  };
  const ksl_vec3_t d01 = in_plane(pulleys_[1].pose.t);
  const double d01_squared = dot(d01, d01);
  if(!(d01_squared > 0.0)) {
    throw shift::core::Exception(prefix + "the centres of the first two "
                                 "pulleys coincide in the belt's plane");
  }
  const double d01_length = std::sqrt(d01_squared);
  const ksl_vec3_t x = {
    {d01.x / d01_length, d01.y / d01_length, d01.z / d01_length}};
  const ksl_vec3_t yaxis = cross(z, x);
  /* the turn rate of that line about z */
  const ksl_vec3_t v0 = point_velocity(pulleys_[0], c0);
  const ksl_vec3_t v1 = point_velocity(pulleys_[1], pulleys_[1].pose.t);
  const ksl_vec3_t dv01 = {{v1.x - v0.x, v1.y - v0.y, v1.z - v0.z}};
  const double line_rate = dot(z, cross(d01, dv01)) / d01_squared;

  /* each span k from pulley a = k to b = k + 1 lies along the common
  tangent: the tangent points are c + r u for the unit normal u of the span,
  with d.u = r_a - r_b, and the belt travels along t = z x u */
  for(size_t k = 0; k < n; k++) {
    const Pulley& a = pulleys_[k];
    const Pulley& b = pulleys_[(k + 1) % n];
    const ksl_vec3_t pa = in_plane(a.pose.t);
    const ksl_vec3_t pb = in_plane(b.pose.t);
    const double dx = dot(pb, x) - dot(pa, x);
    const double dy = dot(pb, yaxis) - dot(pa, yaxis);
    const double distance = std::hypot(dx, dy);
    const double cos_g = (a.radius - b.radius) / distance;
    if(!(std::fabs(cos_g) < 1.0)) {
      throw shift::core::Exception(
        prefix + "pulleys " + std::to_string(k) + " and " +
        std::to_string((k + 1) % n) + " overlap; the belt has no span "
        "between them");
    }
    const double sin_g = -std::sqrt(1.0 - cos_g * cos_g);
    const double ex = dx / distance, ey = dy / distance;
    /* u = cos_g d + sin_g (z x d), in the plane's axes */
    const double ux = cos_g * ex - sin_g * ey;
    const double uy = cos_g * ey + sin_g * ex;
    double angle = std::atan2(uy, ux);
    if(initial) {
      angle_0_[k] = angle;
    } else {
      angle = angle_0_[k] + std::remainder(angle - angle_0_[k], 2.0 * M_PI);
    }
    angle_[k] = angle;
    length_[k] = -distance * sin_g;

    const ksl_vec3_t u = {{ux * x.x + uy * yaxis.x, ux * x.y + uy * yaxis.y,
                           ux * x.z + uy * yaxis.z}};
    direction_[k] = cross(z, u);
    for(int c = 0; c < 3; c++) {
      start_point_[k].at[c] = a.pose.t.at[c] + a.radius * u.at[c];
      end_point_[k].at[c] = b.pose.t.at[c] + b.radius * u.at[c];
    }
  }

  /* the stretch: the length less the belt between the tangent points, the
  belt on a pulley at angle alpha of the normal being r (alpha - theta) less
  the belt slipped forward over it, up to a constant */
  const double* turn = states;
  const double* slipped = slips() ? states + n : nullptr;
  for(size_t k = 0; k < n; k++) {
    const size_t a = k, b = (k + 1) % n;
    const double ra = pulleys_[a].radius, rb = pulleys_[b].radius;
    double e = length_[k] + (ra - rb) * angle_[k] - ra * turn[a] +
               rb * turn[b] - offset_[k];
    if(slipped) {
      e += slipped[b] - slipped[a];
    }
    stretch_[k] = e;
  }

  /* wrap angles, from the normal of the span arriving to that leaving */
  for(size_t k = 0; k < n; k++) {
    const double in = angle_[(k + n - 1) % n], out = angle_[k];
    const double turned = pulleys_[k].radius > 0.0 ? out - in : in - out;
    wrap_[k] = turned - 2.0 * M_PI * std::floor(turned / (2.0 * M_PI));
  }

  /* spring tensions, then the slip they drive */
  const double ea = axial_stiffness_;
  for(size_t k = 0; k < n; k++) {
    const double l0 = length_[k] - stretch_[k];
    spring_tension_[k] = Ramp(ea * stretch_[k] / l0, slack_band_);
  }
  std::fill(slip_.begin(), slip_.end(), 0.0);
  if(slips()) {
    for(size_t k = 0; k < n; k++) {
      const double t_in = spring_tension_[(k + n - 1) % n];
      const double t_out = spring_tension_[k];
      const double difference = t_out - t_in;
      const double limit =
        std::min(t_in, t_out) *
        (std::exp(friction_coefficient_[k] * wrap_[k]) - 1.0);
      const double excess = Ramp(std::fabs(difference) - limit, slack_band_);
      slip_[k] = std::copysign(excess, difference) / slip_damping_;
    }
  }

  /* stretch rates from the speeds of the belt at the tangent points, the
  speed of the pulley's rim plus the slip, along the span */
  for(size_t k = 0; k < n; k++) {
    const size_t a = k, b = (k + 1) % n;
    const ksl_vec3_t va = point_velocity(pulleys_[a], start_point_[k]);
    const ksl_vec3_t vb = point_velocity(pulleys_[b], end_point_[k]);
    const ksl_vec3_t dv = {{vb.x - va.x, vb.y - va.y, vb.z - va.z}};
    stretch_rate_[k] = dot(direction_[k], dv) + slip_[b] - slip_[a];
    const double l0 = length_[k] - stretch_[k];
    tension_[k] = Ramp(ea * (stretch_[k] + damping_ * stretch_rate_[k]) / l0,
                       slack_band_);
  }

  if(rates) {
    for(size_t k = 0; k < n; k++) {
      rates[k] = dot(pulleys_[k].velocity.ang, z) - line_rate;
      if(slips()) {
        rates[n + k] = slip_[k];
      }
    }
  }
}

void shift::core::force_module::Belt::Service(const double* y, double* dy) {
  Evaluate(y + state_offset_, dy ? dy + state_offset_ : nullptr);
  /* each span pulls the pulleys at its ends toward each other along it, at
  its tangent points: as spatial forces in the fixed frame referred to its
  origin */
  const size_t n = pulleys_.size();
  for(size_t k = 0; k < n; k++) {
    const double t = tension_[k];
    if(t == 0.0) {
      continue;
    }
    for(int end = 0; end < 2; end++) {
      const Pulley& pulley = pulleys_[end == 0 ? k : (k + 1) % n];
      const ksl_vec3_t& p = end == 0 ? start_point_[k] : end_point_[k];
      const double sign = end == 0 ? t : -t;
      ksl_coscrew_t load;
      for(int c = 0; c < 3; c++) {
        load.lin.at[c] = sign * direction_[k].at[c];
      }
      load.ang.x = p.y * load.lin.z - p.z * load.lin.y;
      load.ang.y = p.z * load.lin.x - p.x * load.lin.z;
      load.ang.z = p.x * load.lin.y - p.y * load.lin.x;
      context_->AddBodyLoad(pulley.load_index, load);
    }
  }
}

void shift::core::force_module::Belt::UpdateOutputs(double t,
                                                    const double* y) {
  (void)t;
  Evaluate(y + state_offset_, nullptr);
}

std::vector<std::string>
shift::core::force_module::Belt::get_output_names() const {
  std::vector<std::string> names;
  for(size_t k = 0; k < pulleys_.size(); k++) {
    names.push_back("span_" + std::to_string(k) + "_tension");
  }
  if(slips()) {
    for(size_t k = 0; k < pulleys_.size(); k++) {
      names.push_back("pulley_" + std::to_string(k) + "_slip");
    }
  }
  return names;
}

void shift::core::force_module::Belt::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), tension_.begin(), tension_.end());
  if(slips()) {
    values.insert(values.end(), slip_.begin(), slip_.end());
  }
}
