#include "shift/core/force_module/tire/bekker_wong.h"

#include <algorithm>
#include <cmath>

#include "ksl/vec3.h"
#include "shift/core/exception.h"

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

/* 0 below 0, 1 above 1, and a cubic between with no slope at either end */
static double SmoothStep(double t) {
  if(!(t > 0.0)) {
    return 0.0;
  }
  if(t >= 1.0) {
    return 1.0;
  }
  return t * t * (3.0 - 2.0 * t);
}

/* a x + b y + c z */
static ksl_vec3_t Combination(double a, const ksl_vec3_t& x, double b,
                              const ksl_vec3_t& y, double c,
                              const ksl_vec3_t& z) {
  return {{a * x.x + b * y.x + c * z.x, a * x.y + b * y.y + c * z.y,
           a * x.z + b * y.z + c * z.z}};
}

void shift::core::force_module::tire::BekkerWong::DeclareParameters() {
  DeclareParameter("unloaded_radius", &radius,
                   "unloaded radius (m); optional, default 0.344", false);
  DeclareParameter("width", &width, "tire width (m); optional, default 0.235",
                   false);
  DeclareParameter("soil", &soil_, "the soil (see force_module::Soil)", true);
  DeclareParameter("soil_damping", &vertical_damping,
                   "damping of the load on the normal velocity of the contact "
                   "point; optional, default 0",
                   false);
  DeclareParameter("max_stress_angle", &c1,
                   "c1 of the angle of greatest normal stress theta_m = (c1 + "
                   "c2 slip) theta_1; optional, default 0.4",
                   false);
  DeclareParameter("max_stress_slip", &c2,
                   "c2 of theta_m; optional, default 0.3", false);
  DeclareParameter("rebound", &rebound,
                   "rear sinkage over the front sinkage; optional, default 0",
                   false);
  DeclareParameter("grouser_height", &grouser_height,
                   "height of the lugs, for the shear; optional, default 0",
                   false);
  DeclareParameter("stations", &stations,
                   "quadrature points in each of the two zones of the rim; "
                   "optional, default 16",
                   false);
  DeclareParameter("strips", &strips,
                   "quadrature points across the width of a cambered wheel; "
                   "optional, default 5",
                   false);
  DeclareParameter("bulldozing", &bulldozing,
                   "the passive pressure of the soil on the sides of the "
                   "wheel, where they push it; optional, default true",
                   false);
  DeclareParameter("slip_speed", &slip_speed,
                   "speed that smooths the slip at standstill; optional, "
                   "default 0.01",
                   false);
}

void shift::core::force_module::tire::BekkerWong::Prepare() {
  if(!(radius > 0.0) || !(width > 0.0) || stations < 2 ||
     strips < 1 || rebound < 0.0 || !(rebound <= 1.0) ||
     grouser_height < 0.0 || !(slip_speed > 0.0) || c1 < 0.0 ||
     c2 < 0.0 || !(c1 + c2 <= 1.0)) {
    throw shift::core::Exception(
      "Bekker-Wong tire: radius, width and slip_speed must be positive, "
      "stations at least 2, strips at least 1, rebound in [0, 1], "
      "grouser_height not negative and c1, c2 not negative with c1 + c2 at "
      "most 1");
  }
  GaussLegendre(stations, nodes_, weights_);
  GaussLegendre(strips, strip_nodes_, strip_weights_);
  GaussLegendre(6, side_nodes_, side_weights_);
}

double shift::core::force_module::tire::BekkerWong::NormalStress(
  double theta) const {
  if(!(theta_1_ > 0.0) || theta > theta_1_ || theta < theta_2_) {
    return 0.0;
  }
  /* behind the angle of greatest stress, the front's mirrored */
  if(theta < theta_m_) {
    theta = theta_1_ - (theta - theta_2_) / (theta_m_ - theta_2_) *
                         (theta_1_ - theta_m_);
  }
  return soil.Pressure(
    width, effective_radius_ * (std::cos(theta) - std::cos(theta_1_)));
}

void shift::core::force_module::tire::BekkerWong::Evaluate(
  const Contact& contact, const double*, double*, ksl_coscrew_t* force_w) {
  force_w->lin = {{0.0, 0.0, 0.0}};
  force_w->ang = {{0.0, 0.0, 0.0}};
  theta_1_ = theta_2_ = theta_m_ = slip_ = 0.0;
  shear_torque_ = camber_ = bulldozing_force_ = 0.0;
  effective_radius_ = radius;
  if(!contact.in_contact) {
    return;
  }
  const double r = radius, b = width;

  /* the frame of the rim, (f, a, u), in the w frame (x heading, z the
  terrain normal into the terrain): the axle, the direction in the wheel's
  plane towards the terrain, and the heading */
  ksl_vec3_t a = contact.spin_axis_w;
  ksl_vec3_scale(&a, 1.0 / ksl_vec3_l2norm(&a));
  const double sin_gamma = std::min(std::max(a.z, -1.0), 1.0);
  const double cos2 = 1.0 - sin_gamma * sin_gamma;
  if(!(cos2 > 1e-4)) {
    return; // a wheel lying flat
  }
  const double cos_gamma = std::sqrt(cos2);
  const ksl_vec3_t u = {{-sin_gamma * a.x / cos_gamma,
                         -sin_gamma * a.y / cos_gamma,
                         (1.0 - sin_gamma * a.z) / cos_gamma}};
  ksl_vec3_t f;
  ksl_cross_vv(&a, &u, &f);
  camber_ = std::asin(sin_gamma);
  const double r_eff = r * cos_gamma;
  effective_radius_ = r_eff;
  const double depth = contact.depth;

  /* the strips of the width in the soil: H(y) = depth + y sin(gamma) > 0 */
  double y_lo = -0.5 * b, y_hi = 0.5 * b;
  if(sin_gamma > 1e-12) {
    y_lo = std::max(y_lo, -depth / sin_gamma);
  } else if(sin_gamma < -1e-12) {
    y_hi = std::min(y_hi, -depth / sin_gamma);
  } else if(!(depth > 0.0)) {
    return;
  }
  if(!(y_hi > y_lo)) {
    return;
  }

  /* the angles of the strip at the middle of the width */
  {
    const double h0 = std::min(std::max(depth, 0.0), r_eff);
    theta_1_ = std::acos(1.0 - h0 / r_eff);
    theta_2_ = -std::acos(1.0 - rebound * h0 / r_eff);
  }

  /* velocities in the frame of the rim */
  auto in_rim = [&](const ksl_vec3_t& v, double& vf, double& va, double& vu) {
    vf = ksl_dot_vv(&v, &f);
    va = ksl_dot_vv(&v, &a);
    vu = ksl_dot_vv(&v, &u);
  };
  double vf, va, vu, wf, wa, wu;
  in_rim(contact.hub_velocity_w, vf, va, vu);
  in_rim(contact.angular_velocity_w, wf, wa, wu);
  /* the speed of the rim about the hub (positive rolling forward) and of
  the hub: the larger sets the time the soil spends under the tread, and
  gives the slip */
  const double v_rim = -wa * r;
  const double v_max = std::max(std::fabs(v_rim), std::fabs(vf));
  const double speed =
    std::sqrt(v_max * v_max + slip_speed * slip_speed);
  slip_ = (v_rim - vf) / speed;
  const double slip_clamped = std::min(std::max(slip_, 0.0), 1.0);
  theta_m_ = (c1 + c2 * slip_clamped) * theta_1_;

  /* from the contact point (the w frame's origin) to the hub */
  const ksl_vec3_t hub = {{-r * u.x, -r * u.y, depth - r * u.z}};
  const double tan_phi = std::tan(soil.friction_angle);

  ksl_vec3_t force = {{0.0, 0.0, 0.0}}, moment = {{0.0, 0.0, 0.0}};
  double torque = 0.0;
  auto add = [&](const ksl_vec3_t& at, const ksl_vec3_t& df) {
    ksl_vec3_t dm;
    ksl_cross_vv(&at, &df, &dm);
    for(int i = 0; i < 3; i++) {
      force.at[i] += df.at[i];
      moment.at[i] += dm.at[i];
    }
  };

  /* the tread, by strips of the width */
  const bool cambered = std::fabs(sin_gamma) > 1e-9;
  const int strip_count = cambered ? strips : 1;
  const double y_half = 0.5 * (y_hi - y_lo), y_mid = 0.5 * (y_hi + y_lo);
  for(int j = 0; j < strip_count; j++) {
    const double y = y_mid + y_half * (cambered ? strip_nodes_[j] : 0.0);
    const double y_weight = y_half * (cambered ? strip_weights_[j] : 2.0);
    const double h = std::min(depth + y * sin_gamma, r_eff);
    if(!(h > 0.0)) {
      continue;
    }
    const double theta_1 = std::acos(1.0 - h / r_eff);
    const double theta_2 = -std::acos(1.0 - rebound * h / r_eff);
    const double theta_m = (c1 + c2 * slip_clamped) * theta_1;
    /* the shear starts where the grousers' tips enter the soil */
    const double theta_s = std::acos(
      std::min(1.0, (r_eff - h) / (r_eff + grouser_height * cos_gamma)));
    const double sin_s = std::sin(theta_s), cos_s = std::cos(theta_s);
    const ksl_vec3_t offset = Combination(y, a, 0.0, a, 0.0, a);

    auto zone = [&](double lo, double hi) {
      if(!(hi > lo)) {
        return;
      }
      const double half = 0.5 * (hi - lo), mid = 0.5 * (hi + lo);
      for(size_t k = 0; k < nodes_.size(); k++) {
        const double theta = mid + half * nodes_[k];
        const double s = std::sin(theta), c = std::cos(theta);
        /* the normal stress, at the depth z of the tread */
        double theta_eq = theta;
        if(theta < theta_m) {
          theta_eq = theta_1 - (theta - theta_2) / (theta_m - theta_2) *
                                 (theta_1 - theta_m);
        }
        const double sigma =
          soil.Pressure(b, r_eff * (std::cos(theta_eq) - std::cos(theta_1)));
        /* the sliding of the tread over the soil, and the displacement */
        const double s_t = (vf - wu * y) * c - (vu + wf * y) * s + wa * r;
        const double s_l = va + wu * r * s - wf * r * c;
        const double j_t =
          r / speed * ((vf - wu * y) * (sin_s - s) +
                       (vu + wf * y) * (cos_s - c) + wa * r * (theta_s - theta));
        const double j_l =
          r / speed * (va * (theta_s - theta) + wu * r * (c - cos_s) -
                       wf * r * (sin_s - s));
        const double slide = std::hypot(s_t, s_l);
        const double disp = std::hypot(j_t, j_l);
        double along = 0.0, across = 0.0;
        if(slide > 0.0 && disp > 0.0) {
          const double tau = (soil.cohesion + sigma * tan_phi) *
                             -std::expm1(-disp / soil.shear_modulus);
          along = -tau * s_t / slide;
          across = -tau * s_l / slide;
        }
        const double area = y_weight * half * weights_[k] * r;
        const ksl_vec3_t e_r = Combination(s, f, c, u, 0.0, a);
        const ksl_vec3_t e_t = Combination(c, f, -s, u, 0.0, a);
        const ksl_vec3_t df = Combination(-sigma * area, e_r, along * area,
                                          e_t, across * area, a);
        add(Combination(1.0, hub, r, e_r, 1.0, offset), df);
        torque += r * area * along;
      }
    };
    zone(theta_m, theta_1);
    zone(theta_2, theta_m);
  }
  shear_torque_ = torque;

  /* the sides of the tread: the passive pressure of the soil they push */
  if(bulldozing) {
    const double kp = soil.PassiveCoefficient();
    const double edge = 2.0 * soil.cohesion * std::sqrt(kp);
    for(int side = -1; side <= 1; side += 2) {
      const double y = 0.5 * side * b;
      const double h = std::min(depth + y * sin_gamma, r_eff);
      if(!(h > 0.0)) {
        continue;
      }
      /* the circular segment the terrain cuts off the side: from the
      radius at which the side meets the terrain at its lowest, to r */
      const double r_in = r - h / cos_gamma;
      const double r_half = 0.5 * (r - r_in), r_mid = 0.5 * (r + r_in);
      for(size_t i = 0; i < side_nodes_.size(); i++) {
        const double rs = r_mid + r_half * side_nodes_[i];
        if(!(rs > 0.0)) {
          continue;
        }
        const double theta_max = std::acos(std::min(1.0, r_in / rs));
        for(size_t k = 0; k < side_nodes_.size(); k++) {
          const double theta = theta_max * side_nodes_[k];
          const ksl_vec3_t e_r =
            Combination(std::sin(theta), f, std::cos(theta), u, 0.0, a);
          const ksl_vec3_t at = Combination(1.0, hub, rs, e_r, y, a);
          const double depth_below = at.z;
          if(!(depth_below > 0.0)) {
            continue;
          }
          /* the velocity of the point of the side along its outward
          normal: the wheel pushes the soil if it is positive */
          const ksl_vec3_t arm = Combination(rs, e_r, y, a, 0.0, a);
          ksl_vec3_t spin;
          ksl_cross_vv(&contact.angular_velocity_w, &arm, &spin);
          const double outward =
            side * (ksl_dot_vv(&contact.hub_velocity_w, &a) +
                    ksl_dot_vv(&spin, &a));
          const double mobilized = SmoothStep(outward / slip_speed);
          if(!(mobilized > 0.0)) {
            continue;
          }
          const double pressure = edge + soil.unit_weight * depth_below * kp;
          const double area = r_half * side_weights_[i] * theta_max *
                              side_weights_[k] * rs;
          const double push = -side * mobilized * pressure * area;
          bulldozing_force_ += push;
          add(at, Combination(push, a, 0.0, a, 0.0, a));
        }
      }
    }
  }

  /* the load, damped and clipped: the soil only pushes; the damping acts at
  the contact point, it has no moment about it */
  const double fz = std::min(
    force.z - vertical_damping * contact.contact_velocity_w.z, 0.0);
  force_w->lin = {{force.x, force.y, fz}};
  force_w->ang = moment;
}
