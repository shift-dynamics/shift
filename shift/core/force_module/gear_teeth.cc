#include "shift/core/force_module/gear_teeth.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "shift/core/exception.h"

void shift::core::force_module::GearTeeth::DeclareParameters() {
  DeclareParameter("youngs_modulus", &youngs_modulus,
                   "Young's modulus of the gears", true);
  DeclareParameter("poisson_ratio", &poisson_ratio,
                   "Poisson's ratio of the gears; optional, default 0.3",
                   false);
  DeclareParameter("addendum", &addendum,
                   "height of the tip above the pitch circle, in modules; "
                   "optional, default 1",
                   false);
  DeclareParameter("dedendum", &dedendum,
                   "depth of the root below the pitch circle, in modules; "
                   "optional, default 1.25",
                   false);
  DeclareParameter("hub_ratio", &hub_ratio,
                   "radius of the hub over that of the root circle, from 1/7 "
                   "to 1/1.4; optional, default 0.5",
                   false);
  DeclareParameter("points", &points,
                   "points of the table of the stiffness over a mesh; "
                   "optional, default 200",
                   false);
}

void shift::core::force_module::GearTeeth::Configure(
  double radius_1, double radius_2, int teeth_1, int teeth_2,
  double pressure_angle, double module, bool internal) {
  const std::string prefix = "Gear teeth: ";
  if(!(youngs_modulus > 0.0) || !(poisson_ratio > -1.0) ||
     !(poisson_ratio < 0.5) || !(addendum > 0.0) || !(dedendum > 0.0) ||
     !(hub_ratio >= 1.0 / 7.0) || !(hub_ratio <= 1.0 / 1.4) || points < 8) {
    throw shift::core::Exception(
      prefix + "youngs_modulus, addendum and dedendum must be positive, "
               "poisson_ratio below 0.5, hub_ratio from 1/7 to 1/1.4 and "
               "points at least 8");
  }
  internal_ = internal;
  phi_ = pressure_angle;
  const double c = std::cos(phi_), s = std::sin(phi_);
  gear_1_ = {radius_1 * c, radius_1 - dedendum * module, teeth_1};
  gear_2_ = {radius_2 * c, radius_2 - dedendum * module, teeth_2};
  base_pitch_ = 2.0 * M_PI * gear_1_.base / teeth_1;

  auto reach = [](double tip, double base) {
    return std::sqrt(std::max(tip * tip - base * base, 0.0));
  };
  const double tip_1 = radius_1 + addendum * module;
  const double end_1 = reach(tip_1, gear_1_.base);
  double start_1;
  if(internal_) {
    const double tip_2 = radius_2 - addendum * module;
    if(!(tip_2 > gear_2_.base)) {
      throw shift::core::Exception(
        prefix + "the tips of the ring are inside its base circle");
    }
    const double offset = (radius_2 - radius_1) * s;
    start_1 = reach(tip_2, gear_2_.base) - offset;
    ring_tip_ = reach(radius_2 + addendum * module, gear_2_.base);
  } else {
    const double tip_2 = radius_2 + addendum * module;
    const double offset = (radius_1 + radius_2) * s;
    start_1 = offset - reach(tip_2, gear_2_.base);
    rho_sum_ = offset;
  }
  if(!(start_1 > 0.0)) {
    throw shift::core::Exception(
      prefix + "the teeth interfere: contact begins inside the base circle "
               "of gear 1");
  }
  rho_start_ = start_1;
  contact_ratio_ = (end_1 - start_1) / base_pitch_;
  if(!(contact_ratio_ >= 1.0)) {
    throw shift::core::Exception(
      prefix + "the contact ratio is " + std::to_string(contact_ratio_) +
      ", below 1: the teeth lose contact");
  }
}

double shift::core::force_module::GearTeeth::ToothCompliance(const Gear& gear,
                                                             double rho) const {
  const double nu = poisson_ratio;
  const double alpha_2 =
    M_PI / (2.0 * gear.teeth) + std::tan(phi_) - phi_;
  const double alpha_1 = rho / gear.base - alpha_2;
  auto g = [&](double a) {
    return std::sin(a) + (alpha_2 - a) * std::cos(a);
  };

  /* where the tooth starts: the base circle, or the root circle above it,
  found by bisection on the radius of the involute's section */
  auto radius = [&](double a) {
    return gear.base * (std::cos(a) + (alpha_2 - a) * std::sin(a));
  };
  double alpha_r = alpha_2;
  if(gear.root > gear.base) {
    double low = -alpha_1, high = alpha_2;
    for(int i = 0; i < 60; i++) {
      const double middle = 0.5 * (low + high);
      (radius(middle) > gear.root ? low : high) = middle;
    }
    alpha_r = 0.5 * (low + high);
  }

  /* bending, shear and axial compression, by Simpson's rule */
  const double c_1 = std::cos(alpha_1), s_1 = std::sin(alpha_1);
  auto integrand = [&](double a) {
    const double arm = (alpha_2 - a) * std::cos(a);
    const double section = g(a);
    const double bend = 1.0 + c_1 * ((alpha_2 - a) * std::sin(a) - std::cos(a));
    return 3.0 * bend * bend * arm / (2.0 * section * section * section) +
           6.0 * (1.0 + nu) * arm * c_1 * c_1 / (5.0 * section) +
           arm * s_1 * s_1 / (2.0 * section);
  };
  const int n = 128;
  const double step = (alpha_r + alpha_1) / n;
  double sum = integrand(-alpha_1) + integrand(alpha_r);
  for(int i = 1; i < n; i++) {
    sum += (i % 2 ? 4.0 : 2.0) * integrand(-alpha_1 + i * step);
  }
  const double tooth = sum * step / 3.0;

  /* the fillet foundation; the coefficients are those of Sainsot, Velex and
  Duverger, as reprinted by Marafona (2023, table 2.1) */
  const double half_thickness = gear.base * g(alpha_r);
  const double theta = std::asin(std::min(half_thickness / gear.root, 1.0));
  const double h = 1.0 / hub_ratio;
  auto fit = [&](double a, double b, double c, double d, double e, double f) {
    return a / (theta * theta) + b * h * h + c * h / theta + d / theta + e * h +
           f;
  };
  const double l = fit(-5.574e-5, -1.9986e-3, -2.3015e-4, 4.7702e-3, 0.0271,
                       6.8045);
  const double m = fit(60.111e-5, 28.100e-3, -83.431e-4, -9.9256e-3, 0.1624,
                       0.9086);
  const double p = fit(-50.952e-5, 185.50e-3, 0.0538e-4, 53.300e-3, 0.2895,
                       0.9236);
  const double q = fit(-6.2042e-5, 9.0889e-3, -4.0964e-4, 7.8297e-3, -0.1472,
                       0.6904);
  const double u = gear.base / c_1 - gear.root * std::cos(theta);
  const double ratio = u / (2.0 * gear.root * theta);
  const double tangent = s_1 / c_1;
  const double fillet =
    c_1 * c_1 *
    (l * ratio * ratio + m * ratio + p * (1.0 + q * tangent * tangent));
  return tooth + fillet;
}

double shift::core::force_module::GearTeeth::PairStiffness(double s,
                                                           double width) const {
  const double rho_1 = rho_start_ + s * base_pitch_;
  const double hertz = 4.0 * (1.0 - poisson_ratio * poisson_ratio) / M_PI;
  double rho_2;
  if(internal_) {
    /* the ring's tooth as an external one: its tip, at the smallest rho_2, is
    at the tip of the external tooth */
    rho_2 = ring_tip_ - (rho_1 - rho_start_);
  } else {
    rho_2 = rho_sum_ - rho_1;
  }
  const double compliance =
    (hertz + ToothCompliance(gear_1_, rho_1) + ToothCompliance(gear_2_, rho_2)) /
    (youngs_modulus * width);
  return 1.0 / compliance;
}

double shift::core::force_module::GearTeeth::MeshStiffness(double x,
                                                           double width) const {
  /* the pairs in contact are a base pitch apart, placed so that x = 0 has them
  equally far from the two ends of the path */
  const double f = x - std::floor(x);
  const double first = 0.5 * contact_ratio_ + 0.5 + f;
  double k = 0.0;
  for(double s = first - std::floor(first); s <= contact_ratio_; s += 1.0) {
    k += PairStiffness(s, width);
  }
  return k;
}
