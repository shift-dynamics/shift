#include "shift/core/force_module/soil.h"

#include <cmath>
#include <limits>

#include "shift/core/exception.h"

double shift::core::force_module::SoilProperties::Pressure(double b,
                                                           double z) const {
  if(!(z > 0.0)) {
    return 0.0;
  }
  return (k_c / b + k_phi) * std::pow(z, n);
}

/* s of Coulomb's coefficients; see SoilProperties */
static double CoulombRoot(double friction_angle, double wall_friction) {
  return std::sqrt(std::sin(friction_angle + wall_friction) *
                   std::sin(friction_angle) / std::cos(wall_friction));
}

double shift::core::force_module::SoilProperties::PassiveCoefficient() const {
  if(passive_coefficient > 0.0) {
    return passive_coefficient;
  }
  const double c = std::cos(friction_angle);
  const double d = 1.0 - CoulombRoot(friction_angle, wall_friction);
  return c * c / (d * d);
}

double shift::core::force_module::SoilProperties::ActiveCoefficient() const {
  const double c = std::cos(friction_angle);
  const double d = 1.0 + CoulombRoot(friction_angle, wall_friction);
  return c * c / (d * d);
}

double shift::core::force_module::SoilProperties::AtRestCoefficient() const {
  return 1.0 - std::sin(friction_angle);
}

double shift::core::force_module::SoilProperties::PassiveForce(double h) const {
  if(!(h > 0.0)) {
    return 0.0;
  }
  const double kp = PassiveCoefficient();
  return 0.5 * unit_weight * h * h * kp + 2.0 * cohesion * h * std::sqrt(kp);
}

struct SoilRow {
  const char* name;
  double n, k_c, k_phi, cohesion, friction_angle, shear_modulus;
};

/* n, k_c (kN/m^(n+1)), k_phi (kN/m^(n+2)), c (kPa), phi (deg), K (m; 0 for
the default); see LookupSoil */
static const SoilRow kSoils[] = {
  {"dry_sand", 1.1, 0.99, 1528.43, 1.04, 28.0, 0.0},
  {"sandy_loam", 0.7, 5.27, 1515.04, 1.72, 29.0, 0.0},
  {"clayey_soil", 0.5, 13.19, 692.15, 4.14, 13.0, 0.0},
  {"clayey_soil_wet", 0.7, 16.03, 1262.53, 2.07, 10.0, 0.0},
  {"heavy_clay", 0.13, 12.70, 1555.95, 69.95, 34.0, 0.0},
  {"heavy_clay_wet", 0.11, 1.84, 103.27, 20.69, 6.0, 0.0},
  {"lean_clay", 0.2, 16.43, 1724.69, 68.95, 20.0, 0.0},
  {"lean_clay_wet", 0.15, 1.52, 119.61, 13.79, 11.0, 0.0},
  {"lete_sand", 0.79, 102.0, 5301.0, 1.3, 31.1, 0.0},
  {"upland_sandy_loam", 1.10, 74.6, 2080.0, 3.3, 33.7, 0.0},
  {"rubicon_sandy_loam", 0.66, 6.9, 752.0, 3.7, 29.8, 0.0},
  {"north_gower_clayey_loam", 0.73, 41.6, 2471.0, 6.1, 26.6, 0.0},
  {"grenville_loam", 1.01, 0.06, 5880.0, 3.1, 29.8, 0.0},
  {"snow", 1.6, 4.37, 196.72, 1.03, 19.7, 0.0},
  {"tri1_lunar_simulant", 0.404, 6.60, 139.00, 0.36, 43.33, 0.0103},
};

bool shift::core::force_module::LookupSoil(const std::string& name,
                                           SoilProperties& soil) {
  for(const auto& row : kSoils) {
    if(name == row.name) {
      soil = SoilProperties();
      soil.n = row.n;
      soil.k_c = 1e3 * row.k_c;
      soil.k_phi = 1e3 * row.k_phi;
      soil.cohesion = 1e3 * row.cohesion;
      soil.friction_angle = row.friction_angle * M_PI / 180.0;
      if(row.shear_modulus > 0.0) {
        soil.shear_modulus = row.shear_modulus;
      }
      return true;
    }
  }
  return false;
}

std::vector<std::string> shift::core::force_module::SoilNames() {
  std::vector<std::string> names;
  for(const auto& row : kSoils) {
    names.push_back(row.name);
  }
  return names;
}

void shift::core::force_module::Soil::DeclareParameters() {
  const double unset = std::numeric_limits<double>::quiet_NaN();
  n_ = k_c_ = k_phi_ = cohesion_ = friction_angle_ = shear_modulus_ =
    unit_weight_ = wall_friction_ = passive_coefficient_ = youngs_modulus_ =
      poisson_ratio_ = dilatancy_angle_ = max_unit_weight_ = unset;
  DeclareParameter("name", &name_,
                   "a soil of the table (see LookupSoil), whose properties "
                   "the others override; without it every Bekker and "
                   "Mohr-Coulomb property must be given",
                   false);
  DeclareParameter("sinkage_exponent", &n_, "Bekker's n", false);
  DeclareParameter("cohesive_modulus", &k_c_, "Bekker's k_c, N/m^(n+1)",
                   false);
  DeclareParameter("frictional_modulus", &k_phi_,
                   "Bekker's k_phi, N/m^(n+2)", false);
  DeclareParameter("cohesion", &cohesion_, "Pa", false);
  DeclareParameter("friction_angle", &friction_angle_,
                   "internal friction angle, radians", false);
  DeclareParameter("shear_modulus", &shear_modulus_,
                   "Janosi-Hanamoto shear deformation modulus K, m; "
                   "defaults to 0.01",
                   false);
  DeclareParameter("unit_weight", &unit_weight_,
                   "weight per volume, N/m^3, for earth pressure; defaults "
                   "to 16000",
                   false);
  DeclareParameter("max_unit_weight", &max_unit_weight_,
                   "weight per volume of the soil compacted as far as bodies "
                   "compact it, N/m^3, at least unit_weight: under a body "
                   "the soil first compacts, by up to 1 - unit_weight / "
                   "max_unit_weight of the layer that compacts, before it "
                   "is pushed aside (terrain::Bulldozing); without it the "
                   "soil does not compact",
                   false);
  DeclareParameter("wall_friction", &wall_friction_,
                   "friction angle between the soil and the face it presses "
                   "on, radians, from 0 to the friction_angle; with 0, the "
                   "default, the earth pressure is Rankine's",
                   false);
  DeclareParameter("passive_coefficient", &passive_coefficient_,
                   "passive earth-pressure coefficient, normal to the face, "
                   "at least 1, from a test or a table; replaces Coulomb's "
                   "from the angles",
                   false);
  DeclareParameter("youngs_modulus", &youngs_modulus_,
                   "Young's modulus of the soil as a continuum, Pa, for the "
                   "SPH soil (terrain type sph)",
                   false);
  DeclareParameter("poisson_ratio", &poisson_ratio_,
                   "Poisson's ratio of the soil as a continuum, from 0 to "
                   "0.5, for the SPH soil",
                   false);
  DeclareParameter("dilatancy_angle", &dilatancy_angle_,
                   "angle of dilatancy, radians, from 0 to the "
                   "friction_angle, for the SPH soil; 0 for soil whose "
                   "volume does not change as it shears",
                   false);
}

bool shift::core::force_module::Soil::is_given() const {
  if(!name_.empty()) {
    return true;
  }
  for(double value : {n_, k_c_, k_phi_, cohesion_, friction_angle_,
                      shear_modulus_, unit_weight_, wall_friction_,
                      passive_coefficient_, youngs_modulus_, poisson_ratio_,
                      dilatancy_angle_, max_unit_weight_}) {
    if(!std::isnan(value)) {
      return true;
    }
  }
  return false;
}

shift::core::force_module::SoilProperties
shift::core::force_module::Soil::Resolve(const std::string& owner,
                                         bool bekker) const {
  const std::string prefix = owner + ": soil: ";
  SoilProperties soil;
  const bool named = !name_.empty();
  if(named && !LookupSoil(name_, soil)) {
    std::string names;
    for(const auto& n : SoilNames()) {
      names += (names.empty() ? "" : ", ") + n;
    }
    throw shift::core::Exception(prefix + "'" + name_ +
                                 "' is not in the table (" + names + ")");
  }
  auto take = [&](double given, double& value, const char* key,
                  bool needed) {
    if(!std::isnan(given)) {
      value = given;
    } else if(needed && !named) {
      throw shift::core::Exception(prefix + "needs " + key +
                                   " or the name of a soil of the table");
    }
  };
  take(n_, soil.n, "sinkage_exponent", bekker);
  take(k_c_, soil.k_c, "cohesive_modulus", bekker);
  take(k_phi_, soil.k_phi, "frictional_modulus", bekker);
  take(cohesion_, soil.cohesion, "cohesion", true);
  take(friction_angle_, soil.friction_angle, "friction_angle", true);
  take(shear_modulus_, soil.shear_modulus, "shear_modulus", false);
  take(unit_weight_, soil.unit_weight, "unit_weight", false);
  take(wall_friction_, soil.wall_friction, "wall_friction", false);
  take(passive_coefficient_, soil.passive_coefficient, "passive_coefficient",
       false);
  take(youngs_modulus_, soil.youngs_modulus, "youngs_modulus", false);
  take(poisson_ratio_, soil.poisson_ratio, "poisson_ratio", false);
  take(dilatancy_angle_, soil.dilatancy_angle, "dilatancy_angle", false);
  take(max_unit_weight_, soil.max_unit_weight, "max_unit_weight", false);
  Check(soil, prefix, bekker);
  return soil;
}

void shift::core::force_module::Soil::Override(SoilProperties& soil,
                                               const std::string& owner) const {
  const std::string prefix = owner + ": soil: ";
  if(!name_.empty()) {
    const SoilProperties base = soil;
    if(!LookupSoil(name_, soil)) {
      throw shift::core::Exception(prefix + "'" + name_ +
                                   "' is not in the table");
    }
    /* the properties of a continuum are not in the table */
    soil.youngs_modulus = base.youngs_modulus;
    soil.poisson_ratio = base.poisson_ratio;
    soil.dilatancy_angle = base.dilatancy_angle;
    soil.max_unit_weight = base.max_unit_weight;
  }
  auto take = [](double given, double& value) {
    if(!std::isnan(given)) {
      value = given;
    }
  };
  take(n_, soil.n);
  take(k_c_, soil.k_c);
  take(k_phi_, soil.k_phi);
  take(cohesion_, soil.cohesion);
  take(friction_angle_, soil.friction_angle);
  take(shear_modulus_, soil.shear_modulus);
  take(unit_weight_, soil.unit_weight);
  take(wall_friction_, soil.wall_friction);
  take(passive_coefficient_, soil.passive_coefficient);
  take(youngs_modulus_, soil.youngs_modulus);
  take(poisson_ratio_, soil.poisson_ratio);
  take(dilatancy_angle_, soil.dilatancy_angle);
  take(max_unit_weight_, soil.max_unit_weight);
  Check(soil, prefix, soil.k_c + soil.k_phi > 0.0);
}

void shift::core::force_module::Soil::Check(const SoilProperties& soil,
                                            const std::string& prefix,
                                            bool bekker) const {
  if((bekker && (!(soil.n > 0.0) || soil.k_c < 0.0 || soil.k_phi < 0.0 ||
                 !(soil.k_c + soil.k_phi > 0.0))) ||
     soil.cohesion < 0.0 || soil.friction_angle < 0.0 ||
     !(soil.friction_angle < 0.5 * M_PI) || !(soil.shear_modulus > 0.0) ||
     soil.unit_weight < 0.0) {
    throw shift::core::Exception(prefix + "a property is out of range");
  }
  if((!std::isnan(soil.youngs_modulus) && !(soil.youngs_modulus > 0.0)) ||
     (!std::isnan(soil.poisson_ratio) &&
      !(soil.poisson_ratio >= 0.0 && soil.poisson_ratio < 0.5)) ||
     (!std::isnan(soil.dilatancy_angle) &&
      !(soil.dilatancy_angle >= 0.0 &&
        soil.dilatancy_angle <= soil.friction_angle))) {
    throw shift::core::Exception(prefix + "youngs_modulus must be positive, "
                                          "poisson_ratio from 0 to 0.5 and "
                                          "dilatancy_angle from 0 to the "
                                          "friction_angle");
  }
  if(!std::isnan(soil.max_unit_weight) &&
     !(soil.max_unit_weight >= soil.unit_weight && soil.unit_weight > 0.0)) {
    throw shift::core::Exception(prefix + "max_unit_weight must be at least "
                                          "unit_weight, and unit_weight "
                                          "positive");
  }
  if(soil.wall_friction < 0.0 || soil.wall_friction > soil.friction_angle) {
    throw shift::core::Exception(prefix + "wall_friction must be from 0 to "
                                          "the friction_angle");
  }
  if(!std::isnan(passive_coefficient_) && !(passive_coefficient_ >= 1.0)) {
    throw shift::core::Exception(prefix + "passive_coefficient must be at "
                                          "least 1");
  }
  if(!(soil.passive_coefficient > 0.0) &&
     !(CoulombRoot(soil.friction_angle, soil.wall_friction) < 1.0)) {
    throw shift::core::Exception(prefix + "Coulomb's passive pressure has no "
                                          "finite value at these angles: "
                                          "lower wall_friction or give "
                                          "passive_coefficient");
  }
}
