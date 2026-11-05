#include "shift/core/force_module/beam.h"

#include <cmath>

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::force_module::Beam::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "beam", shift::core::force_module::Beam::Create);

std::unique_ptr<shift::core::force_module::Beam>
shift::core::force_module::Beam::Create() {
  return std::make_unique<shift::core::force_module::Beam>();
}

void shift::core::force_module::Beam::DeclareBeamParameters() {
  DeclareParameter("young_modulus", &young_modulus_,
                   "Young's modulus of the material", true);
  DeclareParameter("shear_modulus", &shear_modulus_,
                   "shear modulus of the material; or poisson_ratio", false);
  DeclareParameter("poisson_ratio", &poisson_ratio_,
                   "Poisson's ratio, for the shear modulus E / (2 (1 + nu)); "
                   "or shear_modulus",
                   false);
  DeclareParameter("section", &section_,
                   "\"circle\", \"tube\", \"rectangle\" or \"general\" "
                   "(default)",
                   false);
  DeclareParameter("diameter", &diameter_,
                   "outer diameter of a circle or tube section", false);
  DeclareParameter("inner_diameter", &inner_diameter_,
                   "inner diameter of a tube section", false);
  DeclareParameter("width", &width_,
                   "width of a rectangle section, along the y axis", false);
  DeclareParameter("height", &height_,
                   "height of a rectangle section, along the z axis", false);
  DeclareParameter("area", &area_, "area of a general section", false);
  DeclareParameter("Iy", &Iy_,
                   "second moment of area of a general section about the y "
                   "axis",
                   false);
  DeclareParameter("Iz", &Iz_,
                   "second moment of area of a general section about the z "
                   "axis",
                   false);
  DeclareParameter("J", &J_, "torsion constant of a general section", false);
  DeclareParameter("damping", &beam_damping_,
                   "stiffness-proportional damping coefficient, seconds; "
                   "optional, default 0",
                   false);
}

void shift::core::force_module::Beam::Section(const std::string& prefix) {
  if(section_ == "circle" || section_ == "tube") {
    const double d = diameter_;
    const double di = section_ == "tube" ? inner_diameter_ : 0.0;
    if(!(d > 0.0) || !(di >= 0.0) || !(di < d)) {
      throw shift::core::Exception(prefix + "a " + section_ + " section "
                                   "needs diameter > inner_diameter >= 0");
    }
    area_ = M_PI / 4.0 * (d * d - di * di);
    Iy_ = Iz_ = M_PI / 64.0 * (std::pow(d, 4) - std::pow(di, 4));
    J_ = 2.0 * Iy_;
  } else if(section_ == "rectangle") {
    if(!(width_ > 0.0) || !(height_ > 0.0)) {
      throw shift::core::Exception(prefix + "a rectangle section needs width "
                                   "and height");
    }
    area_ = width_ * height_;
    Iy_ = width_ * std::pow(height_, 3) / 12.0;
    Iz_ = height_ * std::pow(width_, 3) / 12.0;
    /* torsion constant of a rectangle, long side a and short side b (Roark,
    Formulas for Stress and Strain, table 10.7) */
    const double a = std::max(width_, height_);
    const double b = std::min(width_, height_);
    J_ = a * b * b * b *
         (1.0 / 3.0 - 0.21 * b / a * (1.0 - std::pow(b / a, 4) / 12.0));
  } else if(section_ == "general") {
    if(!(area_ > 0.0) || !(Iy_ > 0.0) || !(Iz_ > 0.0) || !(J_ > 0.0)) {
      throw shift::core::Exception(prefix + "a general section needs "
                                   "positive area, Iy, Iz and J");
    }
  } else {
    throw shift::core::Exception(prefix + "section must be \"circle\", "
                                 "\"tube\", \"rectangle\" or \"general\", "
                                 "not \"" + section_ + "\"");
  }
}

void shift::core::force_module::Beam::Load() {
  const std::string prefix = "Beam " + get_name() + ": ";
  if(pairs_.size() != 1) {
    throw shift::core::Exception(prefix + "frames must name two frames, the "
                                 "ends of the beam");
  }
  if(!stiffness_values_.empty() || !damping_values_.empty() ||
     !stiffness_file_.empty() || !damping_file_.empty()) {
    throw shift::core::Exception(prefix + "the stiffness follows from the "
                                 "material and section; give damping as the "
                                 "coefficient \"damping\" only");
  }
  if(!(young_modulus_ > 0.0)) {
    throw shift::core::Exception(prefix + "young_modulus must be positive");
  }
  if(shear_modulus_ > 0.0) {
    G_ = shear_modulus_;
  } else if(poisson_ratio_ > -1.0 && poisson_ratio_ < 0.5) {
    G_ = young_modulus_ / (2.0 * (1.0 + poisson_ratio_));
  } else {
    throw shift::core::Exception(prefix + "give shear_modulus, or "
                                 "poisson_ratio between -1 and 0.5");
  }
  if(beam_damping_ < 0.0) {
    throw shift::core::Exception(prefix + "damping must not be negative");
  }
  Section(prefix);

  /* the rest pose of the pair, end B relative to end A as the model starts
  (the bushing's "initial" rest pose) */
  Bushing::Load();
  const ksl_SE3_t& rest = pairs_[0].rest;
  length_ = rest.t.x;
  const double off_axis = std::hypot(rest.t.y, rest.t.z);
  if(!(length_ > 0.0) || off_axis > 1e-6 * length_) {
    throw shift::core::Exception(
      prefix + "end B must lie on the positive x axis of end A as the model "
      "starts; it is at [" + std::to_string(rest.t.x) + ", " +
      std::to_string(rest.t.y) + ", " + std::to_string(rest.t.z) + "]");
  }

  /* the stiffness of the tip of a beam clamped at A, in A's axes */
  const double E = young_modulus_, L = length_;
  double K[6][6] = {};
  K[0][0] = E * area_ / L;
  K[1][1] = 12.0 * E * Iz_ / (L * L * L);
  K[1][5] = K[5][1] = -6.0 * E * Iz_ / (L * L);
  K[5][5] = 4.0 * E * Iz_ / L;
  K[2][2] = 12.0 * E * Iy_ / (L * L * L);
  K[2][4] = K[4][2] = 6.0 * E * Iy_ / (L * L);
  K[4][4] = 4.0 * E * Iy_ / L;
  K[3][3] = G_ * J_ / L;

  /* the bushing's coordinates are in the rest axes, those of B at rest:
  x_A = R x_rest in each block of three, so K_rest = R^T K_A R by blocks */
  auto R = [&rest](int r, int c) { return rest.R.v[c].at[r]; };
  for(int br = 0; br < 2; br++) {
    for(int bc = 0; bc < 2; bc++) {
      for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 3; c++) {
          double sum = 0.0;
          for(int k = 0; k < 3; k++) {
            for(int l = 0; l < 3; l++) {
              sum += R(k, r) * K[3 * br + k][3 * bc + l] * R(l, c);
            }
          }
          stiffness_[(3 * br + r) * 6 + 3 * bc + c] = sum;
          damping_[(3 * br + r) * 6 + 3 * bc + c] = beam_damping_ * sum;
        }
      }
    }
  }
}
