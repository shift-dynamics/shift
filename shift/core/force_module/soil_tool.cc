#include "shift/core/force_module/soil_tool.h"

#include <algorithm>
#include <cmath>

#include "ksl/mat3x3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::force_module::SoilTool::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "soil_tool", shift::core::force_module::SoilTool::Create);

std::unique_ptr<shift::core::force_module::SoilTool>
shift::core::force_module::SoilTool::Create() {
  return std::make_unique<shift::core::force_module::SoilTool>();
}

void shift::core::force_module::SoilTool::DeclareParameters() {
  DeclareParameter("soil", &soil_,
                   "the soil (see force_module::Soil); its wall_friction is "
                   "the friction angle between the soil and the blade",
                   true);
  DeclareParameter("body", &body_name_, "the body carrying the blade", true);
  DeclareParameter("tip", &tip_,
                   "the middle of the cutting edge, in the body's coordinates",
                   true)
    .set_runtime();
  DeclareParameter("face", &face_,
                   "from the edge up the blade, in the body's coordinates; "
                   "defaults to [0, 0, -1]",
                   false);
  DeclareParameter("edge", &edge_,
                   "along the cutting edge, in the body's coordinates; "
                   "defaults to [0, 1, 0]",
                   false);
  DeclareParameter("front", &front_,
                   "the direction the blade works in, in the body's "
                   "coordinates; defaults to [1, 0, 0]",
                   false);
  DeclareParameter("width", &width_, "along the edge", true);
  DeclareParameter("height", &height_, "along the face", true);
  DeclareParameter("ground", &ground_,
                   "the ground's z (z down, soil below); defaults to 0", false)
    .set_runtime();
  DeclareParameter("adhesion", &adhesion_,
                   "of the soil to the blade, Pa; defaults to 0", false);
  DeclareParameter("slip_speed", &slip_speed_,
                   "speed over which the force builds up; defaults to 0.01",
                   false);
  DeclareParameter("spill_time", &spill_time_,
                   "time for the soil beyond a full pile to spill, and for "
                   "the pile to be left behind; defaults to 1",
                   false);
}

void shift::core::force_module::SoilTool::Connect() {}

static ksl_vec3_t Unit(ksl_vec3_t v, const std::string& what) {
  const double length = ksl_vec3_l2norm(&v);
  if(!(length > 0.0)) {
    throw shift::core::Exception(what + " must not be zero");
  }
  ksl_vec3_scale(&v, 1.0 / length);
  return v;
}

void shift::core::force_module::SoilTool::Load() {
  const std::string prefix = "Soil tool force module '" + name_ + "'";
  soil_properties_ = soil_.Resolve(prefix);
  if(!(width_ > 0.0) || !(height_ > 0.0) || adhesion_ < 0.0 ||
     !(slip_speed_ > 0.0) || !(spill_time_ > 0.0)) {
    throw shift::core::Exception(prefix + ": width, height, slip_speed and "
                                          "spill_time must be positive, and "
                                          "adhesion not negative");
  }
  if(!(soil_properties_.friction_angle > 0.0)) {
    throw shift::core::Exception(prefix + ": needs a soil with a friction "
                                          "angle, for the pile");
  }
  face_ = Unit(face_, prefix + ": face");
  edge_ = Unit(edge_, prefix + ": edge");
  front_ = Unit(front_, prefix + ": front");
  body_ = context_->FindActiveBody(body_name_, get_parent_assembly());
  if(!body_) {
    throw shift::core::Exception(prefix + ": body '" + body_name_ +
                                 "' is not an active body");
  }
  load_index_ = -2;
  equation_count_ = 1;
  q_0_.assign(1, 0.0);
}

/* n_w at the failure angle b */
static double WeightFactor(double b, double rake, double phi, double delta) {
  return (1.0 / std::tan(rake) + 1.0 / std::tan(b)) * std::sin(phi + b) /
         (2.0 * std::sin(delta + rake + phi + b));
}

shift::core::force_module::SoilTool::Factors
shift::core::force_module::SoilTool::ComputeFactors(double rake,
                                                    double friction_angle,
                                                    double wall_friction) {
  const double phi = friction_angle, delta = wall_friction;
  const double r = std::clamp(rake, 0.05, M_PI - delta - phi - 0.12);
  /* the least n_w, between the two ends where it grows without limit: a
  golden section */
  const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
  double low = 1e-3;
  double high = std::min(0.5 * M_PI, M_PI - delta - r - phi - 1e-3);
  double a = high - golden * (high - low), b = low + golden * (high - low);
  double fa = WeightFactor(a, r, phi, delta);
  double fb = WeightFactor(b, r, phi, delta);
  for(int i = 0; i < 48; i++) {
    if(fa < fb) {
      high = b;
      b = a;
      fb = fa;
      a = high - golden * (high - low);
      fa = WeightFactor(a, r, phi, delta);
    } else {
      low = a;
      a = b;
      fa = fb;
      b = low + golden * (high - low);
      fb = WeightFactor(b, r, phi, delta);
    }
  }
  const double beta = 0.5 * (low + high);
  const double s = std::sin(delta + r + phi + beta);
  Factors factors;
  factors.failure_angle = beta;
  factors.weight = WeightFactor(beta, r, phi, delta);
  factors.cohesion = std::cos(phi) / (s * std::sin(beta));
  factors.adhesion = -std::cos(r + phi + beta) / (s * std::sin(r));
  factors.surcharge = std::sin(phi + beta) / s;
  return factors;
}

void shift::core::force_module::SoilTool::Evaluate(const double* y, double* dy,
                                                   bool apply) {
  if(load_index_ == -2) {
    load_index_ = context_->GetBodyLoadIndex(body_);
  }
  const SoilProperties& soil = soil_properties_;
  const ksl_SE3_t pose =
    context_->get_body_origin_pose(body_, context_->get_dhva());
  ksl_vec3_t tip, face, front;
  ksl_product_dv(&pose, &tip_, &tip);
  ksl_product_drv(&pose.R, &face_, &face);
  ksl_product_drv(&pose.R, &front_, &front);

  /* level direction of the work, and the angle of the blade to the ground,
  with up -z */
  const double level = std::hypot(front.x, front.y);
  pile_ = y[state_offset_];
  depth_ = draft_ = lift_ = 0.0;
  double mobilized = 0.0, speed = 0.0;
  if(level > 1e-9) {
    ksl_vec3_t e = {{front.x / level, front.y / level, 0.0}};
    rake_ = std::atan2(-face.z, -ksl_dot_vv(&face, &e));
    const double sin_rake = std::sin(rake_);
    depth_ = std::clamp(tip.z - ground_, 0.0,
                        height_ * std::max(sin_rake, 0.0));
    const ksl_screw_t velocity = context_->GetBodyVelocity(load_index_);
    ksl_vec3_t w_p;
    ksl_cross_vv(&velocity.ang, &tip, &w_p);
    speed = (velocity.lin.x + w_p.x) * e.x + (velocity.lin.y + w_p.y) * e.y;
    if(depth_ > 0.0 && speed > 0.0) {
      mobilized = speed / std::sqrt(speed * speed + slip_speed_ * slip_speed_);
    }
    const Factors n = ComputeFactors(rake_, soil.friction_angle,
                                     soil.wall_friction);
    failure_angle_ = n.failure_angle;
    const double d = depth_;
    const double f = soil.unit_weight * d * d * n.weight +
                     soil.cohesion * d * n.cohesion +
                     adhesion_ * d * n.adhesion +
                     soil.unit_weight * pile_ * n.surcharge;
    const double total = std::max(f, 0.0) * mobilized * width_;
    const double angle = rake_ + soil.wall_friction;
    draft_ = total * std::sin(angle);
    /* the force up the blade; negative pulls it into the soil */
    lift_ = -total * std::cos(angle);
    if(apply && total > 0.0) {
      ksl_coscrew_t load;
      load.lin.x = -draft_ * e.x;
      load.lin.y = -draft_ * e.y;
      load.lin.z = -lift_;
      ksl_vec3_t point = tip;
      if(sin_rake > 1e-9) {
        ksl_axpy_vv(depth_ / (3.0 * sin_rake), &face, &point);
      }
      ksl_cross_vv(&point, &load.lin, &load.ang);
      context_->AddBodyLoad(load_index_, load);
    }
  }
  if(dy) {
    const double t = std::tan(soil.friction_angle);
    const double full = 0.5 * height_ * height_ / t;
    dy[state_offset_] = mobilized * speed * depth_ -
                        std::max(0.0, pile_ - full) / spill_time_ -
                        (1.0 - mobilized) * pile_ / spill_time_;
  }
}

void shift::core::force_module::SoilTool::Service(const double* y, double* dy) {
  Evaluate(y, dy, true);
}

void shift::core::force_module::SoilTool::UpdateOutputs(double,
                                                        const double* y) {
  Evaluate(y, nullptr, false);
}

std::vector<std::string>
shift::core::force_module::SoilTool::get_output_names() const {
  return {"depth", "rake", "failure_angle", "pile", "draft", "lift"};
}

void shift::core::force_module::SoilTool::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(),
                {depth_, rake_, failure_angle_, pile_, draft_, lift_});
}
