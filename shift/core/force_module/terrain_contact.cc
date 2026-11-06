#include "shift/core/force_module/terrain_contact.h"

#include <algorithm>
#include <cmath>

#include "ksl/mat3x3.h"

#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/terrain/base.h"

bool shift::core::force_module::TerrainContact::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "terrain_contact", shift::core::force_module::TerrainContact::Create);

std::unique_ptr<shift::core::force_module::TerrainContact>
shift::core::force_module::TerrainContact::Create() {
  return std::make_unique<shift::core::force_module::TerrainContact>();
}

void shift::core::force_module::TerrainContact::DeclareParameters() {
  DeclareParameter("body", &body_name_, "the body in contact", true);
  DeclareParameter("points", &points_,
                   "x, y, z of each contact point, in the body's coordinates",
                   true)
    .set_row_dim(-1);
  DeclareParameter("stiffness", &stiffness_,
                   "force per depth (to the exponent) at each point", true);
  DeclareParameter("exponent", &exponent_,
                   "of the depth; optional, default 1", false);
  DeclareParameter("damping", &damping_,
                   "Hunt and Crossley's factor, per speed into the terrain; "
                   "optional, default 0",
                   false);
  DeclareParameter("friction", &friction_,
                   "Coulomb friction coefficient; optional, default 0", false);
  DeclareParameter("slip_speed", &slip_speed_,
                   "sliding speed over which the friction builds up; "
                   "optional, default 0.01",
                   false);
}

void shift::core::force_module::TerrainContact::Connect() {}

void shift::core::force_module::TerrainContact::Load() {
  const std::string prefix = "Terrain contact " + get_name() + ": ";
  body_ = context_->FindActiveBody(body_name_, get_parent_assembly());
  if(!body_) {
    throw shift::core::Exception(prefix + "body " + body_name_ +
                                 " not found among the active bodies");
  }
  if(points_.empty() || points_.size() % 3 != 0) {
    throw shift::core::Exception(prefix + "points needs x, y, z of each "
                                          "point");
  }
  if(!(stiffness_ > 0.0) || !(exponent_ >= 1.0) || damping_ < 0.0 ||
     friction_ < 0.0 || !(slip_speed_ > 0.0)) {
    throw shift::core::Exception(prefix + "stiffness and slip_speed must be "
                                          "positive, the exponent at least "
                                          "1, damping and friction not "
                                          "negative");
  }
  load_index_ = -2;
}

void shift::core::force_module::TerrainContact::Evaluate(bool apply) {
  if(load_index_ == -2) {
    load_index_ = context_->GetBodyLoadIndex(body_);
  }
  const ksl_SE3_t pose =
    context_->get_body_origin_pose(body_, context_->get_dhva());
  const ksl_screw_t velocity = context_->GetBodyVelocity(load_index_);
  const auto& terrains = context_->get_active_terrain();
  normal_force_ = contacts_ = depth_ = 0.0;
  ksl_coscrew_t total = ksl_coscrew(0, 0, 0, 0, 0, 0);
  for(size_t i = 0; i + 2 < points_.size(); i += 3) {
    const ksl_vec3_t local = {{points_[i], points_[i + 1], points_[i + 2]}};
    ksl_vec3_t p;
    ksl_product_dv(&pose, &local, &p);
    ksl_vec3_t surface = {{p.x, p.y, 0.0}};
    ksl_vec3_t n = {{0.0, 0.0, 1.0}};
    bool found = terrains.empty();
    for(auto terrain : terrains) {
      if(terrain->Query(p, surface, n)) {
        found = true;
        break;
      }
    }
    if(!found) {
      continue;
    }
    ksl_vec3_t r;
    ksl_subtract_vv(&p, &surface, &r);
    const double d = ksl_dot_vv(&r, &n);
    if(!(d > 0.0)) {
      continue;
    }
    /* the velocity of the point: v + w x p, the spatial velocity being
    referred to the fixed origin */
    ksl_vec3_t v;
    ksl_cross_vv(&velocity.ang, &p, &v);
    for(int c = 0; c < 3; c++) {
      v.at[c] += velocity.lin.at[c];
    }
    const double rate = ksl_dot_vv(&v, &n);
    const double f_n =
      std::max(0.0, stiffness_ * std::pow(d, exponent_) *
                      (1.0 + damping_ * rate));
    ksl_vec3_t v_t = v;
    ksl_axpy_vv(-rate, &n, &v_t);
    const double slip = std::sqrt(ksl_dot_vv(&v_t, &v_t) +
                                  slip_speed_ * slip_speed_);
    ksl_vec3_t f;
    for(int c = 0; c < 3; c++) {
      f.at[c] = -f_n * n.at[c] - friction_ * f_n * v_t.at[c] / slip;
    }
    normal_force_ += f_n;
    contacts_ += 1.0;
    depth_ = std::max(depth_, d);
    ksl_vec3_t moment;
    ksl_cross_vv(&p, &f, &moment);
    for(int c = 0; c < 3; c++) {
      total.lin.at[c] += f.at[c];
      total.ang.at[c] += moment.at[c];
    }
  }
  if(apply && contacts_ > 0.0) {
    context_->AddBodyLoad(load_index_, total);
  }
}

void shift::core::force_module::TerrainContact::Service(const double*,
                                                        double*) {
  Evaluate(true);
}

void shift::core::force_module::TerrainContact::UpdateOutputs(double,
                                                              const double*) {
  Evaluate(false);
}

std::vector<std::string>
shift::core::force_module::TerrainContact::get_output_names() const {
  return {"normal_force", "contacts", "depth"};
}

void shift::core::force_module::TerrainContact::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), {normal_force_, contacts_, depth_});
}
