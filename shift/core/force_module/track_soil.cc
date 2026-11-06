#include "shift/core/force_module/track_soil.h"

#include <algorithm>
#include <cmath>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::force_module::TrackSoil::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "track_soil", shift::core::force_module::TrackSoil::Create);

std::unique_ptr<shift::core::force_module::TrackSoil>
shift::core::force_module::TrackSoil::Create() {
  return std::make_unique<shift::core::force_module::TrackSoil>();
}

void shift::core::force_module::TrackSoil::DeclareParameters() {
  DeclareParameter("soil", &soil_, "the soil (see force_module::Soil)", true);
  DeclareParameter("shoes", &shoe_names_,
                   "the bodies (links) carrying a shoe", true)
    .set_row_dim(-1);
  DeclareParameter("shoe_center", &center_,
                   "center of the plane of the grousers' tips, in each "
                   "shoe body's coordinates",
                   true)
    .set_runtime();
  DeclareParameter("shoe_normal", &normal_,
                   "out of the shoe, into the soil, in its body's "
                   "coordinates; defaults to [0, 0, 1]",
                   false);
  DeclareParameter("shoe_along", &along_,
                   "along the track, across the grousers, in its body's "
                   "coordinates; defaults to [1, 0, 0]",
                   false);
  DeclareParameter("shoe_length", &length_, "along the track", true);
  DeclareParameter("shoe_width", &width_, "across the track", true);
  DeclareParameter("grouser_height", &grouser_height_,
                   "defaults to 0 (a flat shoe)", false);
  DeclareParameter("ground", &ground_,
                   "the ground's z (z down, soil below); defaults to 0",
                   false)
    .set_runtime();
  DeclareParameter("stations_along", &stations_along_,
                   "stations of each shoe along the track; defaults to 3",
                   false);
  DeclareParameter("stations_across", &stations_across_,
                   "stations of each shoe across the track; defaults to 2",
                   false);
  DeclareParameter("unloading_ratio", &unloading_ratio_,
                   "plastic over elastic sinkage: the soil springs back by "
                   "z_p / unloading_ratio; defaults to 10",
                   false)
    .set_runtime();
  DeclareParameter("plastic_time", &plastic_time_,
                   "time for the plastic sinkage to follow the sinkage; "
                   "defaults to 0.01",
                   false);
  DeclareParameter("shear_time", &shear_time_,
                   "time for the shear displacement of a lifted shoe to "
                   "relax; defaults to 0.01",
                   false);
  DeclareParameter("contact_depth", &contact_depth_,
                   "sinkage over which a shoe comes onto the soil, for the "
                   "shear displacement and the cohesion; defaults to 0.001",
                   false);
}

void shift::core::force_module::TrackSoil::Connect() {}

static ksl_vec3_t Normalized(ksl_vec3_t v, const std::string& what) {
  const double length = ksl_vec3_l2norm(&v);
  if(!(length > 0.0)) {
    throw shift::core::Exception(what + " must not be zero");
  }
  ksl_vec3_scale(&v, 1.0 / length);
  return v;
}

void shift::core::force_module::TrackSoil::Load() {
  const std::string prefix = "Track soil force module '" + name_ + "'";
  soil_properties_ = soil_.Resolve(prefix);
  if(shoe_names_.empty()) {
    throw shift::core::Exception(prefix + ": needs its shoes");
  }
  if(!(length_ > 0.0) || !(width_ > 0.0) || grouser_height_ < 0.0 ||
     stations_along_ < 1 || stations_across_ < 1 ||
     !(unloading_ratio_ > 0.0) || !(plastic_time_ > 0.0) ||
     !(shear_time_ > 0.0) || !(contact_depth_ > 0.0)) {
    throw shift::core::Exception(
      prefix + ": shoe_length, shoe_width, unloading_ratio, plastic_time, "
               "shear_time and contact_depth must be positive, the "
               "stations at least 1 and grouser_height not negative");
  }
  normal_ = Normalized(normal_, prefix + ": shoe_normal");
  along_ = Normalized(along_, prefix + ": shoe_along");
  /* along, made perpendicular to the normal, and across */
  ksl_axpy_vv(-ksl_dot_vv(&along_, &normal_), &normal_, &along_);
  along_ = Normalized(along_, prefix + ": shoe_along off the normal");
  ksl_cross_vv(&normal_, &along_, &across_);
  shoes_.clear();
  for(const auto& name : shoe_names_) {
    auto body = context_->FindActiveBody(name, get_parent_assembly());
    if(!body) {
      throw shift::core::Exception(prefix + ": body '" + name +
                                   "' is not an active body");
    }
    shoes_.push_back(body);
  }
  load_indices_.assign(shoes_.size(), -2);
  equation_count_ =
    static_cast<int>(shoes_.size()) * get_shoe_state_count();
  q_0_.assign(equation_count_, 0.0);
  outputs_.assign(get_output_names().size(), 0.0);
}

double shift::core::force_module::TrackSoil::Pressure(double z,
                                                      double z_p) const {
  const double b = std::min(length_, width_);
  const double load = soil_properties_.Pressure(b, z);
  if(!(z_p > 0.0)) {
    return load;
  }
  const double reached = soil_properties_.Pressure(b, z_p);
  const double unload =
    reached - unloading_ratio_ * reached / z_p * (z_p - z);
  return std::max(0.0, std::min(load, unload));
}

void shift::core::force_module::TrackSoil::Service(const double* y,
                                                   double* dy) {
  Evaluate(y + state_offset_, dy + state_offset_, true);
}

void shift::core::force_module::TrackSoil::Evaluate(const double* y,
                                                    double* dy, bool apply) {
  const SoilProperties& soil = soil_properties_;
  const int stations = stations_along_ * stations_across_;
  const int count = get_shoe_state_count();
  const double area = length_ * width_ / stations;
  const double tan_phi = std::tan(soil.friction_angle);
  double normal_total = 0.0, traction = 0.0, side = 0.0;
  for(size_t s = 0; s < shoes_.size(); s++) {
    const double* q = y + s * count;
    double* rate = dy ? dy + s * count : nullptr;
    if(load_indices_[s] == -2) {
      load_indices_[s] = context_->GetBodyLoadIndex(shoes_[s]);
    }
    const ksl_SE3_t d =
      context_->get_body_origin_pose(shoes_[s], context_->get_dhva());
    auto world = [&](const ksl_vec3_t& v) {
      ksl_vec3_t w;
      ksl_product_drv(&d.R, &v, &w);
      return w;
    };
    auto point = [&](const ksl_vec3_t& p) {
      ksl_vec3_t w;
      ksl_product_dv(&d, &p, &w);
      return w;
    };
    const ksl_vec3_t normal = world(normal_);
    /* the shoe's directions in the ground plane */
    ksl_vec3_t a = world(along_), c = world(across_);
    a.z = 0.0;
    c.z = 0.0;
    const double la = ksl_vec3_l2norm(&a), lc = ksl_vec3_l2norm(&c);
    if(la > 0.0) {
      ksl_vec3_scale(&a, 1.0 / la);
    }
    if(lc > 0.0) {
      ksl_vec3_scale(&c, 1.0 / lc);
    }

    /* the stations: sinkage, pressure and shear strength; a shoe facing
    away from the ground (on the top run) has no contact */
    const bool facing = normal.z > 0.0;
    std::vector<ksl_vec3_t> points(stations);
    std::vector<double> pressure(stations, 0.0), strength(stations, 0.0);
    double deepest = -HUGE_VAL, center_depth = 0.0;
    for(int i = 0; i < stations_along_; i++) {
      for(int k = 0; k < stations_across_; k++) {
        const int n = i * stations_across_ + k;
        ksl_vec3_t p = center_;
        ksl_axpy_vv(((i + 0.5) / stations_along_ - 0.5) * length_, &along_,
                    &p);
        ksl_axpy_vv(((k + 0.5) / stations_across_ - 0.5) * width_, &across_,
                    &p);
        points[n] = point(p);
        const double z = points[n].z - ground_;
        deepest = std::max(deepest, z);
        center_depth += z / stations;
        if(rate) {
          rate[n] = std::max(0.0, z - q[n]) / plastic_time_;
        }
        if(!facing || !(z > 0.0)) {
          continue;
        }
        pressure[n] = Pressure(z, q[n]);
        strength[n] = (soil.cohesion * -std::expm1(-z / contact_depth_) +
                       pressure[n] * tan_phi) *
                      area;
      }
    }
    if(!facing) {
      deepest = -HUGE_VAL;
    }
    outputs_[s] = std::max(deepest, 0.0);

    /* the shear displacement, and its rate: the slip of the shoe's center
    in the ground plane while it is on the soil */
    const double jx = q[stations], jy = q[stations + 1];
    if(rate) {
      const double w =
        deepest > 0.0 ? -std::expm1(-deepest / contact_depth_) : 0.0;
      const ksl_screw_t velocity =
        context_->GetBodyVelocity(load_indices_[s]);
      const ksl_vec3_t o = point(center_);
      ksl_vec3_t v;
      ksl_cross_vv(&velocity.ang, &o, &v);
      ksl_axpy_vv(1.0, &velocity.lin, &v);
      rate[stations] = w * v.x - (1.0 - w) * jx / shear_time_;
      rate[stations + 1] = w * v.y - (1.0 - w) * jy / shear_time_;
    }
    if(!(deepest > 0.0)) {
      continue;
    }

    /* the forces: normal at the stations, shear in proportion to their
    strength, earth pressure at the center; all oppose the displacement */
    const double j = std::hypot(jx, jy);
    const double mobilized = -std::expm1(-j / soil.shear_modulus);
    const ksl_vec3_t dir =
      j > 0.0 ? ksl_vec3(jx / j, jy / j, 0.0) : ksl_vec3(0.0, 0.0, 0.0);
    const double h_grouser = std::min(std::max(center_depth, 0.0),
                                      grouser_height_);
    const double h_side = std::max(center_depth, 0.0);
    const double earth_along = width_ * soil.PassiveForce(h_grouser);
    const double earth_across = length_ * soil.PassiveForce(h_side);
    ksl_vec3_t earth = {{0.0, 0.0, 0.0}};
    ksl_axpy_vv(-mobilized * ksl_dot_vv(&dir, &a) * earth_along, &a, &earth);
    ksl_axpy_vv(-mobilized * ksl_dot_vv(&dir, &c) * earth_across, &c,
                &earth);
    ksl_coscrew_t load = ksl_coscrew(0, 0, 0, 0, 0, 0);
    auto add = [&](const ksl_vec3_t& f, const ksl_vec3_t& at) {
      ksl_axpy_vv(1.0, &f, &load.lin);
      ksl_vec3_t m;
      ksl_cross_vv(&at, &f, &m);
      ksl_axpy_vv(1.0, &m, &load.ang);
      normal_total -= f.z;
      traction += ksl_dot_vv(&f, &a);
      side += ksl_dot_vv(&f, &c);
    };
    for(int n = 0; n < stations; n++) {
      if(!(pressure[n] > 0.0) && !(strength[n] > 0.0)) {
        continue;
      }
      ksl_vec3_t f = {{0.0, 0.0, -pressure[n] * area}};
      ksl_axpy_vv(-mobilized * strength[n], &dir, &f);
      add(f, points[n]);
    }
    add(earth, point(center_));
    if(apply) {
      context_->AddBodyLoad(load_indices_[s], load);
    }
  }
  const size_t totals = shoes_.size();
  outputs_[totals] = normal_total;
  outputs_[totals + 1] = traction;
  outputs_[totals + 2] = side;
}

std::vector<std::string>
shift::core::force_module::TrackSoil::get_output_names() const {
  std::vector<std::string> names;
  for(const auto& name : shoe_names_) {
    names.push_back(name + "_sinkage");
  }
  names.push_back("normal_force");
  names.push_back("traction");
  names.push_back("side_force");
  return names;
}

void shift::core::force_module::TrackSoil::UpdateOutputs(double,
                                                         const double* y) {
  Evaluate(y + state_offset_, nullptr, false);
}

void shift::core::force_module::TrackSoil::AppendOutputs(
  std::vector<double>& values) const {
  values.insert(values.end(), outputs_.begin(), outputs_.end());
}
