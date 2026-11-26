#include "shift/core/force_module/track_soil.h"

#include <algorithm>
#include <cmath>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/tangents.h"
#include "shift/core/force_module/soil_contact.h"
#include "shift/core/terrain/scm.h"

bool shift::core::force_module::TrackSoil::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "track_soil", shift::core::force_module::TrackSoil::Create);

std::unique_ptr<shift::core::force_module::TrackSoil>
shift::core::force_module::TrackSoil::Create() {
  return std::make_unique<shift::core::force_module::TrackSoil>();
}

void shift::core::force_module::TrackSoil::DeclareParameters() {
  DeclareParameter("soil", &soil_,
                   "the soil (see force_module::Soil); on a terrain of type "
                   "\"scm\", defaults to the terrain's",
                   false);
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
  DeclareParameter("rut_time", &rut_time_,
                   "time for a shoe off the soil to forget its plastic "
                   "sinkage, so that a shoe going round the track comes "
                   "down on fresh soil; defaults to 0.5",
                   false);
  DeclareParameter("contact_depth", &contact_depth_,
                   "sinkage over which a shoe comes onto the soil, for the "
                   "shear displacement and the cohesion; defaults to 0.001",
                   false);
}

void shift::core::force_module::TrackSoil::Connect() {
  scm_ = nullptr;
  /* among the terrains and their regions, e.g. sand in rigid ground */
  for(auto terrain : shift::core::terrain::Base::WithRegions(
        context_->get_active_terrain())) {
    if(auto scm = dynamic_cast<shift::core::terrain::Scm*>(terrain)) {
      scm_ = scm;
    }
  }
}

/* whether a soil_contact force module lists every one of the shoes */
static bool SoilContactCovers(
  shift::core::Context* context,
  const std::vector<shift::core::body::Base*>& shoes) {
  for(auto module : context->get_active_force_modules()) {
    auto soil = dynamic_cast<shift::core::force_module::SoilContact*>(module);
    if(!soil) {
      continue;
    }
    std::vector<shift::core::body::Base*> bodies;
    for(const auto& name : soil->get_body_names()) {
      bodies.push_back(
        context->FindActiveBody(name, soil->get_parent_assembly()));
    }
    bool covers = true;
    for(auto shoe : shoes) {
      covers = covers &&
               std::find(bodies.begin(), bodies.end(), shoe) != bodies.end();
    }
    if(covers) {
      return true;
    }
  }
  return false;
}

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
  has_soil_ = soil_.is_given();
  if(has_soil_) {
    soil_properties_ = soil_.Resolve(prefix);
  }
  if(shoe_names_.empty()) {
    throw shift::core::Exception(prefix + ": needs its shoes");
  }
  if(!(length_ > 0.0) || !(width_ > 0.0) || grouser_height_ < 0.0 ||
     stations_along_ < 1 || stations_across_ < 1 ||
     !(unloading_ratio_ > 0.0) || !(plastic_time_ > 0.0) ||
     !(shear_time_ > 0.0) || !(rut_time_ > 0.0) ||
     !(contact_depth_ > 0.0)) {
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
  if(!scm_ && !has_soil_) {
    throw shift::core::Exception(prefix + ": needs its soil");
  }
  if(scm_ && !SoilContactCovers(context_, shoes_)) {
    throw shift::core::Exception(
      prefix + ": on the terrain of type \"scm\", the shoes' bodies must be "
               "listed by a soil_contact force module, which carries them on "
               "the soil");
  }
  load_indices_.assign(shoes_.size(), -2);
  equation_count_ =
    static_cast<int>(shoes_.size()) * get_shoe_state_count();
  q_0_.assign(equation_count_, 0.0);
  DeclareOutputs(OutputNames(), &outputs_,
                 "an output of the track on the soil (see its description)");
}

double shift::core::force_module::TrackSoil::Pressure(double z,
                                                      double z_p) const {
  return Pressure(soil_properties_, z, z_p);
}

double shift::core::force_module::TrackSoil::Pressure(
  const SoilProperties& soil, double z, double z_p) const {
  const double b = std::min(length_, width_);
  const double load = soil.Pressure(b, z);
  if(!(z_p > 0.0)) {
    return load;
  }
  const double reached = soil.Pressure(b, z_p);
  const double unload =
    reached - unloading_ratio_ * reached / z_p * (z_p - z);
  return std::max(0.0, std::min(load, unload));
}

double shift::core::force_module::TrackSoil::PressureSlope(
  const SoilProperties& soil, double z, double z_p) const {
  if(!(z_p > 0.0) || !(z > 0.0)) {
    return 0.0;
  }
  const double b = std::min(length_, width_);
  const double load = soil.Pressure(b, z);
  const double reached = soil.Pressure(b, z_p);
  /* p_load(z_p) (1 - r + r z / z_p) on the unloading line */
  const double line = 1.0 - unloading_ratio_ + unloading_ratio_ * z / z_p;
  const double unload = reached * line;
  if(!(unload < load) || !(unload > 0.0)) {
    return 0.0;
  }
  return soil.n * reached / z_p * line -
         reached * unloading_ratio_ * z / (z_p * z_p);
}

double shift::core::force_module::TrackSoil::PressureSinkageSlope(
  const SoilProperties& soil, double z, double z_p) const {
  if(!(z > 0.0)) {
    return 0.0;
  }
  const double b = std::min(length_, width_);
  if(!(z_p > 0.0)) {
    return soil.PressureSlope(b, z);
  }
  const double load = soil.Pressure(b, z);
  const double reached = soil.Pressure(b, z_p);
  const double unload =
    reached - unloading_ratio_ * reached / z_p * (z_p - z);
  if(!(std::min(load, unload) > 0.0)) {
    return 0.0;
  }
  return unload < load ? unloading_ratio_ * reached / z_p
                       : soil.PressureSlope(b, z);
}

bool shift::core::force_module::TrackSoil::AddTangents(const double* y,
                                                      Tangents& tangents) {
  Evaluate(y + state_offset_, nullptr, false, nullptr, &tangents);
  return true;
}

bool shift::core::force_module::TrackSoil::AddStateTangents(
  const double* y, Tangents& tangents) {
  Evaluate(y + state_offset_, nullptr, false, &tangents);
  return true;
}

void shift::core::force_module::TrackSoil::ToTerrain(const ksl_vec3_t& p,
                                                    double& x, double& y,
                                                    double& z) const {
  const ksl_SE3_t& frame = scm_->get_pose();
  const ksl_vec3_t r = {{p.x - frame.t.x, p.y - frame.t.y, p.z - frame.t.z}};
  x = ksl_dot_vv(&frame.R.v0, &r);
  y = ksl_dot_vv(&frame.R.v1, &r);
  z = ksl_dot_vv(&frame.R.v2, &r);
}

void shift::core::force_module::TrackSoil::Service(const double* y,
                                                   double* dy) {
  Evaluate(y + state_offset_, dy + state_offset_, true);
}

void shift::core::force_module::TrackSoil::Evaluate(const double* y,
                                                    double* dy, bool apply,
                                                    Tangents* tangents,
                                                    Tangents* motion) {
  const int stations = stations_along_ * stations_across_;
  const int count = get_shoe_state_count();
  const double area = length_ * width_ / stations;
  double normal_total = 0.0, traction = 0.0, side = 0.0;
  for(size_t s = 0; s < shoes_.size(); s++) {
    const double* q = y + s * count;
    double* rate = dy ? dy + s * count : nullptr;
    /* the index in y of the shoe's first state */
    const int first =
      static_cast<int>(state_offset_) + static_cast<int>(s) * count;
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

    /* the soil under the shoe: over the grid of a deformable terrain, the
    terrain's, which soil_contact carries the shoe on; elsewhere the ground
    plane's */
    const ksl_vec3_t o = point(center_);
    bool over_scm = false;
    const SoilProperties* shoe_soil = has_soil_ ? &soil_properties_ : nullptr;
    if(scm_) {
      double x, y, z;
      ToTerrain(o, x, y, z);
      long i, k;
      over_scm = scm_->NearestNode(x, y, i, k);
      if(over_scm) {
        shoe_soil = scm_->NodeSoil(i, k);
      } else if(!has_soil_) {
        shoe_soil = scm_->SoilAt(o);
      }
    }
    /* the depth of a point below the ground: the undisturbed surface of the
    terrain over its grid */
    auto depth = [&](const ksl_vec3_t& p) {
      if(!over_scm) {
        return p.z - ground_;
      }
      double x, y, z, z_x, z_y;
      ToTerrain(p, x, y, z);
      return z - scm_->LocalZ(x, y, z_x, z_y);
    };
    /* its gradient, in fixed coordinates */
    auto depth_gradient = [&](const ksl_vec3_t& p, double g[3]) {
      if(!over_scm) {
        g[0] = g[1] = 0.0;
        g[2] = 1.0;
        return;
      }
      double x, y, z, z_x, z_y;
      ToTerrain(p, x, y, z);
      scm_->LocalZ(x, y, z_x, z_y);
      const ksl_SE3_t& frame = scm_->get_pose();
      for(int c = 0; c < 3; c++) {
        g[c] = frame.R.v2.at[c] - z_x * frame.R.v0.at[c] - z_y * frame.R.v1.at[c];
      }
    };
    /* the stiffness block of the shoe's body, and the derivative of the
    sinkage at a point p of the shoe, dz / d delta = g^T E(p) */
    double block[36] = {};
    auto sinkage_row = [&](const ksl_vec3_t& p, double row[6]) {
      double g[3];
      depth_gradient(p, g);
      for(int c = 0; c < 6; c++) {
        row[c] = 0.0;
      }
      Tangents::AddPointRow(g, p, 1.0, row);
    };
    /* K -= [v; at x v] row^T, for a force at the point at that changes by v
    per unit of a scalar whose derivative is row */
    auto add_column = [&](const ksl_vec3_t& at, const ksl_vec3_t& v,
                          const double row[6]) {
      ksl_vec3_t m;
      ksl_cross_vv(&at, &v, &m);
      for(int r = 0; r < 3; r++) {
        for(int c = 0; c < 6; c++) {
          block[r * 6 + c] -= v.at[r] * row[c];
          block[(r + 3) * 6 + c] -= m.at[r] * row[c];
        }
      }
    };
    int deepest_station = -1;
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
        const double z = depth(points[n]);
        if(z > deepest) {
          deepest_station = n;
        }
        deepest = std::max(deepest, z);
        center_depth += z / stations;
        if(rate && over_scm) {
          /* the rut is the terrain's: forget it, as off the soil */
          rate[n] = -q[n] / rut_time_;
        } else if(rate) {
          rate[n] = std::max(0.0, z - q[n]) / plastic_time_;
          if(z < 0.0) {
            /* off the soil: the rut stays behind, and the shoe comes down
            on fresh soil */
            rate[n] += q[n] * std::expm1(z / contact_depth_) / rut_time_;
          }
        }
        if(tangents) {
          double slope = 0.0;
          if(over_scm) {
            slope = -1.0 / rut_time_;
          } else {
            slope = z > q[n] ? -1.0 / plastic_time_ : 0.0;
            if(z < 0.0) {
              slope += std::expm1(z / contact_depth_) / rut_time_;
            }
          }
          tangents->AddStateRate(first + n, first + n, slope);
        }
        if(motion && !over_scm) {
          /* the rate with the sinkage */
          double slope = z > q[n] ? 1.0 / plastic_time_ : 0.0;
          if(z < 0.0) {
            slope += q[n] * std::exp(z / contact_depth_) /
                     (contact_depth_ * rut_time_);
          }
          if(slope != 0.0) {
            double row[6];
            sinkage_row(points[n], row);
            for(int c = 0; c < 6; c++) {
              row[c] *= slope;
            }
            motion->AddStateBodyStiffness(first + n, load_indices_[s], row);
          }
        }
        if(!facing || !(z > 0.0) || over_scm || !shoe_soil) {
          continue;
        }
        pressure[n] = Pressure(*shoe_soil, z, q[n]);
        strength[n] = (shoe_soil->cohesion * -std::expm1(-z / contact_depth_) +
                       pressure[n] * std::tan(shoe_soil->friction_angle)) *
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
    const double w =
      deepest > 0.0 ? -std::expm1(-deepest / contact_depth_) : 0.0;
    if(tangents) {
      for(int i = 0; i < 2; i++) {
        tangents->AddStateRate(first + stations + i, first + stations + i,
                               -(1.0 - w) / shear_time_);
      }
    }
    if(rate || motion) {
      const ksl_screw_t velocity =
        context_->GetBodyVelocity(load_indices_[s]);
      ksl_vec3_t v;
      ksl_cross_vv(&velocity.ang, &o, &v);
      ksl_axpy_vv(1.0, &velocity.lin, &v);
      if(rate) {
        rate[stations] = w * v.x - (1.0 - w) * jx / shear_time_;
        rate[stations + 1] = w * v.y - (1.0 - w) * jy / shear_time_;
      }
      if(motion) {
        /* with the velocity of the center, w E(o), with its displacement,
        and with the deepest sinkage through w */
        /* the deepest sinkage is the stations' largest: where several are
        as deep (a flat shoe on flat ground), the mean of theirs, the middle
        of its one-sided derivatives over the symmetric grid */
        double deep_row[6] = {};
        const bool sunk = deepest > 0.0 && deepest_station > -1;
        if(sunk) {
          int tied = 0;
          for(int n = 0; n < stations; n++) {
            if(depth(points[n]) >= deepest - 1e-12) {
              double row[6];
              sinkage_row(points[n], row);
              for(int c = 0; c < 6; c++) {
                deep_row[c] += row[c];
              }
              tied++;
            }
          }
          for(int c = 0; c < 6; c++) {
            deep_row[c] /= tied;
          }
        }
        const double by_deepest =
          sunk ? std::exp(-deepest / contact_depth_) / contact_depth_ : 0.0;
        const double shear_j[2] = {jx, jy};
        for(int i = 0; i < 2; i++) {
          const double axis[3] = {i == 0 ? 1.0 : 0.0, i == 1 ? 1.0 : 0.0,
                                  0.0};
          double row[6] = {};
          Tangents::AddPointRow(axis, o, w, row);
          motion->AddStateBodyDamping(first + stations + i, load_indices_[s],
                                      row);
          /* the center moves with the shoe by E(o) delta, so its velocity
          changes by omega x E(o) delta with the angular velocity omega:
          e_i . (omega x d) = (e_i x omega) . d */
          const ksl_vec3_t e_i = {{axis[0], axis[1], axis[2]}};
          ksl_vec3_t turning;
          ksl_cross_vv(&e_i, &velocity.ang, &turning);
          double stiffness_row[6] = {};
          Tangents::AddPointRow(turning.at, o, w, stiffness_row);
          const double by_w = v.at[i] + shear_j[i] / shear_time_;
          if(by_deepest * by_w != 0.0) {
            for(int c = 0; c < 6; c++) {
              stiffness_row[c] += by_deepest * by_w * deep_row[c];
            }
          }
          motion->AddStateBodyStiffness(first + stations + i, load_indices_[s],
                                        stiffness_row);
        }
      }
    }
    if(!(deepest > 0.0) || !shoe_soil) {
      continue;
    }
    const SoilProperties& soil = *shoe_soil;

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
    add(earth, o);
    if(tangents) {
      /* the load of a force f at p, (f, p x f) */
      auto add_load = [](const ksl_vec3_t& f, const ksl_vec3_t& at,
                         double load[6]) {
        ksl_vec3_t m;
        ksl_cross_vv(&at, &f, &m);
        for(int i = 0; i < 3; i++) {
          load[i] += f.at[i];
          load[3 + i] += m.at[i];
        }
      };
      /* the plastic sinkage of a station changes its pressure, and the
      shear strength with it */
      for(int n = 0; n < stations; n++) {
        if(!(pressure[n] > 0.0)) {
          continue;
        }
        const double slope = PressureSlope(soil, depth(points[n]), q[n]);
        if(slope == 0.0) {
          continue;
        }
        ksl_vec3_t f = {{0.0, 0.0, -slope * area}};
        ksl_axpy_vv(-mobilized * slope * std::tan(soil.friction_angle) * area,
                    &dir, &f);
        double load[6] = {};
        add_load(f, points[n], load);
        tangents->AddStateLoad(first + n, load_indices_[s], load);
      }
      /* the shear displacement: u = mobilized(j) j / |j|, whose derivative
      is g I + (dg/dj / j) j j^T with g = mobilized / j (1 / K at j = 0) */
      const double modulus = soil.shear_modulus;
      double g = 1.0 / modulus, dg = 0.0;
      if(j > 1e-9 * modulus) {
        g = mobilized / j;
        dg = (std::exp(-j / modulus) / modulus * j - mobilized) / (j * j * j);
      }
      for(int b = 0; b < 2; b++) {
        const double j_b = b == 0 ? jx : jy;
        const ksl_vec3_t du = {{(b == 0 ? g : 0.0) + dg * jx * j_b,
                                (b == 1 ? g : 0.0) + dg * jy * j_b, 0.0}};
        double load[6] = {};
        for(int n = 0; n < stations; n++) {
          if(!(strength[n] > 0.0)) {
            continue;
          }
          ksl_vec3_t f = {{0.0, 0.0, 0.0}};
          ksl_axpy_vv(-strength[n], &du, &f);
          add_load(f, points[n], load);
        }
        ksl_vec3_t f = {{0.0, 0.0, 0.0}};
        ksl_axpy_vv(-ksl_dot_vv(&du, &a) * earth_along, &a, &f);
        ksl_axpy_vv(-ksl_dot_vv(&du, &c) * earth_across, &c, &f);
        add_load(f, o, load);
        tangents->AddStateLoad(first + stations + b, load_indices_[s], load);
      }
    }
    if(apply) {
      context_->AddBodyLoad(load_indices_[s], load);
    }
    if(motion) {
      const double tan_phi = std::tan(soil.friction_angle);
      for(int n = 0; n < stations; n++) {
        if(!(pressure[n] > 0.0) && !(strength[n] > 0.0)) {
          continue;
        }
        const double z = depth(points[n]);
        const double by_z = PressureSinkageSlope(soil, z, q[n]);
        const double strength_by_z =
          (soil.cohesion * std::exp(-z / contact_depth_) / contact_depth_ +
           by_z * tan_phi) *
          area;
        ksl_vec3_t v = {{0.0, 0.0, -by_z * area}};
        ksl_axpy_vv(-mobilized * strength_by_z, &dir, &v);
        double row[6];
        sinkage_row(points[n], row);
        add_column(points[n], v, row);
        /* the moment of the force about the moving station */
        ksl_vec3_t f = {{0.0, 0.0, -pressure[n] * area}};
        ksl_axpy_vv(-mobilized * strength[n], &dir, &f);
        Tangents::AddMomentArmBlock(points[n], f, 1.0, block);
      }
      /* the earth pressure with the mean sinkage */
      double center_row[6] = {};
      for(int n = 0; n < stations; n++) {
        double row[6];
        sinkage_row(points[n], row);
        for(int c = 0; c < 6; c++) {
          center_row[c] += row[c] / stations;
        }
      }
      const double along_by_depth =
        center_depth > 0.0 && center_depth < grouser_height_
          ? width_ * soil.PassiveForceSlope(h_grouser)
          : 0.0;
      const double across_by_depth =
        center_depth > 0.0 ? length_ * soil.PassiveForceSlope(h_side) : 0.0;
      ksl_vec3_t v = {{0.0, 0.0, 0.0}};
      ksl_axpy_vv(-mobilized * ksl_dot_vv(&dir, &a) * along_by_depth, &a, &v);
      ksl_axpy_vv(-mobilized * ksl_dot_vv(&dir, &c) * across_by_depth, &c, &v);
      add_column(o, v, center_row);
      Tangents::AddMomentArmBlock(o, earth, 1.0, block);
      /* the earth pressure with the turning of the shoe's directions in
      the ground plane: e = normalize(P world(d)) changes with the rotation
      w by (I - e e^T) / |P world(d)| P (-[world(d) x]) w */
      auto turning = [&](const ksl_vec3_t& local, const ksl_vec3_t& e,
                         double length, double j[9]) {
        const ksl_vec3_t r = world(local);
        const double cross[9] = {0.0,     -r.at[2], r.at[1], r.at[2], 0.0,
                                 -r.at[0], -r.at[1], r.at[0], 0.0};
        for(int row = 0; row < 3; row++) {
          for(int col = 0; col < 3; col++) {
            double value = 0.0;
            for(int m = 0; m < 2; m++) {
              value += ((row == m ? 1.0 : 0.0) - e.at[row] * e.at[m]) *
                       -cross[m * 3 + col];
            }
            j[row * 3 + col] = length > 0.0 ? value / length : 0.0;
          }
        }
      };
      double j_a[9], j_c[9];
      turning(along_, a, la, j_a);
      turning(across_, c, lc, j_c);
      const double force_a = width_ * soil.PassiveForce(h_grouser);
      const double force_c = length_ * soil.PassiveForce(h_side);
      const double d_a = ksl_dot_vv(&dir, &a), d_c = ksl_dot_vv(&dir, &c);
      for(int col = 0; col < 3; col++) {
        /* dE / dw_col */
        ksl_vec3_t de = {{0.0, 0.0, 0.0}};
        double dir_j_a = 0.0, dir_j_c = 0.0;
        for(int m = 0; m < 3; m++) {
          dir_j_a += dir.at[m] * j_a[m * 3 + col];
          dir_j_c += dir.at[m] * j_c[m * 3 + col];
        }
        for(int r = 0; r < 3; r++) {
          de.at[r] = -mobilized * (force_a * (a.at[r] * dir_j_a +
                                              d_a * j_a[r * 3 + col]) +
                                   force_c * (c.at[r] * dir_j_c +
                                              d_c * j_c[r * 3 + col]));
        }
        ksl_vec3_t m;
        ksl_cross_vv(&o, &de, &m);
        for(int r = 0; r < 3; r++) {
          block[r * 6 + 3 + col] -= de.at[r];
          block[(r + 3) * 6 + 3 + col] -= m.at[r];
        }
      }
      motion->AddBodyStiffness(load_indices_[s], load_indices_[s], block);
    }
  }
  const size_t totals = shoes_.size();
  outputs_[totals] = normal_total;
  outputs_[totals + 1] = traction;
  outputs_[totals + 2] = side;
}

std::vector<std::string>
shift::core::force_module::TrackSoil::OutputNames() const {
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
