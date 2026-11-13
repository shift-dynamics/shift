#include "shift/core/terrain/sph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ksl/vec3.h"
#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/log.h"
#include "shift/core/model_parameters.h"
#include "shift/core/sph/cpu_solver.h"
#include "shift/core/sph/gpu_solver.h"
#include "shift/core/terrain/scm.h"

bool shift::core::terrain::Sph::registered_ =
  shift::core::Factory<shift::core::terrain::Base>::get()->RegisterModule(
    "sph", shift::core::terrain::Sph::Create);

std::unique_ptr<shift::core::terrain::Sph> shift::core::terrain::Sph::Create() {
  return std::make_unique<shift::core::terrain::Sph>();
}

shift::core::terrain::Sph::~Sph() {}

void shift::core::terrain::Sph::DeclareParameters() {
  shift::core::terrain::Basic::DeclareParameters();
  DeclareParameter("depth", &depth_,
                   "depth of the soil under the surface (m), rounded to a "
                   "whole number of spacings",
                   true);
  DeclareParameter("spacing", &spacing_,
                   "distance between the particles at rest (m); the extent "
                   "x, y is rounded to a whole number of spacings; default "
                   "0.02",
                   false);
  DeclareParameter("smoothing_ratio", &smoothing_ratio_,
                   "smoothing length of the kernel over the spacing, from 1 "
                   "to 2; default 1.3",
                   false);
  DeclareParameter("artificial_viscosity", &artificial_viscosity_,
                   "Monaghan's artificial viscosity alpha, which damps the "
                   "particles' relative motion; default 0.5",
                   false);
  DeclareParameter("cfl", &cfl_,
                   "substep over the time a compression wave takes to cross "
                   "a smoothing length, from 0 to 1; default 0.25",
                   false);
  DeclareParameter("sound_speed", &sound_speed_,
                   "speed of compression waves for the substep (m/s); 0, "
                   "the default, for that of the soils' elastic properties",
                   false);
  DeclareParameter("marker_layers", &marker_layers_,
                   "layers of fixed markers of the floor and the walls; at "
                   "least 2 smoothing lengths deep; default 3",
                   false);
  DeclareParameter("wall_height", &wall_height_,
                   "height of the walls above the surface (m); default "
                   "depth",
                   false);
  DeclareParameter("substeps_per_step", &substeps_per_step_,
                   "the longest integration step, in stable substeps of "
                   "the soil; default 1. The soil's force on the bodies is "
                   "a step behind them, which puts energy into a light "
                   "body on it when the step is longer",
                   false);
  DeclareParameter("landing_interval", &landing_interval_,
                   "time between the checks for particles that have landed "
                   "outside the extent (s); default 0.01",
                   false);
  DeclareParameter("device", &device_,
                   "\"gpu\" (default) or \"cpu\" (single threaded, for small "
                   "soils)",
                   false);
}

/*!
The columns are spacing apart from x[0], y[0]; each is filled from the
surface down to the floor at depth, and the floor and the walls of markers
are laid out on the same lattice, so that the soil rests on them as on more
soil.
*/
void shift::core::terrain::Sph::Load() {
  shift::core::terrain::Basic::Load();
  const std::string prefix = "Terrain " + get_name() + ": ";
  if(!has_extent() || !(x_range_[1] > x_range_[0]) ||
     !(y_range_[1] > y_range_[0])) {
    throw shift::core::Exception(prefix + "x and y must each be [first, "
                                          "last] with last > first");
  }
  if(!(spacing_ > 0.0) || !(depth_ >= spacing_)) {
    throw shift::core::Exception(prefix + "spacing must be positive and "
                                          "depth at least a spacing");
  }
  if(!(smoothing_ratio_ >= 1.0 && smoothing_ratio_ <= 2.0) ||
     !(cfl_ > 0.0 && cfl_ <= 1.0) || artificial_viscosity_ < 0.0 ||
     sound_speed_ < 0.0) {
    throw shift::core::Exception(prefix + "smoothing_ratio must be from 1 to "
                                          "2, cfl from 0 to 1, and "
                                          "artificial_viscosity and "
                                          "sound_speed not negative");
  }
  if(marker_layers_ < 1) {
    throw shift::core::Exception(prefix + "marker_layers must be at least 1");
  }
  if(marker_layers_ < std::ceil(2.0 * smoothing_ratio_ - 1e-9)) {
    SHIFT_WARNING(prefix + "marker_layers is less than the support of the "
                           "kernel: particles near the floor and walls see "
                           "too few markers");
  }
  if(!(landing_interval_ > 0.0)) {
    throw shift::core::Exception(prefix + "landing_interval must be "
                                          "positive");
  }
  if(substeps_per_step_ < 1) {
    throw shift::core::Exception(prefix + "substeps_per_step must be at "
                                          "least 1");
  }
  if(device_ != "gpu" && device_ != "cpu") {
    throw shift::core::Exception(prefix + "device must be \"gpu\" or "
                                          "\"cpu\", not \"" + device_ + "\"");
  }
  const double wall_height = std::isnan(wall_height_) ? depth_ : wall_height_;
  if(wall_height < 0.0) {
    throw shift::core::Exception(prefix + "wall_height must not be negative");
  }
  const std::vector<shift::core::force_module::SoilProperties>& soils =
    get_soils();
  if(soils.empty()) {
    throw shift::core::Exception(prefix + "needs a soil, of its own or of "
                                          "the terrain that holds it");
  }
  /* a material per soil; a soil without the properties of a continuum is
  an error only where the soil has it */
  const double standard_gravity = 9.80665;
  materials_.assign(soils.size(), shift::core::sph::Material());
  std::vector<char> usable(soils.size(), 0);
  int first_usable = -1;
  for(size_t k = 0; k < soils.size(); k++) {
    const shift::core::force_module::SoilProperties& soil = soils[k];
    if(!soil.has_continuum() || !(soil.unit_weight > 0.0)) {
      continue;
    }
    shift::core::sph::Material& m = materials_[k];
    m.density = static_cast<float>(soil.unit_weight / standard_gravity);
    m.youngs_modulus = static_cast<float>(soil.youngs_modulus);
    m.poisson_ratio = static_cast<float>(soil.poisson_ratio);
    m.friction_angle = static_cast<float>(soil.friction_angle);
    m.dilatancy_angle = static_cast<float>(
      std::isnan(soil.dilatancy_angle) ? 0.0 : soil.dilatancy_angle);
    m.cohesion = static_cast<float>(soil.cohesion);
    usable[k] = 1;
    if(first_usable < 0) {
      first_usable = static_cast<int>(k);
    }
  }
  if(first_usable < 0) {
    throw shift::core::Exception(prefix + "the soil needs youngs_modulus, "
                                          "poisson_ratio and a positive "
                                          "unit_weight");
  }
  /* the soils not used here take the first that can be, so that they do not
  change the substep */
  for(size_t k = 0; k < soils.size(); k++) {
    if(!usable[k]) {
      materials_[k] = materials_[first_usable];
    }
  }
  settings_ = shift::core::sph::Settings();
  settings_.spacing = static_cast<float>(spacing_);
  settings_.smoothing_ratio = static_cast<float>(smoothing_ratio_);
  settings_.artificial_viscosity = static_cast<float>(artificial_viscosity_);
  settings_.cfl = static_cast<float>(cfl_);
  settings_.sound_speed = static_cast<float>(sound_speed_);

  /* the lattice */
  const double s = spacing_;
  origin_ = {x_range_[0], y_range_[0], 0.0};
  columns_x_ = std::max(1L, std::lround((x_range_[1] - x_range_[0]) / s));
  columns_y_ = std::max(1L, std::lround((y_range_[1] - y_range_[0]) / s));
  const long layers = std::max(1L, std::lround(depth_ / s));
  const long above = static_cast<long>(std::ceil(wall_height / s - 1e-9));
  const long walls = marker_layers_;
  if(double(columns_x_ + 2 * walls) * double(columns_y_ + 2 * walls) *
       double(layers + above + walls) >
     double(1u << 30)) {
    throw shift::core::Exception(prefix + "too many particles: a larger "
                                          "spacing or a smaller extent");
  }
  state_ = shift::core::sph::State();
  floor_.assign(size_t(columns_x_) * columns_y_, 0.0);
  auto column_soil = [&](long i, long j) {
    const long ci = std::clamp(i, 0L, columns_x_ - 1);
    const long cj = std::clamp(j, 0L, columns_y_ - 1);
    const double x = origin_[0] + (ci + 0.5) * s;
    const double y = origin_[1] + (cj + 0.5) * s;
    const int index = LocalSoilIndex(x, y);
    if(index < 0 || !usable[index]) {
      throw shift::core::Exception(
        prefix + "the soil at x " + std::to_string(x) + ", y " +
        std::to_string(y) + (index < 0 ? " is not given" :
                                         " needs youngs_modulus, "
                                         "poisson_ratio and a positive "
                                         "unit_weight"));
    }
    return index;
  };
  const float fs = static_cast<float>(s);
  /* the particles, then the markers, each column from the top down */
  for(long j = 0; j < columns_y_; j++) {
    for(long i = 0; i < columns_x_; i++) {
      const double x = origin_[0] + (i + 0.5) * s;
      const double y = origin_[1] + (j + 0.5) * s;
      double z_x, z_y;
      const double top = LocalZ(x, y, z_x, z_y);
      const int index = column_soil(i, j);
      for(long k = 0; k < layers; k++) {
        state_.AddParticle({static_cast<float>((i + 0.5) * s),
                            static_cast<float>((j + 0.5) * s),
                            static_cast<float>(top + (k + 0.5) * s)},
                           static_cast<uint32_t>(index), materials_[index],
                           fs);
      }
      floor_[size_t(j) * columns_x_ + i] = top + layers * s;
    }
  }
  particle_count_ = state_.size();
  for(long j = -walls; j < columns_y_ + walls; j++) {
    for(long i = -walls; i < columns_x_ + walls; i++) {
      const bool inside =
        i >= 0 && i < columns_x_ && j >= 0 && j < columns_y_;
      const double x = origin_[0] + (i + 0.5) * s;
      const double y = origin_[1] + (j + 0.5) * s;
      double z_x, z_y;
      const double top = LocalZ(x, y, z_x, z_y);
      const shift::core::sph::Material& m = materials_[column_soil(i, j)];
      /* under the soil, the floor; around it, the walls from wall_height
      above the surface down to the bottom of the floor */
      for(long k = inside ? layers : -above; k < layers + walls; k++) {
        state_.AddMarker({static_cast<float>((i + 0.5) * s),
                          static_cast<float>((j + 0.5) * s),
                          static_cast<float>(top + (k + 0.5) * s)},
                         m, fs);
      }
    }
  }
  SHIFT_INFO(prefix + std::to_string(particle_count_) + " particles, " +
             std::to_string(state_.size() - particle_count_) + " markers");
  initial_state_ = state_;
  initial_particle_count_ = particle_count_;
  step_ = shift::core::sph::StableStep(settings_, materials_);
  Restart();
}

void shift::core::terrain::Sph::Restart() {
  state_ = initial_state_;
  particle_count_ = initial_particle_count_;
  solver_.reset();
  started_ = false;
  time_ = last_time_ = next_landing_ = 0.0;
  landed_volume_ = lost_volume_ = 0.0;
  displaced_count_ = 0;
  for(ksl_coscrew_t& load : body_loads_) {
    load = ksl_coscrew_t{};
  }
  for(ksl_coscrew_t& impulse : body_impulses_) {
    impulse = ksl_coscrew_t{};
  }
  UpdateSurface();
}

double shift::core::terrain::Sph::FloorZ(double x, double y) const {
  const long i = std::clamp(
    static_cast<long>(std::floor((x - origin_[0]) / spacing_)), 0L,
    columns_x_ - 1);
  const long j = std::clamp(
    static_cast<long>(std::floor((y - origin_[1]) / spacing_)), 0L,
    columns_y_ - 1);
  return floor_[size_t(j) * columns_x_ + i];
}

ksl_SE3_t shift::core::terrain::Sph::get_particle_frame() const {
  ksl_SE3_t frame = get_pose();
  for(int a = 0; a < 3; a++) {
    for(int k = 0; k < 3; k++) {
      frame.t.at[k] += frame.R.v[a].at[k] * origin_[a];
    }
  }
  return frame;
}

double shift::core::terrain::Sph::StepLimit() const {
  return substeps_per_step_ * step_;
}

void shift::core::terrain::Sph::SetBodies(
  const std::vector<body::Base*>& bodies,
  const std::vector<std::vector<collision_geometry::Base*>>& geometry) {
  if(started_) {
    throw shift::core::Exception("Terrain " + get_name() + ": the bodies are "
                                 "set before the first step");
  }
  bodies_ = bodies;
  body_geometry_ = geometry;
  body_geometry_.resize(bodies_.size());
  body_loads_.assign(bodies_.size(), ksl_coscrew_t{});
  body_impulses_.assign(bodies_.size(), ksl_coscrew_t{});
  load_indices_.assign(bodies_.size(), -2);
}

size_t shift::core::terrain::Sph::get_body_marker_count(size_t b) const {
  return b + 1 < body_offsets_.size()
           ? body_offsets_[b + 1] - body_offsets_[b]
           : 0;
}

/*!
In each geometry's frame, the lattice of points spacing apart, half a spacing
off its axes (so that on a face along the axes they are half a spacing inside,
as the walls' markers are): those inside
it (a ray from the point reaches it at distance 0, collision_geometry::Base::
RayCast) with a point outside it within marker_layers spacings along x, y or
z, and the first points rays along its axes, from both sides, reach on its
surface, moved half a spacing inside it as the walls' markers are (where it
is that thick), one per cell of the lattice. The markers are kept in the body's
frame.
*/
void shift::core::terrain::Sph::SampleBodies(shift::core::sph::State& state) {
  const std::string prefix = "Terrain " + get_name() + ": ";
  const double s = spacing_;
  const long layers = marker_layers_;
  const shift::core::sph::Material& material = materials_.front();
  body_offsets_.assign(1, 0);
  for(size_t b = 0; b < bodies_.size(); b++) {
    const ksl_SE3_t body_pose =
      context_->get_body_origin_pose(bodies_[b], context_->get_dhva());
    size_t count = 0;
    for(collision_geometry::Base* geometry : body_geometry_[b]) {
      geometry->UpdatePose(body_pose);
      const ksl_SE3_t& d = geometry->get_d_absolute();
      /* the box of the geometry along its axes, in lattice points */
      long lo[3], hi[3];
      bool bounded = true;
      for(int a = 0; a < 3; a++) {
        double extent[2];
        for(int side = 0; side < 2; side++) {
          const double sign = side == 0 ? -1.0 : 1.0;
          const ksl_vec3_t direction = {{sign * d.R.v[a].x, sign * d.R.v[a].y,
                                         sign * d.R.v[a].z}};
          ksl_vec3_t local, global;
          int hint = 0;
          geometry->Support(direction, local, global, hint);
          const ksl_vec3_t r = {{global.x - d.t.x, global.y - d.t.y,
                                 global.z - d.t.z}};
          extent[side] = ksl_dot_vv(&d.R.v[a], &r);
        }
        if(!std::isfinite(extent[0]) || !std::isfinite(extent[1])) {
          bounded = false;
          break;
        }
        lo[a] = static_cast<long>(std::floor(extent[0] / s)) - 1;
        hi[a] = static_cast<long>(std::ceil(extent[1] / s)) + 1;
      }
      if(!bounded) {
        throw shift::core::Exception(prefix + "the collision geometry " +
                                     geometry->get_name() + " of body " +
                                     bodies_[b]->get_name() +
                                     " is unbounded");
      }
      const long n[3] = {hi[0] - lo[0] + 1, hi[1] - lo[1] + 1,
                         hi[2] - lo[2] + 1};
      if(double(n[0]) * n[1] * n[2] > 5e7) {
        throw shift::core::Exception(prefix + "body " +
                                     bodies_[b]->get_name() +
                                     " is too large for markers at this "
                                     "spacing");
      }
      auto point = [&](double u, double v, double w) {
        const double c[3] = {u, v, w};
        ksl_vec3_t p = d.t;
        for(int a = 0; a < 3; a++) {
          for(int k = 0; k < 3; k++) {
            p.at[k] += d.R.v[a].at[k] * c[a];
          }
        }
        return p;
      };
      auto index = [&](long i, long j, long k) {
        return static_cast<size_t>(((k - lo[2]) * n[1] + (j - lo[1])) * n[0] +
                                   (i - lo[0]));
      };
      /* inside, then the shell: inside, with a point outside within layers
      along an axis (an erosion of the inside, axis by axis) */
      std::vector<char> inside(size_t(n[0]) * n[1] * n[2], 0);
      for(long k = lo[2]; k <= hi[2]; k++) {
        for(long j = lo[1]; j <= hi[1]; j++) {
          for(long i = lo[0]; i <= hi[0]; i++) {
            double t;
            ksl_vec3_t normal;
            if(geometry->RayCast(point((i + 0.5) * s, (j + 0.5) * s,
                                       (k + 0.5) * s),
                                 d.R.v2, 1e-9 * s, t, normal) &&
               t <= 0.0) {
              inside[index(i, j, k)] = 1;
            }
          }
        }
      }
      std::vector<char> core = inside, next(inside.size());
      for(int a = 0; a < 3; a++) {
        for(long k = lo[2]; k <= hi[2]; k++) {
          for(long j = lo[1]; j <= hi[1]; j++) {
            for(long i = lo[0]; i <= hi[0]; i++) {
              char value = 1;
              for(long m = -layers; m <= layers && value; m++) {
                long c[3] = {i, j, k};
                c[a] += m;
                value = c[a] >= lo[a] && c[a] <= hi[a] &&
                        core[index(c[0], c[1], c[2])];
              }
              next[index(i, j, k)] = value;
            }
          }
        }
        core.swap(next);
      }
      std::vector<char> taken(inside.size(), 0);
      auto add = [&](const ksl_vec3_t& p) {
        const ksl_vec3_t r = {{p.x - body_pose.t.x, p.y - body_pose.t.y,
                               p.z - body_pose.t.z}};
        state.AddBodyMarker({static_cast<float>(ksl_dot_vv(&body_pose.R.v0, &r)),
                             static_cast<float>(ksl_dot_vv(&body_pose.R.v1, &r)),
                             static_cast<float>(ksl_dot_vv(&body_pose.R.v2, &r))},
                            static_cast<uint32_t>(b), material,
                            static_cast<float>(s));
        count++;
      };
      for(long k = lo[2]; k <= hi[2]; k++) {
        for(long j = lo[1]; j <= hi[1]; j++) {
          for(long i = lo[0]; i <= hi[0]; i++) {
            if(inside[index(i, j, k)] && !core[index(i, j, k)]) {
              taken[index(i, j, k)] = 1;
              add(point((i + 0.5) * s, (j + 0.5) * s, (k + 0.5) * s));
            }
          }
        }
      }
      /* the surface, along each axis from both sides */
      for(int a = 0; a < 3; a++) {
        const int b1 = (a + 1) % 3, b2 = (a + 2) % 3;
        for(int side = 0; side < 2; side++) {
          const double sign = side == 0 ? 1.0 : -1.0;
          const ksl_vec3_t direction = {{sign * d.R.v[a].x, sign * d.R.v[a].y,
                                         sign * d.R.v[a].z}};
          const double from = (side == 0 ? lo[a] : hi[a]) * s;
          const double reach = (hi[a] - lo[a]) * s;
          for(long v = lo[b2]; v <= hi[b2]; v++) {
            for(long u = lo[b1]; u <= hi[b1]; u++) {
              double c[3];
              c[a] = from;
              c[b1] = (u + 0.5) * s;
              c[b2] = (v + 0.5) * s;
              double t;
              ksl_vec3_t normal;
              if(!geometry->RayCast(point(c[0], c[1], c[2]), direction, reach,
                                    t, normal)) {
                continue;
              }
              /* half a spacing inside, as the walls' markers are, so that the
              soil meets the body at its surface; on the surface where the
              body is thinner */
              c[a] = from + sign * (t + 0.5 * s);
              double depth;
              if(!(geometry->RayCast(point(c[0], c[1], c[2]), direction,
                                     1e-9 * s, depth, normal) &&
                   depth <= 0.0)) {
                c[a] = from + sign * t;
              }
              long cell[3];
              for(int m = 0; m < 3; m++) {
                cell[m] = std::clamp(static_cast<long>(std::floor(c[m] / s)),
                                     lo[m], hi[m]);
              }
              const size_t key = index(cell[0], cell[1], cell[2]);
              if(!taken[key]) {
                taken[key] = 1;
                add(point(c[0], c[1], c[2]));
              }
            }
          }
        }
      }
    }
    if(count == 0) {
      throw shift::core::Exception(prefix + "body " + bodies_[b]->get_name() +
                                   " has no markers");
    }
    body_offsets_.push_back(body_offsets_.back() +
                            static_cast<uint32_t>(count));
  }
}

void shift::core::terrain::Sph::DisplaceSoil(shift::core::sph::State& state) {
  const ksl_SE3_t frame = get_particle_frame();
  std::vector<uint32_t> ids;
  for(size_t b = 0; b < bodies_.size(); b++) {
    for(collision_geometry::Base* geometry : body_geometry_[b]) {
      /* its box along the axes of the particles' frame */
      double lo[3], hi[3];
      for(int a = 0; a < 3; a++) {
        for(int side = 0; side < 2; side++) {
          const double sign = side == 0 ? -1.0 : 1.0;
          const ksl_vec3_t direction = {{sign * frame.R.v[a].x,
                                         sign * frame.R.v[a].y,
                                         sign * frame.R.v[a].z}};
          ksl_vec3_t local, global;
          int hint = 0;
          geometry->Support(direction, local, global, hint);
          const ksl_vec3_t r = {{global.x - frame.t.x, global.y - frame.t.y,
                                 global.z - frame.t.z}};
          (side == 0 ? lo : hi)[a] = ksl_dot_vv(&frame.R.v[a], &r);
        }
      }
      for(size_t k = 0; k < state.size(); k++) {
        if(state.kind[k] & shift::core::sph::State::kMarker) {
          continue;
        }
        const std::array<float, 4>& p = state.position[k];
        bool in_box = true;
        for(int a = 0; a < 3; a++) {
          in_box = in_box && p[a] >= lo[a] && p[a] <= hi[a];
        }
        if(!in_box) {
          continue;
        }
        ksl_vec3_t point = frame.t;
        for(int a = 0; a < 3; a++) {
          for(int m = 0; m < 3; m++) {
            point.at[m] += frame.R.v[a].at[m] * p[a];
          }
        }
        double t;
        ksl_vec3_t normal;
        if(geometry->RayCast(point, frame.R.v2, 1e-9 * spacing_, t, normal) &&
           t <= 0.0) {
          ids.push_back(state.id[k]);
        }
      }
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  if(!ids.empty()) {
    state.RemoveParticles(ids);
    particle_count_ -= ids.size();
    displaced_count_ = ids.size();
    SHIFT_INFO("Terrain " + get_name() + ": " + std::to_string(ids.size()) +
               " particles inside the bodies taken out");
  }
}

void shift::core::terrain::Sph::BodyPoses(
  std::vector<shift::core::sph::BodyPose>& poses) {
  const ksl_SE3_t& d = get_pose();
  poses.resize(bodies_.size());
  for(size_t b = 0; b < bodies_.size(); b++) {
    if(load_indices_[b] == -2) {
      load_indices_[b] = context_->GetBodyLoadIndex(bodies_[b]);
    }
    const ksl_SE3_t pose =
      context_->get_body_origin_pose(bodies_[b], context_->get_dhva());
    const ksl_screw_t velocity = context_->GetBodyVelocity(load_indices_[b]);
    /* the velocity of the body's origin, v + w x t */
    ksl_vec3_t v;
    ksl_cross_vv(&velocity.ang, &pose.t, &v);
    for(int k = 0; k < 3; k++) {
      v.at[k] += velocity.lin.at[k];
    }
    const ksl_vec3_t r = {{pose.t.x - d.t.x, pose.t.y - d.t.y,
                           pose.t.z - d.t.z}};
    shift::core::sph::BodyPose& p = poses[b];
    for(int row = 0; row < 3; row++) {
      for(int column = 0; column < 3; column++) {
        p.R[3 * column + row] = ksl_dot_vv(&d.R.v[row], &pose.R.v[column]);
      }
      p.t[row] = ksl_dot_vv(&d.R.v[row], &r) - origin_[row];
      p.v[row] = ksl_dot_vv(&d.R.v[row], &v);
      p.w[row] = ksl_dot_vv(&d.R.v[row], &velocity.ang);
    }
  }
}

void shift::core::terrain::Sph::Start() {
  /* gravity in the terrain frame: R^T g */
  const ksl_vec3_t* g = context_->get_model_parameters().get_gravity();
  const ksl_SE3_t& d = get_pose();
  settings_.gravity = {static_cast<float>(ksl_dot_vv(&d.R.v0, g)),
                       static_cast<float>(ksl_dot_vv(&d.R.v1, g)),
                       static_cast<float>(ksl_dot_vv(&d.R.v2, g))};
  /* the particles and walls, and the bodies' markers */
  shift::core::sph::State state = state_;
  SampleBodies(state);
  DisplaceSoil(state);
  if(!bodies_.empty()) {
    SHIFT_INFO("Terrain " + get_name() + ": " +
               std::to_string(state.size() - state_.size()) +
               " markers on " + std::to_string(bodies_.size()) + " bodies");
  }
  BodyPoses(last_poses_);
  if(device_ == "gpu") {
    device_ptr_ = context_->GetGpuDevice();
    solver_ = std::make_unique<shift::core::sph::GpuSolver>(
      *device_ptr_, settings_, materials_, state);
  } else {
    solver_ = std::make_unique<shift::core::sph::CpuSolver>(settings_,
                                                            materials_, state);
  }
  solver_->GetState(state_);
}

void shift::core::terrain::Sph::AcceptStep() {
  const double t = context_->get_force_time();
  if(!started_) {
    Start();
    started_ = true;
    time_ = last_time_ = t;
    next_landing_ = t + landing_interval_;
    solver_->set_time(t);
    return;
  }
  if(!(t > last_time_)) {
    return;
  }
  /* to t in equal substeps no longer than the stable step; the bodies'
  markers follow them from the last accepted step to this one */
  const double duration = t - last_time_;
  std::vector<shift::core::sph::BodyPose> poses;
  if(!bodies_.empty()) {
    BodyPoses(poses);
    solver_->SetBodyPaths(last_poses_, poses, last_time_, t);
    solver_->ClearImpulses();
  }
  const long n =
    std::max(1L, static_cast<long>(std::ceil(duration / step_ - 1e-9)));
  solver_->Steps(static_cast<float>(duration / n), n);
  /* the float substeps sum to about duration; the time is t */
  solver_->set_time(t);
  time_ = last_time_ = t;
  if(!bodies_.empty()) {
    UpdateBodyLoads(duration);
    last_poses_.swap(poses);
  }
  if(t >= next_landing_) {
    next_landing_ = t + landing_interval_;
    Land();
  }
}

/*!
The particles are read back to find them; for the soil of an excavator's
pit, a few times a second, that is a small part of the work
*/
void shift::core::terrain::Sph::Land() {
  shift::core::sph::State state;
  solver_->GetState(state);
  Basic* holder = dynamic_cast<Basic*>(get_parent_terrain());
  Scm* scm = dynamic_cast<Scm*>(holder);
  std::vector<uint32_t> ids;
  double lost = 0.0;
  for(size_t k = 0; k < state.size(); k++) {
    if(state.kind[k] & shift::core::sph::State::kMarker) {
      continue;
    }
    const std::array<float, 4>& p = state.position[k];
    const double x = origin_[0] + p[0], y = origin_[1] + p[1];
    const double z = origin_[2] + p[2];
    if(Contains(x, y)) {
      continue;
    }
    double z_x, z_y, surface;
    if(holder) {
      const Base* region = holder->RegionAt(x, y);
      if(region && region != this) {
        continue;
      }
      surface = holder->SurfaceZ(x, y, z_x, z_y);
    } else {
      surface = LocalZ(x, y, z_x, z_y);
    }
    if(!(z >= surface)) {
      continue;
    }
    ids.push_back(state.id[k]);
    const double volume = p[3] / materials_[state.kind[k]].density;
    if(scm && scm->Deposit(x, y, volume)) {
      landed_volume_ += volume;
    } else {
      lost += volume;
    }
  }
  if(ids.empty()) {
    return;
  }
  if(lost > 0.0 && lost_volume_ == 0.0) {
    SHIFT_WARNING("Terrain " + get_name() + ": particles left the soil where "
                  "no SCM terrain takes them; their volume is lost "
                  "(get_lost_volume)");
  }
  lost_volume_ += lost;
  solver_->RemoveParticles(ids);
  particle_count_ -= ids.size();
}

/*!
The mean force and moment of the impulses over duration, from the frame of
the particles (moment about their origin) to the inertial frame (moment about
its origin)
*/
void shift::core::terrain::Sph::UpdateBodyLoads(double duration) {
  std::vector<std::array<double, 6>> impulses;
  solver_->GetImpulses(impulses);
  const ksl_SE3_t& d = get_pose();
  /* the origin of the particles in the inertial frame */
  ksl_vec3_t q = d.t;
  for(int a = 0; a < 3; a++) {
    for(int k = 0; k < 3; k++) {
      q.at[k] += d.R.v[a].at[k] * origin_[a];
    }
  }
  for(size_t b = 0; b < bodies_.size(); b++) {
    ksl_coscrew_t& load = body_loads_[b];
    ksl_vec3_t moment;
    for(int k = 0; k < 3; k++) {
      load.lin.at[k] = 0.0;
      moment.at[k] = 0.0;
      for(int a = 0; a < 3; a++) {
        load.lin.at[k] += d.R.v[a].at[k] * impulses[b][a] / duration;
        moment.at[k] += d.R.v[a].at[k] * impulses[b][3 + a] / duration;
      }
    }
    ksl_vec3_t arm;
    ksl_cross_vv(&q, &load.lin, &arm);
    for(int k = 0; k < 3; k++) {
      load.ang.at[k] = moment.at[k] + arm.at[k];
      body_impulses_[b].lin.at[k] += duration * load.lin.at[k];
      body_impulses_[b].ang.at[k] += duration * load.ang.at[k];
    }
  }
}

void shift::core::terrain::Sph::UpdateOutput() {
  if(solver_) {
    solver_->GetState(state_);
    UpdateSurface();
  }
}

void shift::core::terrain::Sph::UpdateSurface() {
  const double s = spacing_;
  surface_ = floor_;
  std::vector<char> filled(surface_.size(), 0);
  for(size_t k = 0; k < state_.size(); k++) {
    if(state_.kind[k] & shift::core::sph::State::kMarker) {
      continue;
    }
    const std::array<float, 4>& p = state_.position[k];
    const long i = static_cast<long>(std::floor(p[0] / s));
    const long j = static_cast<long>(std::floor(p[1] / s));
    if(i < 0 || i >= columns_x_ || j < 0 || j >= columns_y_) {
      continue;
    }
    const size_t c = size_t(j) * columns_x_ + i;
    const double top = p[2] - 0.5 * s;
    if(!filled[c] || top < surface_[c]) {
      surface_[c] = top;
      filled[c] = 1;
    }
  }
}

double shift::core::terrain::Sph::SurfaceZ(double x, double y, double& z_x,
                                           double& z_y) const {
  if(surface_.empty() || !Contains(x, y)) {
    return LocalZ(x, y, z_x, z_y);
  }
  /* bilinear between the centres of the columns, held at the edges */
  const double s = spacing_;
  const double u = std::clamp((x - origin_[0]) / s - 0.5, 0.0,
                              double(columns_x_ - 1));
  const double v = std::clamp((y - origin_[1]) / s - 0.5, 0.0,
                              double(columns_y_ - 1));
  const long i = std::min(static_cast<long>(u), std::max(0L, columns_x_ - 2));
  const long j = std::min(static_cast<long>(v), std::max(0L, columns_y_ - 2));
  const long i1 = std::min(i + 1, columns_x_ - 1);
  const long j1 = std::min(j + 1, columns_y_ - 1);
  const double a = u - i, b = v - j;
  auto at = [&](long ii, long jj) {
    return surface_[size_t(jj) * columns_x_ + ii];
  };
  const double z00 = at(i, j), z10 = at(i1, j), z01 = at(i, j1),
               z11 = at(i1, j1);
  z_x = ((z10 - z00) * (1.0 - b) + (z11 - z01) * b) / s;
  z_y = ((z01 - z00) * (1.0 - a) + (z11 - z10) * a) / s;
  return (z00 * (1.0 - a) + z10 * a) * (1.0 - b) +
         (z01 * (1.0 - a) + z11 * a) * b;
}
