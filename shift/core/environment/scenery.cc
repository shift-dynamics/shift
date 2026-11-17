#include "shift/core/environment/scenery.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_map>

#include <ksl/mat3x3.h>
#include <ksl/matrix.h>
#include <ksl/vec3.h>

#include "shift/core/environment/surface_material.h"
#include "shift/core/exception.h"
#include "shift/core/terrain/basic.h"
#include "shift/core/terrain/material_map.h"

void shift::core::environment::Scenery::DeclareParameters() {
  DeclareParameter("name", &name_, "name of the scenery", true);
  DeclareParameter("terrain", &terrain_,
                   "name of the terrain the scenery stands on", true);
  DeclareParameter("geometry", &geometry_,
                   "the meshes of the items, one chosen at random for each, "
                   "with z down and the origin where it meets the ground",
                   true);
  DeclareParameter("materials", &materials_,
                   "the surface materials of the terrain's material map "
                   "where items are placed; optional, anywhere if none",
                   false);
  DeclareParameter("margin", &margin_,
                   "least distance (m) from a surface material not in "
                   "materials; defaults to 0",
                   false);
  DeclareParameter("density", &density_,
                   "items tried per square meter of the terrain", true);
  DeclareParameter("spacing", &spacing_,
                   "least distance (m) between two items, and from the items "
                   "of the sceneries in avoid; defaults to 0",
                   false);
  DeclareParameter("avoid", &avoid_,
                   "names of sceneries given before this one whose items "
                   "this one keeps spacing away from; optional",
                   false);
  DeclareParameter("scale", &scale_,
                   "least and greatest scale factor of the meshes, [min, "
                   "max]; defaults to [1, 1]",
                   false)
    .set_row_dim(2);
  DeclareParameter("tilt", &tilt_,
                   "how far the items lean with the slope of the ground: 0 "
                   "upright, 1 normal to the surface; defaults to 0",
                   false);
  DeclareParameter("depth", &depth_,
                   "depth (m) of the origin of the meshes below the surface, "
                   "so that they do not stand clear of a slope; defaults to 0",
                   false);
  DeclareParameter("seed", &seed_,
                   "seed of the random placement; defaults to 1", false);
  DeclareParameter("draw_distance", &draw_distance_,
                   "distance (m) from the camera beyond which the viewer's "
                   "rasterizer leaves the items out; defaults to 0, none",
                   false);
}

void shift::core::environment::Scenery::ActivateObjects(bool parent_is_active) {
  geometry_.ActivateObjects(parent_is_active);
}

void shift::core::environment::Scenery::Check() {
  const std::string prefix = "scenery " + name_ + ": ";
  if(geometry_.get_data().empty()) {
    throw shift::core::Exception(prefix + "geometry is empty");
  }
  if(!(density_ >= 0.0)) {
    throw shift::core::Exception(prefix + "density must not be negative");
  }
  if(!(spacing_ >= 0.0) || !(margin_ >= 0.0)) {
    throw shift::core::Exception(prefix +
                                 "spacing and margin must not be negative");
  }
  if(scale_.size() != 2 || !(scale_[0] > 0.0) || !(scale_[1] >= scale_[0])) {
    throw shift::core::Exception(
      prefix + "scale must be [min, max] with 0 < min <= max");
  }
  if(!(draw_distance_ >= 0.0)) {
    throw shift::core::Exception(prefix + "draw_distance must not be "
                                          "negative");
  }
  if(!(tilt_ >= 0.0 && tilt_ <= 1.0)) {
    throw shift::core::Exception(prefix + "tilt must be from 0 to 1");
  }
}

/* uniform in [0, 1) from 32 random bits, the same on every platform (unlike
std::uniform_real_distribution) */
static double Uniform(std::mt19937& random) {
  return double(random()) / 4294967296.0;
}

void shift::core::environment::Scenery::Place(
  const shift::core::terrain::Basic& terrain,
  const std::vector<const Scenery*>& avoided) {
  const std::string prefix = "scenery " + name_ + ": ";
  items_.clear();
  /* the area tried, in the terrain frame */
  double x0 = 0.0, x1 = 0.0, y0 = 0.0, y1 = 0.0;
  if(terrain.has_elevation_map()) {
    x1 = terrain.get_x_extent();
    y1 = terrain.get_y_extent();
  } else {
    throw shift::core::Exception(prefix + "terrain " + terrain.get_name() +
                                 " has no elevation map to stand on");
  }

  /* the materials where items may stand, by index in the material map */
  const shift::core::terrain::MaterialMap* map = terrain.get_material_map();
  std::vector<bool> allowed;
  if(map && !materials_.empty()) {
    for(const auto* material : map->get_materials()) {
      allowed.push_back(std::find(materials_.begin(), materials_.end(),
                                  material->get_name()) != materials_.end());
    }
  }
  const auto allowed_at = [&](double x, double y) {
    if(allowed.empty()) {
      return true;
    }
    const int index = map->MaterialIndex(x, y);
    return index >= 0 && allowed[size_t(index)];
  };

  /* the items placed, and those avoided, by cells of spacing in the
  inertial xy plane */
  const double cell = std::max(spacing_, 1e-9);
  std::unordered_map<int64_t, std::vector<ksl_vec3_t>> cells;
  const auto key = [cell](double x, double y) {
    const int64_t i = int64_t(std::floor(x / cell));
    const int64_t j = int64_t(std::floor(y / cell));
    return (i << 32) ^ (j & 0xffffffff);
  };
  const auto clear = [&](const ksl_vec3_t& p) {
    if(spacing_ <= 0.0) {
      return true;
    }
    const int64_t i = int64_t(std::floor(p.x / cell));
    const int64_t j = int64_t(std::floor(p.y / cell));
    for(int64_t di = -1; di <= 1; di++) {
      for(int64_t dj = -1; dj <= 1; dj++) {
        const auto it = cells.find(((i + di) << 32) ^ ((j + dj) & 0xffffffff));
        if(it == cells.end()) {
          continue;
        }
        for(const auto& q : it->second) {
          const double dx = p.x - q.x, dy = p.y - q.y;
          if(dx * dx + dy * dy < spacing_ * spacing_) {
            return false;
          }
        }
      }
    }
    return true;
  };
  for(const auto* other : avoided) {
    for(const auto& item : other->get_items()) {
      cells[key(item.pose.t.x, item.pose.t.y)].push_back(item.pose.t);
    }
  }

  std::mt19937 random(static_cast<uint32_t>(seed_));
  const double area = (x1 - x0) * (y1 - y0);
  const long tries = std::lround(density_ * area);
  const int variants = static_cast<int>(geometry_.get_data().size());
  const ksl_SE3_t& terrain_pose = terrain.get_pose();
  for(long n = 0; n < tries; n++) {
    /* every try draws the same numbers, kept or not, so that one item does
    not move the others */
    const double x = x0 + (x1 - x0) * Uniform(random);
    const double y = y0 + (y1 - y0) * Uniform(random);
    const double yaw = 2.0 * M_PI * Uniform(random);
    const double scale = scale_[0] + (scale_[1] - scale_[0]) * Uniform(random);
    const int variant =
      std::min(static_cast<int>(Uniform(random) * variants), variants - 1);
    if(!terrain.Contains(x, y) || !allowed_at(x, y)) {
      continue;
    }
    if(margin_ > 0.0 && !allowed.empty()) {
      bool inside = true;
      for(int k = 0; k < 8 && inside; k++) {
        const double a = k * M_PI / 4.0;
        inside = allowed_at(x + margin_ * std::cos(a),
                            y + margin_ * std::sin(a));
      }
      if(!inside) {
        continue;
      }
    }
    double z_x = 0.0, z_y = 0.0;
    const double z = terrain.LocalZ(x, y, z_x, z_y);
    /* the vertical (-z) leaned toward the upward normal of the surface */
    ksl_vec3_t up = ksl_vec3(z_x, z_y, -1.0);
    ksl_vec3_normalize(&up);
    const ksl_vec3_t vertical = ksl_vec3(0.0, 0.0, -1.0);
    ksl_vec3_t axis;
    ksl_cross_vv(&vertical, &up, &axis);
    const double sine = ksl_vec3_l2norm(&axis);
    ksl_SE3_t local = ksl_SE3_identity();
    if(sine > 1e-12 && tilt_ > 0.0) {
      ksl_vec3_scale(&axis, 1.0 / sine);
      const double angle = tilt_ * std::atan2(sine, ksl_dot_vv(&vertical, &up));
      ksl_mat3x3_setFromAxisAngle(&local.R, &axis, angle);
    }
    local.t = ksl_vec3(x, y, z + depth_);
    const double dc[2] = {std::sin(yaw), std::cos(yaw)};
    ksl_SE3_t turned;
    ksl_product_ddrz(&local, dc, &turned);
    Item item;
    item.variant = variant;
    item.scale = scale;
    ksl_product_dd(&terrain_pose, &turned, &item.pose);
    if(!clear(item.pose.t)) {
      continue;
    }
    cells[key(item.pose.t.x, item.pose.t.y)].push_back(item.pose.t);
    items_.push_back(item);
  }
}
