#include "shift/core/dem/dem.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>

#include "shift/core/collision_geometry/fitting.h"
#include "shift/core/exception.h"
#include "shift/core/geometry/mass_properties.h"

static const double kPi = 3.14159265358979323846;

shift::core::dem::Template shift::core::dem::Template::SingleSphere(
  float radius, float density, uint32_t material) {
  Template t;
  Sphere s;
  s.radius = radius;
  s.material = material;
  t.spheres.push_back(s);
  t.mass = static_cast<float>(4.0 / 3.0 * kPi * double(radius) * radius *
                              radius * density);
  const float moment = 0.4f * t.mass * radius * radius;
  t.inertia = {moment, moment, moment};
  return t;
}

float shift::core::dem::Hull::BoundingRadius() const {
  float most = 0.0f;
  for(const auto& v : vertices) {
    most = std::max(most, std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
  }
  return most;
}

/* the convex hull of points with qhull, its triangles wound
counterclockwise seen from outside (qhull winds them either way: each away
from the mean of the vertices, inside the hull), and its mass properties of
unit density; throws if it has no volume */
static void HullOfPoints(const std::vector<std::array<float, 3>>& points,
                         shift::core::collision_geometry::ConvexHullData& hull,
                         shift::core::geometry::MassProperties& properties) {
  std::vector<ksl_vec3_t> given(points.size());
  for(size_t k = 0; k < points.size(); k++) {
    given[k] = {{points[k][0], points[k][1], points[k][2]}};
  }
  if(!shift::core::collision_geometry::ComputeConvexHull(given, hull)) {
    throw shift::core::Exception("DEM: points with no hull of volume");
  }
  double mean[3] = {0.0, 0.0, 0.0};
  for(const ksl_vec3_t& v : hull.vertices) {
    for(int k = 0; k < 3; k++) {
      mean[k] += v.at[k] / hull.vertices.size();
    }
  }
  for(size_t f = 0; f + 2 < hull.faces.size(); f += 3) {
    const ksl_vec3_t& a = hull.vertices[hull.faces[f]];
    const ksl_vec3_t& b = hull.vertices[hull.faces[f + 1]];
    const ksl_vec3_t& c = hull.vertices[hull.faces[f + 2]];
    double e1[3], e2[3], m[3];
    for(int k = 0; k < 3; k++) {
      e1[k] = b.at[k] - a.at[k];
      e2[k] = c.at[k] - a.at[k];
      m[k] = a.at[k] - mean[k];
    }
    const double outward = m[0] * (e1[1] * e2[2] - e1[2] * e2[1]) +
                           m[1] * (e1[2] * e2[0] - e1[0] * e2[2]) +
                           m[2] * (e1[0] * e2[1] - e1[1] * e2[0]);
    if(outward < 0.0) {
      std::swap(hull.faces[f + 1], hull.faces[f + 2]);
    }
  }
  if(!shift::core::geometry::ComputeMassProperties(hull.vertices, hull.faces,
                                                   properties) ||
     !(properties.volume > 0.0)) {
    throw shift::core::Exception("DEM: points with no hull of volume");
  }
}

shift::core::dem::Hull shift::core::dem::Hull::Of(
  const std::vector<std::array<float, 3>>& points, uint32_t material,
  std::vector<std::array<uint32_t, 3>>* triangles) {
  shift::core::collision_geometry::ConvexHullData data;
  shift::core::geometry::MassProperties properties;
  HullOfPoints(points, data, properties);
  if(data.vertices.size() > kMaxHullVertices) {
    throw shift::core::Exception(
      "DEM: a hull of " + std::to_string(data.vertices.size()) +
      " vertices, more than " + std::to_string(kMaxHullVertices));
  }
  Hull hull;
  hull.material = material;
  hull.radius = static_cast<float>(
    std::cbrt(3.0 * properties.volume / (4.0 * kPi)));
  for(const ksl_vec3_t& v : data.vertices) {
    hull.vertices.push_back({static_cast<float>(v.at[0]),
                             static_cast<float>(v.at[1]),
                             static_cast<float>(v.at[2])});
  }
  const float scale = hull.BoundingRadius();
  /* the planes of the triangles, each plane once */
  for(size_t f = 0; f + 2 < data.faces.size(); f += 3) {
    const ksl_vec3_t& a = data.vertices[data.faces[f]];
    const ksl_vec3_t& b = data.vertices[data.faces[f + 1]];
    const ksl_vec3_t& c = data.vertices[data.faces[f + 2]];
    double e1[3], e2[3], n[3];
    for(int k = 0; k < 3; k++) {
      e1[k] = b.at[k] - a.at[k];
      e2[k] = c.at[k] - a.at[k];
    }
    n[0] = e1[1] * e2[2] - e1[2] * e2[1];
    n[1] = e1[2] * e2[0] - e1[0] * e2[2];
    n[2] = e1[0] * e2[1] - e1[1] * e2[0];
    const double size = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if(triangles) {
      triangles->push_back({data.faces[f], data.faces[f + 1],
                            data.faces[f + 2]});
    }
    if(!(size > 0.0)) {
      continue;
    }
    const std::array<float, 4> plane = {
      static_cast<float>(n[0] / size), static_cast<float>(n[1] / size),
      static_cast<float>(n[2] / size),
      static_cast<float>((n[0] * a.at[0] + n[1] * a.at[1] + n[2] * a.at[2]) /
                         size)};
    bool seen = false;
    for(const auto& q : hull.planes) {
      seen = seen || (plane[0] * q[0] + plane[1] * q[1] + plane[2] * q[2] >
                        1.0f - 1e-6f &&
                      std::fabs(plane[3] - q[3]) <= 1e-5f * scale);
    }
    if(!seen) {
      hull.planes.push_back(plane);
    }
  }
  if(hull.planes.size() > kMaxHullPlanes) {
    throw shift::core::Exception("DEM: a hull of more than " +
                                 std::to_string(kMaxHullPlanes) + " faces");
  }
  /* the polygon of each plane: its vertices, counterclockwise about its
  normal */
  for(const auto& p : hull.planes) {
    const std::array<float, 3> axis =
      std::fabs(p[0]) < 0.57735f ? std::array<float, 3>{1.0f, 0.0f, 0.0f}
                                 : std::array<float, 3>{0.0f, 1.0f, 0.0f};
    float t1[3] = {p[1] * axis[2] - p[2] * axis[1],
                   p[2] * axis[0] - p[0] * axis[2],
                   p[0] * axis[1] - p[1] * axis[0]};
    const float size = std::sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
    for(float& c : t1) {
      c /= size;
    }
    const float t2[3] = {p[1] * t1[2] - p[2] * t1[1],
                         p[2] * t1[0] - p[0] * t1[2],
                         p[0] * t1[1] - p[1] * t1[0]};
    std::vector<uint32_t> face;
    float center[3] = {0.0f, 0.0f, 0.0f};
    for(uint32_t v = 0; v < hull.vertices.size(); v++) {
      const auto& x = hull.vertices[v];
      if(std::fabs(p[0] * x[0] + p[1] * x[1] + p[2] * x[2] - p[3]) <=
         1e-5f * scale) {
        face.push_back(v);
        for(int k = 0; k < 3; k++) {
          center[k] += x[k];
        }
      }
    }
    for(float& c : center) {
      c /= face.size();
    }
    auto angle = [&](uint32_t v) {
      const auto& x = hull.vertices[v];
      const float d[3] = {x[0] - center[0], x[1] - center[1], x[2] - center[2]};
      return std::atan2(d[0] * t2[0] + d[1] * t2[1] + d[2] * t2[2],
                        d[0] * t1[0] + d[1] * t1[1] + d[2] * t1[2]);
    };
    std::sort(face.begin(), face.end(),
              [&](uint32_t a, uint32_t b) { return angle(a) < angle(b); });
    if(face.size() < 3 || face.size() > kMaxFaceVertices) {
      throw shift::core::Exception(
        "DEM: a face of a hull of " + std::to_string(face.size()) +
        " vertices, not 3 to " + std::to_string(kMaxFaceVertices));
    }
    hull.faces.push_back(face);
  }
  return hull;
}

shift::core::dem::Template shift::core::dem::Template::OfHull(
  const std::vector<std::array<float, 3>>& points, float density,
  uint32_t material) {
  shift::core::collision_geometry::ConvexHullData data;
  shift::core::geometry::MassProperties properties;
  HullOfPoints(points, data, properties);
  /* the principal axes: the eigenvectors of the inertia (at[column][row]),
  the columns of V */
  ksl_mat3x3_t inertia, axes;
  const double entries[3][3] = {{properties.Ixx, properties.Ixy, properties.Izx},
                                {properties.Ixy, properties.Iyy, properties.Iyz},
                                {properties.Izx, properties.Iyz, properties.Izz}};
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      inertia.at[j][i] = entries[i][j];
    }
  }
  shift::core::collision_geometry::Jacobi(inertia, axes);
  Template t;
  t.mass = static_cast<float>(density * properties.volume);
  for(int k = 0; k < 3; k++) {
    t.inertia[k] = static_cast<float>(density * inertia.at[k][k]);
  }
  /* the hull's vertices along the principal axes from the centroid */
  std::vector<std::array<float, 3>> moved;
  for(const ksl_vec3_t& v : data.vertices) {
    std::array<float, 3> x;
    for(int k = 0; k < 3; k++) {
      double sum = 0.0;
      for(int i = 0; i < 3; i++) {
        sum += axes.at[k][i] * (v.at[i] - properties.centroid.at[i]);
      }
      x[k] = static_cast<float>(sum);
    }
    moved.push_back(x);
  }
  t.hull = Hull::Of(moved, material, &t.mesh_triangles);
  t.mesh_vertices = t.hull.vertices;
  return t;
}

/* the depth of x inside the planes of a hull: the least distance to a
plane, negative outside */
static float DepthInHull(const shift::core::dem::Hull& hull,
                         const std::array<float, 3>& x) {
  float least = std::numeric_limits<float>::infinity();
  for(const auto& p : hull.planes) {
    least = std::min(least, p[3] - (p[0] * x[0] + p[1] * x[1] + p[2] * x[2]));
  }
  return least;
}

shift::core::dem::Template shift::core::dem::Template::ClumpOfHull(
  const Template& hull, uint32_t count, uint32_t material) {
  if(hull.hull.vertices.empty() || count == 0) {
    throw shift::core::Exception("DEM: a clump of no hull or no spheres");
  }
  std::array<float, 3> low = {1e30f, 1e30f, 1e30f},
                       high = {-1e30f, -1e30f, -1e30f};
  for(const auto& v : hull.hull.vertices) {
    for(int k = 0; k < 3; k++) {
      low[k] = std::min(low[k], v[k]);
      high[k] = std::max(high[k], v[k]);
    }
  }
  /* the points of a lattice of n along the longest side, inside the hull */
  auto lattice = [&](int n, std::vector<std::array<float, 3>>& points) {
    float extent = 0.0f;
    for(int k = 0; k < 3; k++) {
      extent = std::max(extent, high[k] - low[k]);
    }
    const float h = extent / n;
    for(float z = low[2] + 0.5f * h; z < high[2]; z += h) {
      for(float y = low[1] + 0.5f * h; y < high[1]; y += h) {
        for(float x = low[0] + 0.5f * h; x < high[0]; x += h) {
          if(DepthInHull(hull.hull, {x, y, z}) > 0.0f) {
            points.push_back({x, y, z});
          }
        }
      }
    }
  };
  std::vector<std::array<float, 3>> centers, samples;
  lattice(12, centers);
  lattice(32, samples);
  std::vector<bool> covered(samples.size(), false);
  Template t = hull;
  t.hull = Hull();
  for(uint32_t k = 0; k < count; k++) {
    size_t best = centers.size(), most = 0;
    for(size_t c = 0; c < centers.size(); c++) {
      const float r = DepthInHull(hull.hull, centers[c]);
      size_t n = 0;
      for(size_t i = 0; i < samples.size(); i++) {
        const float dx = samples[i][0] - centers[c][0],
                    dy = samples[i][1] - centers[c][1],
                    dz = samples[i][2] - centers[c][2];
        n += !covered[i] && dx * dx + dy * dy + dz * dz <= r * r;
      }
      if(n > most) {
        most = n;
        best = c;
      }
    }
    if(best == centers.size()) {
      break;
    }
    Sphere sphere;
    sphere.center = centers[best];
    sphere.radius = DepthInHull(hull.hull, centers[best]);
    sphere.material = material;
    t.spheres.push_back(sphere);
    for(size_t i = 0; i < samples.size(); i++) {
      const float dx = samples[i][0] - sphere.center[0],
                  dy = samples[i][1] - sphere.center[1],
                  dz = samples[i][2] - sphere.center[2];
      covered[i] = covered[i] || dx * dx + dy * dy + dz * dz <=
                                   sphere.radius * sphere.radius;
    }
  }
  if(t.spheres.empty()) {
    throw shift::core::Exception("DEM: a hull too thin to fill");
  }
  return t;
}

std::vector<std::array<float, 3>> shift::core::dem::RockPoints(uint32_t seed,
                                                              float size,
                                                              uint32_t count,
                                                              float jitter) {
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> uniform(0.0f, 1.0f);
  const float axes[3] = {size * (0.7f + 0.6f * uniform(generator)),
                         size * (0.7f + 0.6f * uniform(generator)),
                         size * (0.7f + 0.6f * uniform(generator))};
  const float golden = static_cast<float>(kPi * (3.0 - std::sqrt(5.0)));
  std::vector<std::array<float, 3>> points;
  for(uint32_t k = 0; k < count; k++) {
    const float z = 1.0f - 2.0f * (k + 0.5f) / count;
    const float ring = std::sqrt(std::max(0.0f, 1.0f - z * z));
    const float angle = golden * k;
    const float scale =
      1.0f + jitter * (2.0f * uniform(generator) - 1.0f);
    points.push_back({axes[0] * scale * ring * std::cos(angle),
                      axes[1] * scale * ring * std::sin(angle),
                      axes[2] * scale * z});
  }
  return points;
}

/* the spheres of a template as the contact law, the step and the margin
see it: its spheres, or for a hull the sphere of its volume */
static std::vector<shift::core::dem::Sphere> ContactSpheres(
  const shift::core::dem::Template& t) {
  if(t.hull.vertices.empty()) {
    return t.spheres;
  }
  shift::core::dem::Sphere s;
  s.radius = t.hull.radius;
  s.material = t.hull.material;
  return {s};
}

void shift::core::dem::State::resize(size_t n) {
  template_index.resize(n);
  position.resize(n);
  orientation.resize(n);
  velocity.resize(n);
  angular_velocity.resize(n);
}

void shift::core::dem::State::AddGrain(uint32_t index,
                                       const std::array<double, 3>& x) {
  template_index.push_back(index);
  position.push_back(x);
  orientation.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  velocity.push_back({0.0f, 0.0f, 0.0f});
  angular_velocity.push_back({0.0f, 0.0f, 0.0f});
}

static const char* kGrainColumns =
  "grain,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz";

void shift::core::dem::WriteGrains(const std::string& path,
                                   const State& state,
                                   const std::vector<std::string>& names) {
  std::ofstream file(path);
  if(!file) {
    throw shift::core::Exception("DEM: can't write the grains to " + path);
  }
  file << "# grains of the DEM: shape, position (m), orientation (unit "
          "quaternion, scalar last), velocity (m/s), angular velocity in "
          "principal axes (rad/s)\n"
       << kGrainColumns << "\n";
  char line[512];
  for(size_t g = 0; g < state.size(); g++) {
    const uint32_t t = state.template_index[g];
    if(t >= names.size()) {
      throw shift::core::Exception("DEM: a grain of no shape");
    }
    const auto& p = state.position[g];
    const auto& q = state.orientation[g];
    const auto& v = state.velocity[g];
    const auto& w = state.angular_velocity[g];
    /* 17 digits read a double back exactly, 9 a float */
    std::snprintf(line, sizeof(line),
                  ",%.17g,%.17g,%.17g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                  "%.9g,%.9g,%.9g\n",
                  p[0], p[1], p[2], double(q[0]), double(q[1]), double(q[2]),
                  double(q[3]), double(v[0]), double(v[1]), double(v[2]),
                  double(w[0]), double(w[1]), double(w[2]));
    file << names[t] << line;
  }
  file.close();
  if(!file) {
    throw shift::core::Exception("DEM: can't write the grains to " + path);
  }
}

void shift::core::dem::ReadGrains(const std::string& path,
                                  const std::vector<std::string>& names,
                                  State& state) {
  std::ifstream file(path);
  if(!file) {
    throw shift::core::Exception("DEM: can't read the grains of " + path);
  }
  std::string line;
  size_t number = 0;
  bool header = false;
  while(std::getline(file, line)) {
    number++;
    if(!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if(line.empty() || line[0] == '#') {
      continue;
    }
    const std::string where =
      "DEM: grains " + path + ", line " + std::to_string(number) + ": ";
    if(!header) {
      if(line != kGrainColumns) {
        throw shift::core::Exception(where + "the columns are not " +
                                     kGrainColumns);
      }
      header = true;
      continue;
    }
    std::istringstream fields(line);
    std::string name, field;
    std::getline(fields, name, ',');
    const auto found = std::find(names.begin(), names.end(), name);
    if(found == names.end()) {
      throw shift::core::Exception(where + "shape '" + name +
                                   "' is not among the model's grains");
    }
    double value[13];
    for(int k = 0; k < 13; k++) {
      char* end = nullptr;
      if(!std::getline(fields, field, ',') ||
         (value[k] = std::strtod(field.c_str(), &end), end == field.c_str()) ||
         *end != '\0' || !std::isfinite(value[k])) {
        throw shift::core::Exception(where + "needs 13 numbers after the "
                                             "shape");
      }
    }
    state.AddGrain(static_cast<uint32_t>(found - names.begin()),
                   {value[0], value[1], value[2]});
    for(int k = 0; k < 4; k++) {
      state.orientation.back()[k] = static_cast<float>(value[3 + k]);
    }
    for(int k = 0; k < 3; k++) {
      state.velocity.back()[k] = static_cast<float>(value[7 + k]);
      state.angular_velocity.back()[k] = static_cast<float>(value[10 + k]);
    }
  }
  if(!header) {
    throw shift::core::Exception("DEM: grains " + path + " has no line of "
                                 "columns " + kGrainColumns);
  }
}

/* the shear modulus of a material */
static double ShearModulus(const shift::core::dem::Material& m) {
  return m.youngs_modulus / (2.0 * (1.0 + m.poisson_ratio));
}

static double EffectiveModulus(const shift::core::dem::Material& a,
                               const shift::core::dem::Material& b) {
  return 1.0 / ((1.0 - double(a.poisson_ratio) * a.poisson_ratio) /
                  a.youngs_modulus +
                (1.0 - double(b.poisson_ratio) * b.poisson_ratio) /
                  b.youngs_modulus);
}

static void CheckMaterial(const std::vector<shift::core::dem::Material>& m,
                          uint32_t k) {
  if(k >= m.size()) {
    throw shift::core::Exception("DEM: a sphere or wall of no material");
  }
}

/* the density of a template: its mass over its spheres' volume */
static double Density(const shift::core::dem::Template& t) {
  double volume = 0.0;
  for(const auto& s : ContactSpheres(t)) {
    volume += 4.0 / 3.0 * kPi * double(s.radius) * s.radius * s.radius;
  }
  return volume > 0.0 ? t.mass / volume : 0.0;
}

double shift::core::dem::RayleighTime(const std::vector<Template>& templates,
                                      const std::vector<Material>& materials) {
  double shortest = std::numeric_limits<double>::infinity();
  for(const Template& t : templates) {
    const double rho = Density(t);
    for(const Sphere& s : ContactSpheres(t)) {
      CheckMaterial(materials, s.material);
      const Material& m = materials[s.material];
      const double time = kPi * s.radius * std::sqrt(rho / ShearModulus(m)) /
                          (0.1631 * m.poisson_ratio + 0.8766);
      shortest = std::min(shortest, time);
    }
  }
  return shortest;
}

double shift::core::dem::CollisionTime(const std::vector<Template>& templates,
                                       const std::vector<Material>& materials,
                                       double speed) {
  double shortest = std::numeric_limits<double>::infinity();
  for(const Template& ta : templates) {
    for(const Sphere& sa : ContactSpheres(ta)) {
      CheckMaterial(materials, sa.material);
      for(const Template& tb : templates) {
        for(const Sphere& sb : ContactSpheres(tb)) {
          CheckMaterial(materials, sb.material);
          const double m = double(ta.mass) * tb.mass / (ta.mass + tb.mass);
          const double r =
            double(sa.radius) * sb.radius / (sa.radius + sb.radius);
          const double e =
            EffectiveModulus(materials[sa.material], materials[sb.material]);
          const double time =
            2.87 * std::pow(m * m / (r * e * e * speed), 0.2);
          shortest = std::min(shortest, time);
        }
      }
    }
  }
  return shortest;
}

double shift::core::dem::StableStep(const std::vector<Template>& templates,
                                    const std::vector<Material>& materials,
                                    double speed, double fraction) {
  return fraction * std::min(RayleighTime(templates, materials),
                             CollisionTime(templates, materials, speed));
}

std::vector<std::array<float, 8>> shift::core::dem::PairTable(
  const Settings& settings, const std::vector<Material>& materials) {
  const size_t n = materials.size();
  std::vector<std::array<float, 8>> table(n * n);
  for(size_t a = 0; a < n; a++) {
    for(size_t b = 0; b < n; b++) {
      const Material& ma = materials[a];
      const Material& mb = materials[b];
      PairProperties p;
      p.restitution = 0.5f * (ma.restitution + mb.restitution);
      p.friction = 0.5f * (ma.friction + mb.friction);
      p.rolling_resistance =
        0.5f * (ma.rolling_resistance + mb.rolling_resistance);
      for(const PairProperties& given : settings.pairs) {
        if((given.a == a && given.b == b) || (given.a == b && given.b == a)) {
          p = given;
        }
      }
      const double e = EffectiveModulus(ma, mb);
      const double g =
        1.0 / (2.0 * (2.0 - ma.poisson_ratio) * (1.0 + ma.poisson_ratio) /
                 ma.youngs_modulus +
               2.0 * (2.0 - mb.poisson_ratio) * (1.0 + mb.poisson_ratio) /
                 mb.youngs_modulus);
      const double log_e = std::log(std::max(double(p.restitution), 1e-12));
      const double beta = log_e / std::sqrt(log_e * log_e + kPi * kPi);
      table[a * n + b] = {static_cast<float>(e),
                          static_cast<float>(g),
                          static_cast<float>(beta),
                          p.friction,
                          p.rolling_resistance,
                          p.restitution,
                          0.0f,
                          0.0f};
    }
  }
  return table;
}

float shift::core::dem::Margin(const Settings& settings,
                               const std::vector<Template>& templates) {
  if(settings.margin > 0.0f) {
    return settings.margin;
  }
  float smallest = std::numeric_limits<float>::infinity();
  for(const Template& t : templates) {
    for(const Sphere& s : ContactSpheres(t)) {
      smallest = std::min(smallest, s.radius);
    }
  }
  return 0.2f * smallest;
}

float shift::core::dem::CellWidth(const Settings& settings,
                                  const std::vector<Template>& templates) {
  float largest = 0.0f;
  for(const Template& t : templates) {
    for(const Sphere& s : t.spheres) {
      largest = std::max(largest, s.radius);
    }
    largest = std::max(largest, t.hull.BoundingRadius());
  }
  return 2.0f * largest + Margin(settings, templates);
}

void shift::core::dem::ToAnchor(double x, float w, int32_t& cell,
                                float& offset) {
  const double c = std::floor(x / w);
  if(!(std::abs(c) < 2.0e9)) {
    throw shift::core::Exception("DEM: a position too far from the origin");
  }
  cell = static_cast<int32_t>(c);
  offset = static_cast<float>(x - c * double(w));
  /* rounding may give w itself */
  if(offset >= w) {
    cell++;
    offset = 0.0f;
  }
}

double shift::core::dem::FromAnchor(int32_t cell, float offset, float w) {
  return double(cell) * double(w) + double(offset);
}

uint32_t shift::core::dem::CellHash(const std::array<int32_t, 3>& c,
                                    uint32_t mask) {
  return ((static_cast<uint32_t>(c[0]) * 73856093u) ^
          (static_cast<uint32_t>(c[1]) * 19349663u) ^
          (static_cast<uint32_t>(c[2]) * 83492791u)) &
         mask;
}

uint32_t shift::core::dem::HashTableSize(size_t count) {
  uint32_t size = 1024;
  while(size < 2 * count && size < (1u << 30)) {
    size <<= 1;
  }
  return size;
}

void shift::core::dem::Subdivide(
  const std::vector<std::array<float, 3>>& vertices,
  const std::vector<std::array<uint32_t, 3>>& triangles, float length,
  std::vector<std::array<float, 3>>& out_vertices,
  std::vector<std::array<uint32_t, 3>>& out_triangles) {
  for(const auto& t : triangles) {
    for(uint32_t v : t) {
      if(v >= vertices.size()) {
        throw shift::core::Exception("DEM: a triangle of no vertex");
      }
    }
  }
  out_vertices = vertices;
  out_triangles.clear();
  std::vector<std::array<uint32_t, 3>> pending(triangles.rbegin(),
                                               triangles.rend());
  auto distance = [&](uint32_t a, uint32_t b) {
    double sum = 0.0;
    for(int k = 0; k < 3; k++) {
      const double d = double(out_vertices[a][k]) - out_vertices[b][k];
      sum += d * d;
    }
    return std::sqrt(sum);
  };
  auto middle = [&](uint32_t a, uint32_t b) {
    std::array<float, 3> m;
    for(int k = 0; k < 3; k++) {
      m[k] = 0.5f * (out_vertices[a][k] + out_vertices[b][k]);
    }
    out_vertices.push_back(m);
    return static_cast<uint32_t>(out_vertices.size() - 1);
  };
  while(!pending.empty()) {
    const std::array<uint32_t, 3> t = pending.back();
    pending.pop_back();
    if(std::max({distance(t[0], t[1]), distance(t[1], t[2]),
                 distance(t[2], t[0])}) <= length) {
      out_triangles.push_back(t);
      continue;
    }
    const uint32_t a = middle(t[0], t[1]), b = middle(t[1], t[2]),
                   c = middle(t[2], t[0]);
    pending.push_back({c, b, t[2]});
    pending.push_back({a, t[1], b});
    pending.push_back({a, b, c});
    pending.push_back({t[0], a, c});
  }
}

void shift::core::dem::Solver::SetBodyPaths(
  const std::vector<shift::core::sph::BodyPose>& start,
  const std::vector<shift::core::sph::BodyPose>& end, double t0, double t1) {
  if(start.size() != bodies_.size() || end.size() != bodies_.size()) {
    throw shift::core::Exception("DEM: a path for each body is needed");
  }
  path_start_ = start;
  path_end_ = end;
  path_t0_ = t0;
  path_t1_ = t1;
}

void shift::core::dem::Solver::AnchoredPosesAt(
  double time, std::vector<std::array<float, 4>>& rows) const {
  rows.assign(7 * bodies_.size(), {0.0f, 0.0f, 0.0f, 0.0f});
  for(size_t b = 0; b < bodies_.size(); b++) {
    shift::core::sph::BodyPose pose = bodies_[b].pose;
    if(!path_end_.empty()) {
      if(path_t1_ > path_t0_) {
        const double duration = path_t1_ - path_t0_;
        pose = shift::core::sph::Interpolate(path_start_[b], path_end_[b],
                                             duration,
                                             (time - path_t0_) / duration);
      } else {
        pose = path_end_[b];
      }
    }
    std::array<float, 4>* row = &rows[7 * b];
    for(int k = 0; k < 3; k++) {
      int32_t cell;
      float offset;
      ToAnchor(pose.t[k], layout_.cell, cell, offset);
      std::memcpy(&row[0][k], &cell, 4);
      row[1][k] = offset;
      for(int c = 0; c < 3; c++) {
        row[2 + c][k] = static_cast<float>(pose.R[3 * c + k]);
      }
      row[5][k] = static_cast<float>(pose.v[k]);
      row[6][k] = static_cast<float>(pose.w[k]);
    }
  }
}

void shift::core::dem::Solver::BodyMassRows(
  const std::vector<std::array<float, 4>>& poses,
  std::vector<std::array<float, 4>>& rows) const {
  rows.assign(5 * bodies_.size(), {0.0f, 0.0f, 0.0f, 0.0f});
  for(size_t b = 0; b < bodies_.size(); b++) {
    const Body& body = bodies_[b];
    if(!(body.mass > 0.0f)) {
      continue;
    }
    std::array<float, 4>* out = &rows[5 * b];
    out[0][0] = body.mass;
    /* the columns of R */
    const std::array<float, 4>* column = &poses[7 * b + 2];
    const auto& i = body.inertia;
    /* the inverse of the symmetric inertia, in double */
    const double a = i[0], d = i[1], f = i[2], bb = i[3], e = i[4], c = i[5];
    const double c00 = d * f - e * e, c01 = c * e - bb * f, c02 = bb * e - c * d,
                 c11 = a * f - c * c, c12 = bb * c - a * e, c22 = a * d - bb * bb;
    const double det = a * c00 + bb * c01 + c * c02;
    const double inv[3][3] = {{c00 / det, c01 / det, c02 / det},
                              {c01 / det, c11 / det, c12 / det},
                              {c02 / det, c12 / det, c22 / det}};
    /* R inv R^T */
    for(int r = 0; r < 3; r++) {
      for(int k = 0; k < 3; k++) {
        double sum = 0.0;
        for(int m = 0; m < 3; m++) {
          for(int n = 0; n < 3; n++) {
            sum += double(column[m][r]) * inv[m][n] * double(column[n][k]);
          }
        }
        out[1 + r][k] = static_cast<float>(sum);
      }
    }
    for(int k = 0; k < 3; k++) {
      out[4][k] = (column[0][k] * body.center_of_mass[0] +
                   column[1][k] * body.center_of_mass[1]) +
                  column[2][k] * body.center_of_mass[2];
    }
  }
}

shift::core::dem::Solver::Solver(const Settings& settings,
                                 const std::vector<Material>& materials,
                                 const std::vector<Template>& templates,
                                 const State& state,
                                 const std::vector<Body>& bodies)
  : settings_(settings), materials_(materials), templates_(templates),
    bodies_(bodies) {
  if(materials.empty() || templates.empty()) {
    throw shift::core::Exception("DEM: no materials or no templates");
  }
  if(state.size() == 0) {
    throw shift::core::Exception("DEM: no grains");
  }
  if(settings.walls.size() > 8) {
    throw shift::core::Exception("DEM: more than 8 walls");
  }
  if(settings.max_neighbors == 0 || settings.max_neighbors > 64) {
    throw shift::core::Exception("DEM: max_neighbors must be 1 to 64");
  }
  if(settings.max_triangles == 0 || settings.max_triangles > 32) {
    throw shift::core::Exception("DEM: max_triangles must be 1 to 32");
  }
  for(const Material& m : materials) {
    if(!(m.youngs_modulus > 0.0f) || !(m.poisson_ratio > -1.0f) ||
       !(m.poisson_ratio < 0.5f) || !(m.restitution >= 0.0f) ||
       !(m.friction >= 0.0f) || !(m.rolling_resistance >= 0.0f)) {
      throw shift::core::Exception("DEM: a material out of range");
    }
  }
  Layout& l = layout_;
  l.materials = static_cast<uint32_t>(materials.size());
  for(const Template& t : templates) {
    const bool hull = !t.hull.vertices.empty();
    if(hull == !t.spheres.empty() || !(t.mass > 0.0f) ||
       !(t.inertia[0] > 0.0f) || !(t.inertia[1] > 0.0f) ||
       !(t.inertia[2] > 0.0f)) {
      throw shift::core::Exception(
        "DEM: a template needs spheres or a hull, not both, a mass and an "
        "inertia");
    }
    l.template_mass.push_back(
      {t.mass, t.inertia[0], t.inertia[1], t.inertia[2]});
    l.template_spheres.push_back(
      {static_cast<uint32_t>(l.template_sphere.size()),
       static_cast<uint32_t>(hull ? 1 : t.spheres.size()), 0u, 0u});
    if(hull) {
      if(t.hull.vertices.size() < 4 ||
         t.hull.vertices.size() > kMaxHullVertices ||
         t.hull.planes.size() < 4 || t.hull.planes.size() > kMaxHullPlanes ||
         !(t.hull.radius > 0.0f)) {
        throw shift::core::Exception(
          "DEM: a hull needs 4 to " + std::to_string(kMaxHullVertices) +
          " vertices, 4 to " + std::to_string(kMaxHullPlanes) +
          " planes and a radius");
      }
      CheckMaterial(materials, t.hull.material);
      l.hulls = true;
      l.template_sphere.push_back({0.0f, 0.0f, 0.0f, t.hull.BoundingRadius()});
      l.template_material.push_back(t.hull.material);
      l.template_hull.push_back(
        {static_cast<uint32_t>(l.hull_vertex.size()),
         static_cast<uint32_t>(t.hull.vertices.size()),
         static_cast<uint32_t>(l.hull_plane.size()),
         static_cast<uint32_t>(t.hull.planes.size())});
      for(const auto& v : t.hull.vertices) {
        l.hull_vertex.push_back({v[0], v[1], v[2], t.hull.radius});
      }
      if(t.hull.faces.size() != t.hull.planes.size()) {
        throw shift::core::Exception("DEM: a hull needs a face per plane");
      }
      for(size_t f = 0; f < t.hull.planes.size(); f++) {
        l.hull_plane.push_back(t.hull.planes[f]);
        const auto& face = t.hull.faces[f];
        if(face.size() < 3 || face.size() > kMaxFaceVertices) {
          throw shift::core::Exception("DEM: a face of a hull of 3 to " +
                                       std::to_string(kMaxFaceVertices) +
                                       " vertices");
        }
        std::array<uint32_t, 4> packed = {
          static_cast<uint32_t>(face.size()), 0u, 0u, 0u};
        for(size_t k = 0; k < face.size(); k++) {
          if(face[k] >= t.hull.vertices.size()) {
            throw shift::core::Exception("DEM: a face of no vertex");
          }
          packed[1 + k / 4] |= face[k] << (8 * (k % 4));
        }
        l.hull_face.push_back(packed);
      }
      continue;
    }
    for(const Sphere& s : t.spheres) {
      if(!(s.radius > 0.0f)) {
        throw shift::core::Exception("DEM: a sphere of no radius");
      }
      CheckMaterial(materials, s.material);
      l.template_sphere.push_back(
        {s.center[0], s.center[1], s.center[2], s.radius});
      l.template_material.push_back(s.material);
      l.template_hull.push_back({0u, 0u, 0u, 0u});
    }
  }
  if(l.hulls && settings.method != ContactMethod::kSmc) {
    throw shift::core::Exception(
      "DEM: grains of convex hulls take the smooth method (smc) only");
  }
  l.cell = CellWidth(settings, templates);
  l.margin = Margin(settings, templates);
  size_t spheres = 0;
  for(size_t g = 0; g < state.size(); g++) {
    const uint32_t t = state.template_index[g];
    if(t >= templates.size()) {
      throw shift::core::Exception("DEM: a grain of no template");
    }
    std::array<int32_t, 4> anchor;
    std::array<float, 4> offset;
    for(int k = 0; k < 3; k++) {
      ToAnchor(state.position[g][k], l.cell, anchor[k], offset[k]);
    }
    anchor[3] = static_cast<int32_t>(t);
    offset[3] = 0.0f;
    l.anchor.push_back(anchor);
    l.offset.push_back(offset);
    l.first_sphere.push_back(static_cast<uint32_t>(spheres));
    for(size_t s = 0; s < l.template_spheres[t][1]; s++) {
      l.sphere_info.push_back(
        {static_cast<uint32_t>(g), l.template_spheres[t][0] + uint32_t(s)});
    }
    spheres += l.template_spheres[t][1];
  }
  /* one invocation per sphere: up to 65535 workgroups of 128 */
  if(spheres > 65535u * 128u) {
    throw shift::core::Exception("DEM: more than 8388480 spheres");
  }
  l.spheres = static_cast<uint32_t>(spheres);
  l.mask = HashTableSize(spheres) - 1;
  for(const Wall& w : settings.walls) {
    const double length = std::sqrt(w.normal[0] * w.normal[0] +
                                    w.normal[1] * w.normal[1] +
                                    w.normal[2] * w.normal[2]);
    if(!(length > 0.0)) {
      throw shift::core::Exception("DEM: a wall of no normal");
    }
    CheckMaterial(materials, w.material);
    std::array<int32_t, 4> anchor;
    std::array<float, 4> offset;
    std::array<float, 4> normal;
    for(int k = 0; k < 3; k++) {
      const double n = w.normal[k] / length;
      ToAnchor(n * w.offset / length, l.cell, anchor[k], offset[k]);
      normal[k] = static_cast<float>(n);
    }
    anchor[3] = 0;
    offset[3] = 0.0f;
    uint32_t material = w.material;
    std::memcpy(&normal[3], &material, 4);
    l.wall_anchor.push_back(anchor);
    l.wall_offset.push_back(offset);
    l.wall_normal.push_back(normal);
  }
  l.pairs = PairTable(settings, materials);
  /* the bodies' triangles, no longer than a cell */
  for(size_t b = 0; b < bodies.size(); b++) {
    CheckMaterial(materials, bodies[b].material);
    std::vector<std::array<float, 3>> vertices;
    std::vector<std::array<uint32_t, 3>> triangles;
    Subdivide(bodies[b].vertices, bodies[b].triangles, 0.999f * l.cell,
              vertices, triangles);
    for(const auto& t : triangles) {
      uint32_t tags[3] = {static_cast<uint32_t>(b), bodies[b].material, 0u};
      for(int k = 0; k < 3; k++) {
        std::array<float, 4> v = {vertices[t[k]][0], vertices[t[k]][1],
                                  vertices[t[k]][2], 0.0f};
        std::memcpy(&v[3], &tags[k], 4);
        l.triangle_local.push_back(v);
      }
    }
  }
  if(l.triangle_local.size() / 3 > 65535u * 128u / 8u) {
    throw shift::core::Exception("DEM: more than 1048560 triangles");
  }
  l.triangles = static_cast<uint32_t>(l.triangle_local.size() / 3);
  l.triangle_mask = HashTableSize(8 * size_t(l.triangles)) - 1;
}

void shift::core::dem::Solver::FromLayout(
  const std::vector<std::array<int32_t, 4>>& anchor,
  const std::vector<std::array<float, 4>>& offset, State& state) const {
  state.resize(anchor.size());
  for(size_t g = 0; g < anchor.size(); g++) {
    state.template_index[g] = static_cast<uint32_t>(anchor[g][3]);
    for(int k = 0; k < 3; k++) {
      state.position[g][k] =
        FromAnchor(anchor[g][k], offset[g][k], layout_.cell);
    }
  }
}
