#include "shift/gui/simulation.h"

#include "shift/core/body/flex.h"

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/body/base.h"
#include "shift/core/force_module/dem.h"
#include "shift/core/geometry/cad.h"
#include "shift/core/geometry/mesh.h"
#include "shift/core/integrator/base.h"
#include "shift/core/log.h"
#include "shift/core/light/ies.h"
#include "shift/core/light/spot.h"
#include "shift/core/log.h"
#include "shift/core/environment/surface_material.h"
#include "shift/core/terrain/basic.h"
#include "shift/core/terrain/scm.h"
#include "shift/core/terrain/sph.h"

#include "ksl/matrix.h"
#include "ksl/quaternion.h"
#include "ksl/SE3.h"
#include "ksl/vec3.h"

#include <H5Cpp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <sstream>
#include <map>
#include <tuple>
#include <set>
#include <thread>


static double WallTime() {
  return std::chrono::duration<double>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

/* a pose in single precision */
static ksl_SE3f_t ToPose(const ksl_SE3_t& d) {
  ksl_SE3f_t f;
  for(int k = 0; k < 12; k++) {
    f.as_array[k] = float(d.as_array[k]);
  }
  return f;
}

/* the box that holds no point yet, which points then grow */
static void EmptyBox(ksl_vec3f_t& box_min, ksl_vec3f_t& box_max) {
  box_min = ksl_vec3f(1e30f, 1e30f, 1e30f);
  box_max = ksl_vec3f(-1e30f, -1e30f, -1e30f);
}

static void ComputeMeshNormals(shift::gui::SceneMesh& mesh) {
  shift::gui::ComputeNormals(mesh.vertices, mesh.indices);
}

/* the closest point of triangle a, b, c to p, as barycentric coordinates
(Ericson, Real-Time Collision Detection, 5.1.5) */
static void ClosestOnTriangle(const ksl_vec3_t& p, const ksl_vec3_t& a,
                              const ksl_vec3_t& b, const ksl_vec3_t& c,
                              double l[3]) {
  ksl_vec3_t ab, ac, ap, bp, cp;
  ksl_subtract_vv(&b, &a, &ab);
  ksl_subtract_vv(&c, &a, &ac);
  ksl_subtract_vv(&p, &a, &ap);
  const double d1 = ksl_dot_vv(&ab, &ap), d2 = ksl_dot_vv(&ac, &ap);
  auto set = [&](double x, double y, double z) {
    l[0] = x;
    l[1] = y;
    l[2] = z;
  };
  if(d1 <= 0.0 && d2 <= 0.0) {
    return set(1.0, 0.0, 0.0);
  }
  ksl_subtract_vv(&p, &b, &bp);
  const double d3 = ksl_dot_vv(&ab, &bp), d4 = ksl_dot_vv(&ac, &bp);
  if(d3 >= 0.0 && d4 <= d3) {
    return set(0.0, 1.0, 0.0);
  }
  const double vc = d1 * d4 - d3 * d2;
  if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double t = d1 / (d1 - d3);
    return set(1.0 - t, t, 0.0);
  }
  ksl_subtract_vv(&p, &c, &cp);
  const double d5 = ksl_dot_vv(&ab, &cp), d6 = ksl_dot_vv(&ac, &cp);
  if(d6 >= 0.0 && d5 <= d6) {
    return set(0.0, 0.0, 1.0);
  }
  const double vb = d5 * d2 - d1 * d6;
  if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double t = d2 / (d2 - d6);
    return set(1.0 - t, 0.0, t);
  }
  const double va = d3 * d6 - d5 * d4;
  if(va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return set(0.0, 1.0 - t, t);
  }
  const double denominator = 1.0 / (va + vb + vc);
  const double y = vb * denominator, z = vc * denominator;
  set(1.0 - y - z, y, z);
}

/* half the mean length of the edges of the boundary triangles of a flexible
body's mesh: the spacing of its nodes on the surface */
static double FlexNodeSpacing(const shift::core::body::Flex& flex) {
  const auto& positions = flex.get_positions();
  double sum = 0.0;
  size_t count = 0;
  for(const auto& t : flex.get_mesh_boundary()) {
    for(int e = 0; e < 3; e++) {
      ksl_vec3_t d;
      ksl_subtract_vv(&positions[size_t(t[e])],
                      &positions[size_t(t[(e + 1) % 3])], &d);
      sum += ksl_vec3_l2norm(&d);
      count++;
    }
  }
  return count > 0 ? 0.5 * sum / double(count) : 0.0;
}

/* the shapes of a flexible body (the elastic coordinates', then the
fixed-interface modes computed, as SceneMesh::flex_basis) at the samples of
its CAD: those of the closest point of the mesh's surface, from the
quadratic shape functions of its boundary triangle, nearest the closest
node of the surface; and the vertices of the mesh drawn at that triangle's
nodes (vertex_point: the point of each vertex) with the same weights, for
the stress */
static void FlexSampleBasis(const shift::core::body::Flex& flex,
                            const std::vector<int>& vertex_point,
                            shift::gui::SceneCad& cad) {
  const auto& positions = flex.get_positions();
  const auto& shapes = flex.get_shapes();
  const auto& computed_shapes = flex.get_computed_shapes();
  const size_t n = flex.get_elastic_count();
  const size_t computed = flex.get_frequencies().size();
  const size_t columns = n + computed;
  const auto& boundary = flex.get_mesh_boundary();
  const size_t samples = cad.sample_points.size() / 4;
  cad.sample_basis.assign(3 * columns * samples, 0.f);
  cad.sample_vertices.clear();
  cad.sample_weights.clear();
  if(boundary.empty() || samples == 0) {
    return;
  }
  /* a vertex drawn at each point (the faces' vertices at a node carry the
  same stress) */
  std::map<int, uint32_t> vertex_at;
  for(size_t v = 0; v < vertex_point.size(); v++) {
    vertex_at.emplace(vertex_point[v], uint32_t(v));
  }
  const bool stress = !vertex_at.empty();
  if(stress) {
    cad.sample_vertices.assign(6 * samples, 0u);
    cad.sample_weights.assign(6 * samples, 0.f);
  }
  /* the boundary triangles of each surface node, and the nodes in cells of
  a grid */
  std::map<int, std::vector<size_t>> triangles_of;
  for(size_t k = 0; k < boundary.size(); k++) {
    for(int node : boundary[k]) {
      triangles_of[node].push_back(k);
    }
  }
  const double cell = std::max(2.0 * FlexNodeSpacing(flex), 1e-9);
  auto key = [&](const ksl_vec3_t& x) {
    return std::array<int64_t, 3>{int64_t(std::floor(x.x / cell)),
                                  int64_t(std::floor(x.y / cell)),
                                  int64_t(std::floor(x.z / cell))};
  };
  std::map<std::array<int64_t, 3>, std::vector<int>> cells;
  for(const auto& [node, list] : triangles_of) {
    cells[key(positions[size_t(node)])].push_back(node);
  }
  for(size_t k = 0; k < samples; k++) {
    const float* x = cad.sample_points.data() + 4 * k;
    const ksl_vec3_t p = ksl_vec3(x[0], x[1], x[2]);
    /* the nearest surface node, the search growing ring by ring */
    const auto center = key(p);
    int nearest = -1;
    double best = 1e300;
    int last_ring = 64;
    for(int ring = 0; ring <= last_ring; ring++) {
      for(int64_t i = -ring; i <= ring; i++) {
        for(int64_t j = -ring; j <= ring; j++) {
          for(int64_t m = -ring; m <= ring; m++) {
            if(std::max({std::abs(i), std::abs(j), std::abs(m)}) != ring) {
              continue;
            }
            auto it = cells.find({center[0] + i, center[1] + j,
                                  center[2] + m});
            if(it == cells.end()) {
              continue;
            }
            for(int node : it->second) {
              ksl_vec3_t d;
              ksl_subtract_vv(&positions[size_t(node)], &p, &d);
              const double distance = ksl_dot_vv(&d, &d);
              if(distance < best) {
                best = distance;
                nearest = node;
              }
            }
          }
        }
      }
      /* a node found in this ring may be farther than one in the next */
      if(nearest >= 0 && last_ring == 64) {
        last_ring = ring + 1;
      }
    }
    if(nearest < 0) {
      continue;
    }
    /* the closest point of the triangles of that node */
    double best_distance = 1e300;
    size_t triangle = 0;
    double l[3] = {1.0, 0.0, 0.0};
    for(size_t t : triangles_of[nearest]) {
      const auto& tri = boundary[t];
      double b[3];
      ClosestOnTriangle(p, positions[size_t(tri[0])], positions[size_t(tri[1])],
                        positions[size_t(tri[2])], b);
      ksl_vec3_t q = ksl_vec3(0.0, 0.0, 0.0);
      for(int c = 0; c < 3; c++) {
        ksl_axpy_vv(b[c], &positions[size_t(tri[c])], &q);
      }
      ksl_vec3_t d;
      ksl_subtract_vv(&q, &p, &d);
      const double distance = ksl_dot_vv(&d, &d);
      if(distance < best_distance) {
        best_distance = distance;
        triangle = t;
        std::copy(b, b + 3, l);
      }
    }
    /* quadratic triangle: corners l_i (2 l_i - 1), mid-sides 4 l_i l_j
    (3 between 0 and 1, 4 between 1 and 2, 5 between 2 and 0) */
    const double N[6] = {l[0] * (2.0 * l[0] - 1.0), l[1] * (2.0 * l[1] - 1.0),
                         l[2] * (2.0 * l[2] - 1.0), 4.0 * l[0] * l[1],
                         4.0 * l[1] * l[2], 4.0 * l[2] * l[0]};
    float* basis = cad.sample_basis.data() + 3 * columns * k;
    for(int a = 0; a < 6; a++) {
      const size_t node = size_t(boundary[triangle][a]);
      if(stress) {
        auto it = vertex_at.find(int(node));
        if(it != vertex_at.end()) {
          cad.sample_vertices[6 * k + size_t(a)] = it->second;
          cad.sample_weights[6 * k + size_t(a)] = float(N[a]);
        }
      }
      for(size_t j = 0; j < n; j++) {
        for(int c = 0; c < 3; c++) {
          basis[3 * j + c] += float(N[a] * shapes[node * n + j].lin.at[c]);
        }
      }
      for(size_t m = 0; m < computed; m++) {
        for(int c = 0; c < 3; c++) {
          basis[3 * (n + m) + c] +=
            float(N[a] * computed_shapes[3 * (node * computed + m) + c]);
        }
      }
    }
  }
}

static shift::gui::SceneMesh
MakeSceneMesh(const shift::gui::MeshData& data,
              const std::vector<shift::gui::MaterialData>& materials,
              shift::gui::TextureLoader& textures) {
  shift::gui::SceneMesh mesh;

  const size_t vertex_count = data.vertices.size() / 3;
  const bool has_normals = data.normals.size() == data.vertices.size();
  const bool has_uvs = data.texture_coordinates.size() == data.vertices.size();
  mesh.vertices.resize(vertex_count);
  EmptyBox(mesh.bounds_min, mesh.bounds_max);
  for(size_t i = 0; i < vertex_count; i++) {
    for(int k = 0; k < 3; k++) {
      mesh.vertices[i].position[k] = data.vertices[3 * i + k];
      mesh.vertices[i].normal[k] = has_normals ? data.normals[3 * i + k] : 0.f;
      mesh.bounds_min.at[k] =
        std::min(mesh.bounds_min.at[k], data.vertices[3 * i + k]);
      mesh.bounds_max.at[k] =
        std::max(mesh.bounds_max.at[k], data.vertices[3 * i + k]);
    }
    /* assimp gives texture coordinates with v = 0 at the bottom of the image
    (OpenGL); the texture layers are uploaded with the top row of the image at
    v = 0 */
    if(has_uvs) {
      mesh.vertices[i].uv[0] = data.texture_coordinates[3 * i + 0];
      mesh.vertices[i].uv[1] = 1.f - data.texture_coordinates[3 * i + 1];
    } else {
      mesh.vertices[i].uv[0] = mesh.vertices[i].uv[1] = 0.f;
    }
  }
  mesh.indices.assign(data.faces.begin(), data.faces.end());
  if(!has_normals) {
    ComputeMeshNormals(mesh);
  }

  if(data.material_index >= 0 &&
     data.material_index < static_cast<int>(materials.size())) {
    const auto& material = materials[data.material_index];
    mesh.albedo = ksl_vec3f(material.Kd[0], material.Kd[1], material.Kd[2]);
    mesh.opacity = std::clamp(material.opacity, 0.f, 1.f);
    mesh.alpha_cutoff = material.alpha_cutoff;
    // use the metallic-roughness value if given, otherwise map the
    // Blinn-Phong exponent to an approximately equivalent GGX roughness
    const float roughness = material.roughness >= 0.f
                              ? material.roughness
                              : std::sqrt(2.f / (material.Ns + 2.f));
    mesh.roughness = std::clamp(roughness, 0.05f, 1.f);
    mesh.metallic = std::clamp(material.metalness, 0.f, 1.f);

    mesh.base_color_layer = textures.Load(material.base_color_map);
    mesh.metallic_roughness_layer = textures.LoadMetallicRoughness(
      material.metallic_roughness_map, material.metallic_map,
      material.roughness_map);
    mesh.normal_layer = textures.Load(material.normal_map);
    mesh.normal_scale = material.normal_scale;
    mesh.triplanar = material.triplanar;
    mesh.texture_scale = material.texture_scale;
  }
  return mesh;
}


/* a KSL single precision vector of a double precision one */
static ksl_vec3f_t ToVec3f(const ksl_vec3_t& v) {
  return ksl_vec3f(float(v.x), float(v.y), float(v.z));
}

/* a sphere of center c and radius r in mesh, of segments around its z axis
and rings from pole to pole, its triangles wound counterclockwise seen from
outside */
static void AddSphere(const std::array<float, 3>& c, float r, int segments,
                      int rings, shift::gui::SceneMesh& mesh) {
  const uint32_t first = static_cast<uint32_t>(mesh.vertices.size());
  for(int j = 0; j <= rings; j++) {
    const double polar = M_PI * j / rings;
    for(int i = 0; i <= segments; i++) {
      const double azimuth = 2.0 * M_PI * i / segments;
      const float n[3] = {float(std::sin(polar) * std::cos(azimuth)),
                          float(std::sin(polar) * std::sin(azimuth)),
                          float(std::cos(polar))};
      shift::gui::Vertex v;
      for(int k = 0; k < 3; k++) {
        v.position[k] = c[k] + r * n[k];
        v.normal[k] = n[k];
      }
      v.uv[0] = float(i) / segments;
      v.uv[1] = float(j) / rings;
      mesh.vertices.push_back(v);
    }
  }
  const uint32_t row = uint32_t(segments + 1);
  for(uint32_t j = 0; j < uint32_t(rings); j++) {
    for(uint32_t i = 0; i < uint32_t(segments); i++) {
      const uint32_t a = first + j * row + i, b = a + 1, d = a + row,
                     e = d + 1;
      if(j > 0) {
        mesh.indices.insert(mesh.indices.end(), {a, d, b});
      }
      if(j + 1 < uint32_t(rings)) {
        mesh.indices.insert(mesh.indices.end(), {b, d, e});
      }
    }
  }
  for(int k = 0; k < 3; k++) {
    mesh.bounds_min.at[k] = std::min(mesh.bounds_min.at[k], c[k] - r);
    mesh.bounds_max.at[k] = std::max(mesh.bounds_max.at[k], c[k] + r);
  }
}

/* triangles of vertices in mesh, flat shaded: each triangle its own three
vertices, of its normal */
static void AddTriangles(
  const std::vector<std::array<float, 3>>& vertices,
  const std::vector<std::array<uint32_t, 3>>& triangles,
  shift::gui::SceneMesh& mesh) {
  for(const auto& t : triangles) {
    const auto& a = vertices[t[0]];
    const auto& b = vertices[t[1]];
    const auto& c = vertices[t[2]];
    /* the normal divided by its length, which for a grain a few millimeters
    across is twice an area of the order of 1e-5 m^2 (QVector3D::normalized()
    returned zero below 1e-5, and the face was drawn black); a triangle of no
    area is left out */
    const ksl_vec3f_t pa = ksl_vec3f(a[0], a[1], a[2]);
    const ksl_vec3f_t pb = ksl_vec3f(b[0], b[1], b[2]);
    const ksl_vec3f_t pc = ksl_vec3f(c[0], c[1], c[2]);
    ksl_vec3f_t ab, ac, n;
    ksl_subtract_vvf(&pb, &pa, &ab);
    ksl_subtract_vvf(&pc, &pa, &ac);
    ksl_cross_vvf(&ab, &ac, &n);
    const float length = ksl_vec3f_l2norm(&n);
    if(!(length > 0.0f)) {
      continue;
    }
    ksl_vec3f_scale(&n, 1.0f / length);
    for(int k = 0; k < 3; k++) {
      const auto& x = vertices[t[k]];
      shift::gui::Vertex v;
      for(int i = 0; i < 3; i++) {
        v.position[i] = x[i];
        v.normal[i] = n.at[i];
        mesh.bounds_min.at[i] = std::min(mesh.bounds_min.at[i], x[i]);
        mesh.bounds_max.at[i] = std::max(mesh.bounds_max.at[i], x[i]);
      }
      v.uv[0] = v.uv[1] = 0.0f;
      mesh.indices.push_back(static_cast<uint32_t>(mesh.vertices.size()));
      mesh.vertices.push_back(v);
    }
  }
}

/* the colors of the shapes of grains, in turn: sand, granite, clay, moss,
slate, ochre */
static const float kGrainColors[][3] = {
  {0.62f, 0.52f, 0.38f}, {0.48f, 0.48f, 0.50f}, {0.55f, 0.33f, 0.24f},
  {0.36f, 0.42f, 0.28f}, {0.30f, 0.34f, 0.40f}, {0.70f, 0.50f, 0.20f}};

/* the pose of a grain: its principal axes at its center of mass */
static ksl_SE3f_t GrainPose(const double position[3],
                            const float orientation[4]) {
  /* scalar last, as KSL's; in double precision, as KSL's single precision
  ksl_quaternionf_toMat3x3f takes z y for z w in two entries */
  ksl_quaternion_t q;
  for(int k = 0; k < 4; k++) {
    q.at[k] = orientation[k];
  }
  ksl_mat3x3_t r;
  ksl_quaternion_toMat3x3(&q, &r);
  ksl_SE3f_t pose;
  for(int c = 0; c < 3; c++) {
    for(int k = 0; k < 3; k++) {
      pose.R.at[c][k] = float(r.at[c][k]);
    }
  }
  pose.t = ksl_vec3f(float(position[0]), float(position[1]),
                     float(position[2]));
  return pose;
}

/* a terrain: its control points for the tessellation shaders, and a
triangle mesh of the surface (terrain::Basic::LocalZ) in world coordinates,
for the ray tracer and devices without tessellation. A deformable terrain
(terrain::Scm) is drawn over its grid only, bilinear between the nodes as
the soil models see it, with a ring of the undisturbed surface one spacing
beyond the grid, where its surface returns to the undisturbed one; its
control points start on the undisturbed surface (Simulation::ScmView) and
its mesh stays there */
static int AddTerrain(const shift::core::terrain::Basic& terrain,
                      shift::gui::TextureLoader& textures,
                      shift::gui::Scene& scene) {
  constexpr int kMaximumMeshSamples = 513; // per side
  constexpr int kMinimumMeshSamples = 257; // per side
  const auto* scm = dynamic_cast<const shift::core::terrain::Scm*>(&terrain);
  double x_extent = terrain.get_x_extent();
  double y_extent = terrain.get_y_extent();
  double x0 = 0.0, y0 = 0.0; // of the control points in the terrain frame
  shift::gui::SceneTerrain t;
  if(scm) {
    const long nx = scm->get_node_count_x(), ny = scm->get_node_count_y();
    x0 = scm->NodeX(-1);
    y0 = scm->NodeY(-1);
    x_extent = scm->get_spacing() * double(nx + 1);
    y_extent = scm->get_spacing() * double(ny + 1);
    t.width = static_cast<int>(nx + 2);
    t.height = static_cast<int>(ny + 2);
    t.z.reserve(size_t(t.width) * size_t(t.height));
    for(long j = -1; j <= ny; j++) {
      for(long i = -1; i <= nx; i++) {
        /* under an SPH region, its floor: its particles are the surface */
        const auto* sph = dynamic_cast<const shift::core::terrain::Sph*>(
          scm->RegionAt(scm->NodeX(i), scm->NodeY(j)));
        t.z.push_back(float(sph ? sph->FloorZ(scm->NodeX(i), scm->NodeY(j))
                                : scm->UndisturbedZ(i, j)));
      }
    }
    t.x_scale = t.y_scale = float(scm->get_spacing());
    t.bicubic = false;
    t.deformable = true;
  } else {
    t.width = static_cast<int>(terrain.get_x_samples());
    t.height = static_cast<int>(terrain.get_y_samples());
    t.z.assign(terrain.get_control_points().begin(),
               terrain.get_control_points().end());
    t.x_scale = float(terrain.get_x_scale());
    t.y_scale = float(terrain.get_y_scale());
    t.bicubic = terrain.is_bicubic();
  }
  if(!(x_extent > 0.0) || !(y_extent > 0.0)) {
    return -1;
  }
  /* the terrain's pose, its origin moved to (x0, y0) in its frame */
  ksl_SE3f_t pose = ToPose(terrain.get_pose());
  const ksl_SE3f_t terrain_frame = pose; // the material map's origin
  const ksl_vec3f_t corner = ksl_vec3f(float(x0), float(y0), 0.f);
  ksl_vec3f_t origin;
  ksl_product_dvf(&pose, &corner, &origin);
  pose.t = origin;
  t.pose = pose;

  /* the mesh, in world coordinates */
  shift::gui::SceneMesh mesh;
  mesh.albedo = ksl_vec3f(0.42f, 0.38f, 0.33f); // dry earth
  mesh.roughness = 0.95f;
  mesh.base_color_layer = textures.Load(terrain.get_texture_path());
  if(mesh.base_color_layer >= 0) {
    mesh.albedo = ksl_vec3f(1.f, 1.f, 1.f);
  }
  mesh.normal_layer = textures.Load(terrain.get_normal_map_path());
  mesh.normal_scale = float(terrain.get_normal_scale());
  mesh.triplanar = true; // along the world axes: the mesh is on the fixed body
  mesh.texture_scale = float(terrain.get_texture_scale());
  mesh.texture_variation = terrain.get_texture_variation();
  // no grid of repeated tiles over its extent: hex-tiled, unless its tiles
  // are varied instead
  mesh.hex_tiling = !mesh.texture_variation;
  for(const auto* surface : terrain.get_surfaces()) {
    shift::gui::SceneSurface s;
    s.base_color_layer = textures.Load(surface->get_texture_path());
    if(s.base_color_layer < 0) {
      continue; // not found: drawn without it
    }
    s.normal_layer = textures.Load(surface->get_normal_map_path());
    s.normal_scale = float(surface->get_normal_scale());
    s.texture_scale = float(surface->get_texture_scale());
    s.patch_size = float(surface->get_patch_size());
    s.level = shift::gui::SurfaceNoiseLevel(
      static_cast<int>(mesh.surfaces.size()), surface->get_coverage());
    mesh.surfaces.push_back(s);
  }
  /* the surface materials by area, in place of the terrain's texture */
  if(const auto* map = terrain.get_material_map()) {
    int materials = 0;
    for(const auto& m : scene.material_maps) {
      materials += int(m.materials.size());
    }
    if(int(scene.material_maps.size()) ==
         shift::gui::Scene::kMaxMaterialMaps ||
       materials + int(map->get_materials().size()) >
         shift::gui::Scene::kMaxGroundMaterials) {
      qWarning("Terrain %s: more than %d material maps or %d surface "
               "materials in the scene; drawn with its own texture",
               terrain.get_name().c_str(),
               shift::gui::Scene::kMaxMaterialMaps,
               shift::gui::Scene::kMaxGroundMaterials);
    } else {
      shift::gui::SceneMaterialMap m;
      ksl_SE3f_inverted(&terrain_frame, &m.world_to_map);
      m.width = int(map->get_x_samples());
      m.height = int(map->get_y_samples());
      m.x_scale = float(map->get_x_scale());
      m.y_scale = float(map->get_y_scale());
      m.blend_width = float(map->get_blend_width());
      m.indices = map->get_indices();
      for(const auto* material : map->get_materials()) {
        shift::gui::SceneGroundMaterial g;
        g.base_color_layer = textures.Load(material->get_texture_path());
        g.normal_layer = textures.Load(material->get_normal_map_path());
        g.roughness_layer = textures.LoadMetallicRoughness(
          "", "", material->get_roughness_map_path());
        g.height_layer = textures.Load(material->get_height_map_path());
        g.texture_scale = float(material->get_texture_scale());
        g.normal_scale = float(material->get_normal_scale());
        g.height_scale = float(material->get_height_scale());
        g.color = ToVec3f(material->get_color());
        g.roughness = float(material->get_roughness());
        g.label = material->get_label();
        m.materials.push_back(g);
      }
      mesh.material_map = int(scene.material_maps.size());
      scene.material_maps.push_back(std::move(m));
    }
  }
  /* the regions drawn as surfaces of their own are holes in this one */
  for(const auto* region : terrain.get_regions()) {
    if(dynamic_cast<const shift::core::terrain::Basic*>(region) &&
       !dynamic_cast<const shift::core::terrain::Sph*>(region) &&
       region->has_extent()) {
      const auto& x = region->get_x_range();
      const auto& y = region->get_y_range();
      mesh.holes.push_back(
        {float(x[0]), float(y[0]), float(x[1]), float(y[1])});
    }
  }
  /* twice the resolution of the control points, and at least
  kMinimumMeshSamples a side so that the curvature of a small map is drawn
  smoothly, at most kMaximumMeshSamples a side */
  int mesh_x = std::clamp(
    std::max(2 * (t.width - 1) + 1, kMinimumMeshSamples), 2,
    kMaximumMeshSamples);
  int mesh_y = std::clamp(
    std::max(2 * (t.height - 1) + 1, kMinimumMeshSamples), 2,
    kMaximumMeshSamples);
  if(scm) {
    /* a vertex at each control point and between them, s per cell along
    each side, which the ray tracer moves with the levels of the nodes
    (Viewport::ShapeRtTerrain): two triangles per square of the mesh, those
    of square (u, v) at 2 (v s (width - 1) + u). At most 2^21 triangles. */
    constexpr double kMaximumTriangles = 2097152.0;
    const double cells = double(t.width - 1) * double(t.height - 1);
    int s = 4;
    while(s > 1 && 2.0 * cells * s * s > kMaximumTriangles) {
      s--;
    }
    t.mesh_subdivisions = s;
    mesh_x = s * (t.width - 1) + 1;
    mesh_y = s * (t.height - 1) + 1;
  }
  EmptyBox(mesh.bounds_min, mesh.bounds_max);
  for(int j = 0; j < mesh_y; j++) {
    const double y = y_extent * j / (mesh_y - 1);
    for(int i = 0; i < mesh_x; i++) {
      const double x = x_extent * i / (mesh_x - 1);
      double z_x, z_y;
      const double z = terrain.LocalZ(x0 + x, y0 + y, z_x, z_y);
      /* outward, up: (z_x, z_y, -1) normalized */
      const double length = std::sqrt(z_x * z_x + z_y * z_y + 1.0);
      const ksl_vec3f_t n = ksl_vec3f(
        float(z_x / length), float(z_y / length), float(-1.0 / length));
      const ksl_vec3f_t local = ksl_vec3f(float(x), float(y), float(z));
      ksl_vec3f_t world, normal;
      ksl_product_dvf(&pose, &local, &world);
      ksl_product_drvf(&pose.R, &n, &normal);
      shift::gui::Vertex v;
      for(int a = 0; a < 3; a++) {
        v.position[a] = world.at[a];
        v.normal[a] = normal.at[a];
        mesh.bounds_min.at[a] = std::min(mesh.bounds_min.at[a], world.at[a]);
        mesh.bounds_max.at[a] = std::max(mesh.bounds_max.at[a], world.at[a]);
      }
      v.uv[0] = float(x0 + x);
      v.uv[1] = float(y0 + y);
      mesh.vertices.push_back(v);
    }
  }
  /* without the triangles inside a hole (the raster shader drops the
  pixels of the rest inside it) */
  auto in_hole = [&mesh](const shift::gui::Vertex& v) {
    for(const std::array<float, 4>& h : mesh.holes) {
      if(v.uv[0] >= h[0] && v.uv[0] <= h[2] && v.uv[1] >= h[1] &&
         v.uv[1] <= h[3]) {
        return true;
      }
    }
    return false;
  };
  const uint32_t nx = uint32_t(mesh_x);
  for(uint32_t j = 0; j + 1 < uint32_t(mesh_y); j++) {
    for(uint32_t i = 0; i + 1 < nx; i++) {
      const uint32_t a = j * nx + i, b = a + 1, c = a + nx, e = c + 1;
      const auto& v = mesh.vertices;
      if(!(in_hole(v[a]) && in_hole(v[b]) && in_hole(v[e]))) {
        mesh.indices.insert(mesh.indices.end(), {a, b, e});
      }
      if(!(in_hole(v[a]) && in_hole(v[e]) && in_hole(v[c]))) {
        mesh.indices.insert(mesh.indices.end(), {a, e, c});
      }
    }
  }
  t.mesh_index = static_cast<int>(scene.meshes.size());
  mesh.terrain = static_cast<int>(scene.terrains.size());
  scene.meshes.push_back(std::move(mesh));
  scene.instances.push_back({t.mesh_index, 0});
  scene.terrains.push_back(std::move(t));
  return static_cast<int>(scene.terrains.size()) - 1;
}

/* the particles of an SPH soil: a sphere of the particles' volume, s^3,
of the color of earth, darker than the terrain's so that the grains show */
static int AddParticles(const shift::core::terrain::Sph& sph,
                        shift::gui::Scene& scene) {
  shift::gui::SceneParticles particles;
  particles.radius =
    float(sph.get_settings().spacing * std::cbrt(3.0 / (4.0 * M_PI)));
  shift::gui::SceneMesh mesh;
  mesh.albedo = ksl_vec3f(0.36f, 0.31f, 0.25f);
  mesh.roughness = 0.95f;
  /* an icosphere, subdivided twice: 320 triangles */
  const float g = (1.f + std::sqrt(5.f)) / 2.f;
  std::vector<ksl_vec3f_t> points = {
    ksl_vec3f(-1, g, 0),  ksl_vec3f(1, g, 0),   ksl_vec3f(-1, -g, 0),
    ksl_vec3f(1, -g, 0),  ksl_vec3f(0, -1, g),  ksl_vec3f(0, 1, g),
    ksl_vec3f(0, -1, -g), ksl_vec3f(0, 1, -g),  ksl_vec3f(g, 0, -1),
    ksl_vec3f(g, 0, 1),   ksl_vec3f(-g, 0, -1), ksl_vec3f(-g, 0, 1)};
  std::vector<uint32_t> faces = {
    0, 11, 5, 0, 5,  1,  0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
    4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
    6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1};
  for(ksl_vec3f_t& p : points) {
    ksl_vec3f_normalize(&p);
  }
  for(int level = 0; level < 2; level++) {
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> middles;
    auto middle = [&](uint32_t a, uint32_t b) {
      const auto key = std::minmax(a, b);
      auto [it, inserted] = middles.try_emplace(key, 0);
      if(inserted) {
        it->second = uint32_t(points.size());
        ksl_vec3f_t m;
        ksl_add_vvf(&points[a], &points[b], &m);
        ksl_vec3f_normalize(&m);
        points.push_back(m);
      }
      return it->second;
    };
    std::vector<uint32_t> finer;
    for(size_t f = 0; f < faces.size(); f += 3) {
      const uint32_t a = faces[f], b = faces[f + 1], c = faces[f + 2];
      const uint32_t ab = middle(a, b), bc = middle(b, c), ca = middle(c, a);
      finer.insert(finer.end(),
                   {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
    }
    faces.swap(finer);
  }
  for(const ksl_vec3f_t& p : points) {
    shift::gui::Vertex v;
    for(int k = 0; k < 3; k++) {
      v.position[k] = particles.radius * p.at[k];
      v.normal[k] = p.at[k];
    }
    v.uv[0] = v.uv[1] = 0.f;
    mesh.vertices.push_back(v);
  }
  mesh.indices = faces;
  const float r = particles.radius;
  mesh.bounds_min = ksl_vec3f(-r, -r, -r);
  mesh.bounds_max = ksl_vec3f(r, r, r);
  particles.mesh_index = static_cast<int>(scene.meshes.size());
  scene.meshes.push_back(std::move(mesh));
  scene.particles.push_back(particles);
  return static_cast<int>(scene.particles.size()) - 1;
}

/* a light of the model, if its frame is on one of the bodies drawn */

/* the sky, sun, fog and water of the model, or the viewer's own if it gives none */
static shift::gui::SceneEnvironment
MakeEnvironment(const shift::core::environment::Environment* model) {
  shift::gui::SceneEnvironment e;
  if(!model) {
    return e;
  }
  e.given = true;
  const auto& sky = model->get_sky();
  e.sky_type = sky.get_type() == "physical"
                 ? shift::gui::SceneEnvironment::kPhysical
                 : (sky.get_type() == "hdri" && !sky.get_path().empty()
                      ? shift::gui::SceneEnvironment::kImage
                      : shift::gui::SceneEnvironment::kGradient);
  e.zenith = ToVec3f(sky.get_zenith_color());
  e.horizon = ToVec3f(sky.get_horizon_color());
  e.ground = ToVec3f(sky.get_ground_color());
  e.turbidity = float(sky.get_turbidity());
  e.image_path = sky.get_path();
  e.image_rotation = float(sky.get_rotation() * M_PI / 180.0);
  e.sky_intensity = float(sky.get_intensity());
  const auto& sun = model->get_sun();
  e.sun_direction = ToVec3f(sun.Direction());
  ksl_vec3f_normalize(&e.sun_direction);
  e.sun_intensity = float(sun.get_intensity());
  e.sun_color_given = sun.has_color();
  if(e.sun_color_given) {
    e.sun_color = ToVec3f(sun.get_color());
  }
  const auto& fog = model->get_fog();
  e.fog_density = float(fog.get_density());
  e.fog_height_falloff = float(fog.get_height_falloff());
  e.fog_height = float(fog.get_height());
  e.fog_color_given = fog.has_color();
  if(e.fog_color_given) {
    e.fog_color = ToVec3f(fog.get_color());
  }
  if(const auto* water = model->get_water()) {
    e.water = true;
    e.water_height = float(water->get_height());
    e.water_color = ToVec3f(water->get_color());
    e.water_extinction = ToVec3f(water->get_extinction());
    e.wave_height = float(water->get_wave_height());
    e.wavelength = float(water->get_wavelength());
    e.wave_direction = float(water->get_wave_direction() * M_PI / 180.0);
    e.wind_speed = float(water->get_wind_speed());
  }
  return e;
}

static void AddLight(const shift::core::light::Base& light,
                     const std::vector<shift::core::body::Base*>& bodies,
                     shift::gui::Scene& scene,
                     std::vector<const shift::core::light::Base*>& sources) {
  using shift::gui::Scene;
  using shift::gui::SceneLight;
  if(!light.get_frame()) {
    return; // Connect warned
  }
  const auto body = std::find(bodies.begin(), bodies.end(),
                              light.get_parent_body());
  if(body == bodies.end()) {
    return;
  }
  SceneLight l;
  const std::string& type = light.get_type();
  if(type == "point") {
    l.type = SceneLight::kPoint;
  } else if(type == "spot") {
    l.type = SceneLight::kSpot;
  } else if(type == "directional") {
    l.type = SceneLight::kDirectional;
  } else if(type == "ies") {
    l.type = SceneLight::kIes;
  } else {
    return;
  }
  l.body_index = static_cast<int>(body - bodies.begin());
  l.frame = ToPose(*light.get_frame()->get_d_wrt_origin());
  auto color = [](const ksl_vec3_t& c, double factor) {
    return ksl_vec3f(float(c.x * factor), float(c.y * factor),
                     float(c.z * factor));
  };
  l.ambient = color(light.get_Ka(), 1.0);
  l.diffuse = color(light.get_Kd(), 1.0);
  l.specular = color(light.get_Ks(), 1.0);
  l.intensity = float(light.get_intensity());
  l.attenuation = color(light.get_attenuation(), 1.0);
  if(auto spot = dynamic_cast<const shift::core::light::Spot*>(&light)) {
    l.cone_angle = float(spot->get_cone_angle());
    l.inner_cone_angle = float(spot->get_inner_cone_angle());
  }
  if(auto ies = dynamic_cast<const shift::core::light::IES*>(&light)) {
    std::vector<float> profile(size_t(Scene::kIesVertical) *
                               Scene::kIesHorizontal);
    for(int h = 0; h < Scene::kIesHorizontal; h++) {
      const double horizontal = 2.0 * M_PI * h / (Scene::kIesHorizontal - 1);
      for(int v = 0; v < Scene::kIesVertical; v++) {
        const double vertical = M_PI * v / (Scene::kIesVertical - 1);
        profile[size_t(h) * Scene::kIesVertical + v] =
          float(ies->Distribution(vertical, horizontal));
      }
    }
    l.ies_profile = static_cast<int>(scene.ies_profiles.size());
    scene.ies_profiles.push_back(std::move(profile));
  }
  scene.lights.push_back(l);
  sources.push_back(&light);
}

shift::gui::Simulation::~Simulation() {
  JoinWorker();
  context_.reset();
  RemoveLogDirectory();
}

void shift::gui::Simulation::Unload() {
  JoinWorker();
  scm_views_.clear();
  sph_views_.clear();
  terrain_indices_.clear();
  particles_indices_.clear();
  dem_views_.clear();
  grain_records_ = 0;
  flex_views_.clear();
  preview_view_ = -1;
  context_.reset();
  bodies_.clear();
  light_sources_.clear();
  scene_ = Scene();
  mesh_bvhs_.clear();
  ClearResults();
}

void shift::gui::Simulation::Load(const std::string& filename) {
  Unload();
  RemoveLogDirectory();
  filename_ = filename;
  std::string extension = std::filesystem::path(filename).extension().string();
  for(char& c : extension) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  try {
    if(extension == ".h5" || extension == ".hdf5") {
      LoadLog(filename);
    } else {
      LoadModel(filename, {});
    }
  } catch(...) {
    // e.g. stopped by the load observer once the context was made
    Unload();
    throw;
  }
}

void shift::gui::Simulation::Reload(const Json::Value& model) {
  if(model_path_.empty()) {
    throw shift::core::Exception("no model has been loaded");
  }
  // a copy: LoadModel sets it
  const std::string path = model_path_;
  Unload();
  // a log's files stay where they were taken out to
  try {
    LoadModel(path, file_map_, &model);
  } catch(...) {
    Unload();
    throw;
  }
}

void shift::gui::Simulation::LoadModel(
  const std::string& path, std::map<std::string, std::string> file_map,
  const Json::Value* model) {
  model_path_ = path;
  file_map_ = file_map;
  auto context = std::make_unique<shift::core::Context>();
  context->set_file_map(std::move(file_map));
  context->set_load_observer(load_observer_);
  if(model) {
    context->LoadJSON(*model, path);
  } else {
    context->LoadJSON(path);
  }
  context->set_load_observer(nullptr);
  context_ = std::move(context);

  // the context is the fixed (inertial) body; list it first, followed by
  // the remaining active bodies
  bodies_.push_back(context_->get_fixed_body());
  std::set<shift::core::body::Base*> listed(bodies_.begin(), bodies_.end());
  for(auto list : {&context_->get_active_articulated_bodies(),
                   &context_->get_active_free_bodies()}) {
    for(auto body : *list) {
      if(listed.insert(body).second) {
        bodies_.push_back(body);
      }
    }
  }

  /* meshes that share imported data (the same files, see
  geometry::Mesh::get_asset) become one SceneMesh each, drawn once per body */
  TextureLoader textures(scene_);
  std::map<const shift::core::geometry::MeshAsset*, std::vector<int>>
    asset_meshes;
  std::vector<std::vector<int>> mesh_bodies;
  std::map<const shift::core::geometry::Base*, std::vector<int>>
    geometry_meshes; // for the lights that geometries glow with
  for(size_t i = 0; i < bodies_.size(); i++) {
    ReportLoad("Preparing the meshes for drawing", int(i), int(bodies_.size()));
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(!mesh || !mesh->is_active()) {
        continue;
      }
      /* a flexible body's meshed solid is drawn by its mesh's surface,
      deformed (AddFlexViews) */
      auto flex = dynamic_cast<const shift::core::body::Flex*>(bodies_[i]);
      if(flex && flex->builds_model() &&
         flex->get_fe_geometry() == mesh->get_name()) {
        continue;
      }
      auto [it, inserted] = asset_meshes.try_emplace(mesh->get_asset());
      if(inserted) {
        textures.AddEmbedded(mesh->get_asset()->embedded_textures);
        for(const auto& data : mesh->get_mesh_data()) {
          if(data.faces.empty()) {
            continue;
          }
          it->second.push_back(static_cast<int>(scene_.meshes.size()));
          scene_.meshes.push_back(
            MakeSceneMesh(data, mesh->get_material_data(), textures));
          mesh_bodies.emplace_back();
        }
      }
      for(int mesh_index : it->second) {
        mesh_bodies[mesh_index].push_back(static_cast<int>(i));
      }
      geometry_meshes[geometry.get()] = it->second;
    }
  }
  /* the scenery: its meshes placed on the fixed body (bodies_[0]), each
  item an instance of the meshes of its variant */
  std::vector<std::vector<SceneInstance>> mesh_placements(mesh_bodies.size());
  for(auto& scenery : context_->get_scenery()) {
    ReportLoad("Preparing the scenery for drawing");
    std::vector<std::vector<int>> variant_meshes;
    for(auto& geometry : scenery->get_geometry()) {
      variant_meshes.emplace_back();
      auto mesh = dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(!mesh || !mesh->is_active()) {
        continue;
      }
      auto [it, inserted] = asset_meshes.try_emplace(mesh->get_asset());
      if(inserted) {
        textures.AddEmbedded(mesh->get_asset()->embedded_textures);
        for(const auto& data : mesh->get_mesh_data()) {
          if(data.faces.empty()) {
            continue;
          }
          it->second.push_back(static_cast<int>(scene_.meshes.size()));
          scene_.meshes.push_back(
            MakeSceneMesh(data, mesh->get_material_data(), textures));
          scene_.meshes.back().draw_distance =
            float(scenery->get_draw_distance());
          mesh_bodies.emplace_back();
          mesh_placements.emplace_back();
        }
      }
      variant_meshes.back() = it->second;
    }
    for(const auto& item : scenery->get_items()) {
      SceneInstance placed;
      placed.body_index = 0;
      placed.placement = ToPose(item.pose);
      placed.scale = float(item.scale);
      for(int mesh_index : variant_meshes[size_t(item.variant)]) {
        placed.mesh_index = mesh_index;
        mesh_placements[size_t(mesh_index)].push_back(placed);
      }
    }
  }
  for(size_t m = 0; m < mesh_bodies.size(); m++) {
    for(int body : mesh_bodies[m]) {
      SceneInstance instance;
      instance.mesh_index = static_cast<int>(m);
      instance.body_index = body;
      scene_.instances.push_back(instance);
    }
    for(const SceneInstance& placed : mesh_placements[m]) {
      scene_.instances.push_back(placed);
    }
  }
  /* the levels of detail of the meshes, made on all the cores */
  {
    ReportLoad("Simplifying the meshes for the distance");
    const auto start = std::chrono::steady_clock::now();
    std::atomic<size_t> next{0};
    // the foliage's impostors, baked beside its levels and added after
    const size_t mesh_count = scene_.meshes.size();
    std::vector<SceneMeshImpostor> impostors(mesh_count);
    std::vector<uint8_t> baked(mesh_count, 0);
    auto work = [&]() {
      for(size_t m = next++; m < mesh_count; m = next++) {
        BuildSceneMeshLods(scene_.meshes[m]);
        baked[m] = BakeSceneMeshImpostor(scene_, scene_.meshes[m],
                                         impostors[m]);
      }
    };
    std::vector<std::thread> threads;
    const unsigned count = std::max(1u, std::thread::hardware_concurrency());
    for(unsigned t = 1; t < count; t++) {
      threads.emplace_back(work);
    }
    work();
    for(auto& thread : threads) {
      thread.join();
    }
    size_t impostor_count = 0;
    for(size_t m = 0; m < mesh_count; m++) {
      if(baked[m]) {
        AddSceneMeshImpostor(scene_, int(m), std::move(impostors[m]));
        impostor_count++;
      }
    }
    size_t levels = 0;
    for(const auto& mesh : scene_.meshes) {
      levels += mesh.lods.size();
    }
    SHIFT_INFO("Made " + std::to_string(levels) + " levels of detail (" +
               std::to_string(impostor_count) + " impostors) in " +
               std::to_string(std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - start)
                                .count()) +
               " s");
  }
  ReportLoad("Preparing the flexible bodies for drawing");
  AddFlexViews();
  /* CAD geometry: one SceneCad per file read (geometry::Cad::get_asset),
  scale, color and opacity, drawn once per body */
  std::map<std::tuple<const shift::core::geometry::CadAsset*, double,
                      std::vector<double>, double>,
           int>
    asset_cads;
  for(size_t i = 0; i < bodies_.size(); i++) {
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      if(!cad || !cad->is_active() || !cad->get_asset()) {
        continue;
      }
      /* a flexible body's meshed solid: a CAD of its own, deformed as its
      mesh (AddFlexViews), sampled at about the spacing of the mesh's
      nodes */
      auto flex = dynamic_cast<const shift::core::body::Flex*>(bodies_[i]);
      if(flex && flex->builds_model() &&
         flex->get_fe_geometry() == cad->get_name()) {
        const FlexView* view = nullptr;
        for(const auto& v : flex_views_) {
          if(v.body == static_cast<int>(i)) {
            view = &v;
          }
        }
        if(!view) {
          continue;
        }
        ReportLoad("Flexible body '" + flex->get_name() +
                   "': sampling its CAD for drawing deformed");
        SceneCad scene_cad = MakeSceneCad(cad->get_asset()->model,
                                          cad->get_asset()->bodies,
                                          cad->get_scale());
        const auto& color = cad->get_color();
        if(color.size() == 3) {
          scene_cad.albedo =
            ksl_vec3f(float(color[0]), float(color[1]), float(color[2]));
        }
        scene_cad.opacity = float(cad->get_opacity());
        SampleCadPatches(cad->get_asset()->model, cad->get_asset()->bodies,
                         FlexNodeSpacing(*flex), scene_cad);
        FlexSampleBasis(*flex, view->vertex_point, scene_cad);
        scene_cad.flex_mesh = view->mesh;
        scene_cad.bodies.push_back(static_cast<int>(i));
        scene_.cads.push_back(std::move(scene_cad));
        continue;
      }
      const auto key = std::make_tuple(cad->get_asset(), cad->get_scale(),
                                       cad->get_color(), cad->get_opacity());
      auto [it, inserted] = asset_cads.try_emplace(key, 0);
      if(inserted) {
        it->second = static_cast<int>(scene_.cads.size());
        scene_.cads.push_back(MakeSceneCad(cad->get_asset()->model,
                                           cad->get_asset()->bodies,
                                           cad->get_scale()));
        auto& scene_cad = scene_.cads.back();
        const auto& color = cad->get_color();
        if(color.size() == 3) {
          scene_cad.albedo =
            ksl_vec3f(float(color[0]), float(color[1]), float(color[2]));
        }
        scene_cad.opacity = float(cad->get_opacity());
      }
      scene_.cads[it->second].bodies.push_back(static_cast<int>(i));
    }
  }
  /* terrains, on the fixed body (body 0) */
  scm_views_.clear();
  ReportLoad("Preparing the terrain for drawing");
  sph_views_.clear();
  terrain_indices_.clear();
  particles_indices_.clear();
  /* the terrains and their regions: a region drawn as a surface of its
  own (e.g. SCM sand in rigid ground) is a hole in the terrain that holds
  it; an SPH soil is drawn by its particles alone */
  for(auto terrain : shift::core::terrain::Base::WithRegions(
        context_->get_active_terrain())) {
    if(auto sph = dynamic_cast<shift::core::terrain::Sph*>(terrain)) {
      SphView view;
      view.sph = sph;
      view.particles = AddParticles(*sph, scene_);
      particles_indices_[terrain] = view.particles;
      Snapshot(*sph, view.initial);
      sph_views_.push_back(std::move(view));
      continue;
    }
    auto basic = dynamic_cast<shift::core::terrain::Basic*>(terrain);
    if(!basic) {
      continue;
    }
    const int index = AddTerrain(*basic, textures, scene_);
    if(index >= 0) {
      terrain_indices_[terrain] = index;
    }
    auto scm = dynamic_cast<shift::core::terrain::Scm*>(basic);
    if(scm && index >= 0) {
      AddScmView(*scm, index);
    }
  }
  /* lights, on the bodies of their frames */
  for(auto light : context_->get_active_lights()) {
    AddLight(*light, bodies_, scene_, light_sources_);
  }
  scene_.environment = MakeEnvironment(context_->get_environment());
  /* the meshes that glow with the lights */
  for(size_t k = 0; k < light_sources_.size(); k++) {
    for(const auto& name : light_sources_[k]->get_glow()) {
      shift::core::body::Base* body = bodies_[scene_.lights[k].body_index];
      for(auto& geometry : body->get_geometry()) {
        auto meshes = geometry_meshes.find(geometry.get());
        if(geometry->get_name() != name || meshes == geometry_meshes.end()) {
          continue;
        }
        for(int m : meshes->second) {
          scene_.meshes[m].glow_light = static_cast<int>(k);
        }
      }
    }
  }
  scene_.body_count = static_cast<int>(bodies_.size());
  /* the meshes' ray tracing hierarchies, the longest part of making the
  scene ready to draw, here rather than in the viewer as it draws */
  mesh_bvhs_.clear();
  mesh_bvhs_.resize(scene_.meshes.size());
  for(size_t m = 0; m < scene_.meshes.size(); m++) {
    ReportLoad("Preparing the meshes for ray tracing", int(m),
               int(scene_.meshes.size()));
    const SceneMesh& mesh = scene_.meshes[m];
    // a deformable terrain's triangles tagged, and a flexible body's mesh's,
    // for the viewer to follow
    const bool deformable =
      mesh.terrain >= 0 &&
      mesh.terrain < static_cast<int>(scene_.terrains.size()) &&
      scene_.terrains[mesh.terrain].deformable;
    mesh_bvhs_[m] = BuildSceneMeshBvh(mesh, deformable || mesh.deformable);
  }
  /* the grains, their poses after the bodies' */
  dem_views_.clear();
  grain_records_ = 0;
  for(auto module : context_->get_active_force_modules()) {
    if(auto dem = dynamic_cast<shift::core::force_module::Dem*>(module)) {
      AddDemView(*dem);
    }
  }
  shown_lights_.assign(light_sources_.size(), 0.f);
  for(size_t k = 0; k < light_sources_.size(); k++) {
    shown_lights_[k] = scene_.lights[k].intensity;
  }

  state_size_ = context_->get_state_count();
  StoreInitialState();
  playback_dhva_ = context_->get_dhva();
  interpolated_.assign(state_size_, 0.0);

  context_->set_state_observer(
    [this](const double time, const std::vector<double>& y) {
      Observe(time, y);
    });

  ShowContextState();
}

void shift::gui::Simulation::StoreInitialState() {
  // the integrators copy q, dq and the free body states as one contiguous
  // block starting at q
  const double* q = context_->get_q().data();
  initial_state_.assign(q, q + state_size_);
}

// ------------------------------------------------------------------ run

void shift::gui::Simulation::Run() {
  if(!context_ || is_computing()) {
    return;
  }
  JoinWorker();

  // start from the configuration currently shown
  if(has_results()) {
    std::memcpy(context_->get_q().data(), interpolated_.data(),
                state_size_ * sizeof(double));
    context_->UpdateDHVA();
    RestartTerrains(false);
  }
  /* the grains shown, or, without results, those the DEM holds */
  RestartGrains(false);
  ClearResults();

  std::lock_guard<std::mutex> lock(mutex_);
  computing_ = true;
  stop_requested_ = false;
  error_.clear();
  wall_start_ = WallTime();
  real_time_factor_ = 0.0;
  worker_ = std::thread(&Simulation::Work, this);
}

void shift::gui::Simulation::Work() {
  std::string error;
  try {
    context_->Simulate();
  } catch(StopRequested&) {
  } catch(std::exception& e) {
    error = e.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  computing_ = false;
  error_ = error;
}

void shift::gui::Simulation::Observe(double time,
                                     const std::vector<double>& y) {
  // runs on the worker thread from Context::WriteStates at each output step
  bool knot = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    knot = times_.empty() || time > times_.back();
  }
  /* the grains, read from the solver (from the GPU) without the lock; the
  worker alone adds knot points */
  std::vector<GrainRecord> grains;
  if(knot && grain_records_ > 0) {
    grains.resize(grain_records_);
    for(const auto& view : dem_views_) {
      view.dem->GetGrains(worker_grains_);
      for(size_t g = 0; g < worker_grains_.size(); g++) {
        GrainRecord& r = grains[view.record + g];
        for(int k = 0; k < 3; k++) {
          r.position[k] = worker_grains_.position[g][k];
          r.velocity[k] = worker_grains_.velocity[g][k];
          r.angular_velocity[k] = worker_grains_.angular_velocity[g][k];
        }
        for(int k = 0; k < 4; k++) {
          r.orientation[k] = worker_grains_.orientation[g][k];
        }
      }
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if(stop_requested_) {
    throw StopRequested();
  }
  if(knot) {
    times_.push_back(time);
    states_.insert(states_.end(), y.begin(), y.begin() + state_size_);
    knot_deltas_.push_back(deltas_.size());
    grains_.insert(grains_.end(), grains.begin(), grains.end());
    // the lights as the controllers set them for this state
    for(const auto* light : light_sources_) {
      light_values_.push_back(float(light->get_intensity()));
    }
  }
  // the particles, at the knot point just recorded (or the last one)
  for(SphView& view : sph_views_) {
    if(view.knots.size() < times_.size()) {
      view.knots.emplace_back();
    }
    Snapshot(*view.sph, view.knots.back());
  }
  // the levels changed, with the knot point just recorded (or the last one,
  // at a repeated time)
  for(size_t v = 0; v < scm_views_.size(); v++) {
    ScmView& view = scm_views_[v];
    for(const auto& change : view.scm->get_output_changes()) {
      float& level = view.worker[size_t(change.index)];
      if(level != change.level) {
        deltas_.push_back({int32_t(v), change.index, level, change.level});
        level = change.level;
      }
    }
  }
  const double wall = WallTime() - wall_start_;
  real_time_factor_ = wall > 0.0 ? time / wall : 0.0;
}

void shift::gui::Simulation::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  JoinWorker();
}

void shift::gui::Simulation::JoinWorker() {
  if(worker_.joinable()) {
    worker_.join();
  }
  stop_requested_ = false;
}

void shift::gui::Simulation::Reset() {
  if(!context_) {
    return;
  }
  Stop();
  RestartTerrains(true);
  RestartGrains(true);
  for(SphView& view : sph_views_) {
    view.sph->Restart();
    Snapshot(*view.sph, view.initial);
  }
  ClearResults();
  std::memcpy(context_->get_q().data(), initial_state_.data(),
              state_size_ * sizeof(double));
  context_->UpdateDHVA();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_.clear();
  }
  ShowContextState();
}

void shift::gui::Simulation::ClearResults() {
  std::lock_guard<std::mutex> lock(mutex_);
  times_.clear();
  states_.clear();
  light_values_.clear();
  deltas_.clear();
  knot_deltas_.clear();
  grains_.clear();
  shown_i0_ = 1;
  shown_i1_ = 0;
  base_delta_ = partial_begin_ = partial_end_ = 0;
  for(auto& view : scm_views_) {
    view.base = view.worker = view.shown;
  }
  for(auto& view : sph_views_) {
    view.knots.clear();
    view.shown = -1;
    view.changed = true;
  }
  real_time_factor_ = 0.0;
  shown_time_ = 0.0;
}

bool shift::gui::Simulation::is_computing() {
  std::lock_guard<std::mutex> lock(mutex_);
  return computing_;
}

bool shift::gui::Simulation::has_results() {
  std::lock_guard<std::mutex> lock(mutex_);
  return !times_.empty();
}

double shift::gui::Simulation::get_computed_time() {
  std::lock_guard<std::mutex> lock(mutex_);
  return times_.empty() ? 0.0 : times_.back();
}

double shift::gui::Simulation::get_end_time() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return 0.0;
  }
  return context_->get_active_integrators()[0]->get_time_span();
}

double shift::gui::Simulation::get_time_step() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return 0.0;
  }
  return context_->get_active_integrators()[0]->get_time_step();
}

void shift::gui::Simulation::set_end_time(double time_span) {
  if(context_ && !is_computing() &&
     !context_->get_active_integrators().empty()) {
    context_->get_active_integrators()[0]->set_time_span(time_span);
  }
}

void shift::gui::Simulation::set_time_step(double time_step) {
  if(context_ && !is_computing() &&
     !context_->get_active_integrators().empty()) {
    context_->get_active_integrators()[0]->set_time_step(time_step);
  }
}

std::string shift::gui::Simulation::get_integrator_type() const {
  if(!context_ || context_->get_active_integrators().empty()) {
    return "";
  }
  return context_->get_active_integrators()[0]->get_type();
}

std::string shift::gui::Simulation::get_error() {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

double shift::gui::Simulation::get_real_time_factor() {
  std::lock_guard<std::mutex> lock(mutex_);
  return real_time_factor_;
}

// ------------------------------------------------------------- playback

void shift::gui::Simulation::ShowTime(double t) {
  if(!context_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(times_.empty()) {
      return;
    }
    // knot points t0 <= t <= t1, y = y0 + (y1 - y0) / (t1 - t0) * (t - t0)
    t = std::clamp(t, times_.front(), times_.back());
    const size_t i1 = std::min<size_t>(
      std::lower_bound(times_.begin(), times_.end(), t) - times_.begin(),
      times_.size() - 1);
    const size_t i0 = i1 > 0 ? i1 - 1 : 0;
    const double span = times_[i1] - times_[i0];
    const double p = span > 0.0 ? (t - times_[i0]) / span : 1.0;
    const double* y0 = states_.data() + i0 * state_size_;
    const double* y1 = states_.data() + i1 * state_size_;
    for(size_t i = 0; i < state_size_; i++) {
      interpolated_[i] = y0[i] + p * (y1[i] - y0[i]);
    }
    const size_t n = light_sources_.size();
    if(light_values_.size() >= (i1 + 1) * n) {
      for(size_t k = 0; k < n; k++) {
        const float a = light_values_[i0 * n + k];
        const float b = light_values_[i1 * n + k];
        shown_lights_[k] = a + float(p) * (b - a);
      }
    }
    // the particles: those of the knot at or before t
    ShowParticles(long(p >= 1.0 ? i1 : i0));
    // the states the peak of the stress drawn is of: those up to t
    const size_t peak_count = (p >= 1.0 ? i1 : i0) + 1;
    for(auto& view : flex_views_) {
      if(view.peak_count != peak_count) {
        view.peak_count = peak_count;
        view.changed = view.changed || stress_peak_;
      }
    }
    // the levels: those of knot i0 and the fraction p of the deltas to i1
    if(!scm_views_.empty()) {
      const auto knot_end = [&](size_t k) {
        return k + 1 < knot_deltas_.size() ? knot_deltas_[k + 1]
                                           : deltas_.size();
      };
      if(i1 > i0) {
        ShowLevels(knot_end(i0), knot_deltas_[i1], knot_end(i1), p);
      } else {
        ShowLevels(knot_end(i1), 0, 0, 0.0);
      }
    }
    InterpolateGrains(i0, i1, p);
  }
  shown_time_ = t;
  // the worker owns the context DHVA while computing; evaluate the
  // displacements into a separate buffer
  context_->UpdateD(playback_dhva_, interpolated_.data());
  context_->UpdateFreeDV(playback_dhva_, interpolated_.data() +
                                           context_->get_free_state_offset());
  UpdatePoses(playback_dhva_);
  ShowElastic(interpolated_.data());
}

void shift::gui::Simulation::ShowContextState() {
  for(size_t k = 0; k < light_sources_.size(); k++) {
    shown_lights_[k] = float(light_sources_[k]->get_intensity());
  }
  context_->ComputeMobility();
  UpdatePoses(context_->get_dhva());
  ShowElastic(context_->get_q().data());
  /* the state edited is not one recorded: no peak but its own */
  for(auto& view : flex_views_) {
    if(view.peak_count != 0) {
      view.peak_count = 0;
      view.changed = view.changed || stress_peak_;
    }
  }
  /* the grains the DEM holds: the worker is not running */
  shift::core::dem::State state;
  for(const auto& view : dem_views_) {
    view.dem->GetGrains(state);
    ShowGrains(view, state);
  }
  shown_i0_ = 1;
  shown_i1_ = 0;
}

void shift::gui::Simulation::UpdatePoses(
  const shift::core::topology::DHVA& dhva) {
  poses_.time = shown_time_;
  poses_.lights = shown_lights_;
  // the grains' poses after the bodies' stay as they are
  poses_.bodies.resize(bodies_.size() + size_t(scene_.grain_count));
  for(size_t i = 0; i < bodies_.size(); i++) {
    if(bodies_[i] == context_->get_fixed_body()) {
      ksl_SE3f_setIdentity(&poses_.bodies[i]);
    } else {
      poses_.bodies[i] =
        ToPose(context_->get_body_origin_pose(bodies_[i], dhva));
    }
  }
  poses_changed_ = true;
}

// ------------------------------------------------------ flexible bodies

void shift::gui::Simulation::AddFlexViews() {
  flex_views_.clear();
  preview_view_ = -1;
  for(size_t i = 0; i < bodies_.size(); i++) {
    auto flex = dynamic_cast<const shift::core::body::Flex*>(bodies_[i]);
    if(!flex || !flex->builds_model() || flex->get_mesh_boundary().empty()) {
      continue;
    }
    FlexView view;
    view.flex = flex;
    view.body = static_cast<int>(i);
    view.mesh = static_cast<int>(scene_.meshes.size());
    view.eta.assign(flex->get_elastic_count(), 0.0);
    SceneMesh mesh;
    mesh.deformable = true;
    for(auto& geometry : bodies_[i]->get_geometry()) {
      auto cad = dynamic_cast<shift::core::geometry::Cad*>(geometry.get());
      if(cad && cad->get_name() == flex->get_fe_geometry() &&
         cad->get_color().size() == 3) {
        const auto& c = cad->get_color();
        mesh.albedo = ksl_vec3f(float(c[0]), float(c[1]), float(c[2]));
      }
      /* a mesh's: the diffuse color of its first part's material */
      auto triangles =
        dynamic_cast<shift::core::geometry::Mesh*>(geometry.get());
      if(triangles && triangles->get_name() == flex->get_fe_geometry() &&
         !triangles->get_mesh_data().empty()) {
        const int m = triangles->get_mesh_data()[0].material_index;
        const auto& materials = triangles->get_material_data();
        if(m >= 0 && size_t(m) < materials.size()) {
          const float* kd = materials[size_t(m)].Kd;
          mesh.albedo = ksl_vec3f(kd[0], kd[1], kd[2]);
        }
      }
    }
    /* a vertex for each node on each face, so that edges between faces
    stay sharp; each 6-node triangle as four */
    std::map<std::pair<int, int>, uint32_t> vertex_of;
    const auto& boundary = flex->get_mesh_boundary();
    const auto& faces = flex->get_mesh_boundary_face();
    auto vertex = [&](int point, int face) {
      auto [it, inserted] = vertex_of.try_emplace(
        std::make_pair(point, face), uint32_t(view.vertex_point.size()));
      if(inserted) {
        view.vertex_point.push_back(point);
      }
      return it->second;
    };
    static const int kSubTriangles[4][3] = {
      {0, 3, 5}, {3, 1, 4}, {5, 4, 2}, {3, 4, 5}};
    for(size_t k = 0; k < boundary.size(); k++) {
      for(const auto& t : kSubTriangles) {
        for(int c : t) {
          mesh.indices.push_back(vertex(boundary[k][c], faces[k]));
        }
      }
    }
    /* the sides of the elements, corner to midside node to corner: each
    once, by its nodes, though the faces either side have vertices of their
    own */
    static const int kSides[6][2] = {{0, 3}, {3, 1}, {1, 4},
                                     {4, 2}, {2, 5}, {5, 0}};
    /* a vertex at each node the attachments tie to their frames, on the
    surface (one of the faces' vertices at it) */
    std::map<int, uint32_t> vertex_at_point;
    for(const auto& [key, v] : vertex_of) {
      vertex_at_point.emplace(key.first, v);
    }
    /* each attachment's color on its nodes' vertices and on a vertex of
    its own at its frame (the point after the mesh nodes, which moves with
    the shapes as the nodes do), and its spider from that vertex to them */
    std::map<uint32_t, float> vertex_color;
    std::set<uint32_t> constrained;
    const auto& attachment_nodes = flex->get_attachment_nodes();
    for(size_t k = 0; k < attachment_nodes.size(); k++) {
      const float color = float(1 + AttachmentColorIndex(k));
      const int frame_point = int(flex->get_mesh_node_count() + k);
      const uint32_t centre = vertex(frame_point, -1);
      vertex_color[centre] = color;
      mesh.constrained_vertices.push_back(centre);
      for(int node : attachment_nodes[k]) {
        auto it = vertex_at_point.find(node);
        if(it == vertex_at_point.end()) {
          continue; // inside the material
        }
        if(constrained.insert(it->second).second) {
          mesh.constrained_vertices.push_back(it->second);
          vertex_color[it->second] = color;
        }
        mesh.spider_indices.push_back(centre);
        mesh.spider_indices.push_back(it->second);
      }
    }
    std::set<std::pair<int, int>> sides;
    for(size_t k = 0; k < boundary.size(); k++) {
      for(const auto& side : kSides) {
        const int a = boundary[k][side[0]], b = boundary[k][side[1]];
        if(sides.insert(std::minmax(a, b)).second) {
          mesh.edge_indices.push_back(vertex(a, faces[k]));
          mesh.edge_indices.push_back(vertex(b, faces[k]));
        }
      }
    }
    /* the rigid part of each shape: t = mean of u, theta = S^-1 sum
    d x u, S = sum (|d|^2 I - d d^T), d the node from the centroid */
    const auto& positions = flex->get_positions();
    const auto& shapes = flex->get_shapes();
    const size_t nodes = flex->get_mesh_node_count();
    const size_t n = flex->get_elastic_count();
    for(size_t p = 0; p < nodes; p++) {
      ksl_axpy_vv(1.0, &positions[p], &view.centroid);
    }
    ksl_vec3_scale(&view.centroid, 1.0 / double(std::max<size_t>(nodes, 1)));
    ksl_mat3x3_t S = {};
    for(size_t p = 0; p < nodes; p++) {
      ksl_vec3_t d;
      ksl_subtract_vv(&positions[p], &view.centroid, &d);
      const double dd = ksl_dot_vv(&d, &d);
      for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
          S.at[c][r] += (r == c ? dd : 0.0) - d.at[r] * d.at[c];
        }
      }
    }
    ksl_mat3x3_t S_inverse;
    const bool fit = nodes >= 3 && ksl_mat3x3_inverted(&S, &S_inverse) == 0;
    view.rigid_translation.assign(n, ksl_vec3(0.0, 0.0, 0.0));
    view.rigid_rotation.assign(n, ksl_vec3(0.0, 0.0, 0.0));
    for(size_t j = 0; fit && j < n; j++) {
      ksl_vec3_t moment = ksl_vec3(0.0, 0.0, 0.0);
      for(size_t p = 0; p < nodes; p++) {
        const ksl_vec3_t& u = shapes[p * n + j].lin;
        ksl_vec3_t d, dxu;
        ksl_subtract_vv(&positions[p], &view.centroid, &d);
        ksl_cross_vv(&d, &u, &dxu);
        ksl_axpy_vv(1.0, &dxu, &moment);
        ksl_axpy_vv(1.0, &u, &view.rigid_translation[j]);
      }
      ksl_vec3_scale(&view.rigid_translation[j], 1.0 / double(nodes));
      ksl_product_drv(&S_inverse, &moment, &view.rigid_rotation[j]);
    }
    /* the undeformed surface, and the basis: the shapes of the elastic
    coordinates, then the fixed-interface modes computed */
    const size_t computed = flex->get_frequencies().size();
    const auto& computed_shapes = flex->get_computed_shapes();
    mesh.flex_columns = static_cast<uint32_t>(n + computed);
    mesh.flex_centroid = ksl_vec3f(float(view.centroid.x),
                                   float(view.centroid.y),
                                   float(view.centroid.z));
    mesh.vertices.resize(view.vertex_point.size());
    mesh.flex_basis.reserve(3 * mesh.flex_columns * mesh.vertices.size());
    for(size_t v = 0; v < mesh.vertices.size(); v++) {
      const size_t p = size_t(view.vertex_point[v]);
      Vertex& vertex = mesh.vertices[v];
      for(int c = 0; c < 3; c++) {
        vertex.position[c] = float(positions[p].at[c]);
      }
      vertex.uv[0] = 0.f;
      auto color = vertex_color.find(uint32_t(v));
      vertex.uv[1] = color != vertex_color.end() ? color->second : 0.f;
      for(size_t j = 0; j < n; j++) {
        for(int c = 0; c < 3; c++) {
          mesh.flex_basis.push_back(float(shapes[p * n + j].lin.at[c]));
        }
      }
      /* an attachment's frame (a point after the mesh nodes) is held in
      the fixed-interface modes */
      const bool node = p < flex->get_mesh_node_count();
      for(size_t k = 0; k < computed; k++) {
        for(int c = 0; c < 3; c++) {
          mesh.flex_basis.push_back(
            node ? float(computed_shapes[3 * (p * computed + k) + c]) : 0.f);
        }
      }
    }
    /* the stress of each vertex in each shape, from its node's */
    const auto& surface_nodes = flex->get_surface_nodes();
    const auto& surface_stresses = flex->get_surface_stresses();
    const size_t columns = mesh.flex_columns;
    if(!surface_nodes.empty() &&
       surface_stresses.size() == 6 * columns * surface_nodes.size()) {
      mesh.flex_stress_basis.reserve(6 * columns * mesh.vertices.size());
      for(size_t v = 0; v < mesh.vertices.size(); v++) {
        const int p = view.vertex_point[v];
        if(p >= int(flex->get_mesh_node_count())) {
          /* an attachment's frame: no stress, never drawn on a surface */
          mesh.flex_stress_basis.insert(mesh.flex_stress_basis.end(),
                                        6 * columns, 0.f);
          continue;
        }
        auto it =
          std::lower_bound(surface_nodes.begin(), surface_nodes.end(), p);
        if(it == surface_nodes.end() || *it != p) {
          mesh.flex_stress_basis.clear();
          break;
        }
        const double* s = surface_stresses.data() +
                          6 * columns * size_t(it - surface_nodes.begin());
        for(size_t k = 0; k < 6 * columns; k++) {
          mesh.flex_stress_basis.push_back(float(s[k]));
        }
      }
    }
    ComputeMeshNormals(mesh);
    EmptyBox(mesh.bounds_min, mesh.bounds_max);
    for(const Vertex& v : mesh.vertices) {
      for(int c = 0; c < 3; c++) {
        mesh.bounds_min.at[c] = std::min(mesh.bounds_min.at[c], v.position[c]);
        mesh.bounds_max.at[c] = std::max(mesh.bounds_max.at[c], v.position[c]);
      }
    }
    scene_.meshes.push_back(std::move(mesh));
    scene_.instances.push_back({view.mesh, view.body});
    flex_views_.push_back(std::move(view));
  }
}

std::vector<int> shift::gui::Simulation::get_flex_bodies() const {
  std::vector<int> bodies;
  for(const auto& view : flex_views_) {
    bodies.push_back(view.body);
  }
  return bodies;
}

void shift::gui::Simulation::ShowElastic(const double* y) {
  for(auto& view : flex_views_) {
    const int q = view.flex->get_elastic_q_index();
    if(q < 0) {
      continue;
    }
    for(size_t j = 0; j < view.eta.size(); j++) {
      if(view.eta[j] != y[q + j]) {
        view.eta[j] = y[q + j];
        view.changed = true;
      }
    }
  }
}

void shift::gui::Simulation::set_flex_stresses(std::vector<bool> bodies,
                                               bool automatic, double low,
                                               double high) {
  stress_bodies_ = std::move(bodies);
  stress_automatic_ = automatic;
  stress_range_[0] = low;
  stress_range_[1] = high;
  for(auto& view : flex_views_) {
    view.changed = true;
    view.peak_sent = 0;
  }
}

void shift::gui::Simulation::set_flex_stress_quantity(
  FlexUpdate::Quantity quantity, double safety_low, double safety_high) {
  stress_quantity_ = quantity;
  safety_range_[0] = safety_low;
  safety_range_[1] = safety_high;
  for(auto& view : flex_views_) {
    view.changed = true;
    view.peak_sent = 0;
  }
}

void shift::gui::Simulation::set_flex_stress_peak(bool peak) {
  stress_peak_ = peak;
  for(auto& view : flex_views_) {
    view.changed = true;
    view.peak_sent = 0;
  }
}

bool shift::gui::Simulation::has_flex_stresses(int body) const {
  for(const auto& view : flex_views_) {
    if(view.body == body) {
      return !scene_.meshes[size_t(view.mesh)].flex_stress_basis.empty();
    }
  }
  return false;
}

void shift::gui::Simulation::set_deformation_scale(double scale) {
  if(scale != deformation_scale_) {
    deformation_scale_ = scale;
    for(auto& view : flex_views_) {
      view.changed = true;
    }
  }
}

void shift::gui::Simulation::PreviewShape(int body, int kind, int coordinate,
                                          double amplitude) {
  for(size_t v = 0; v < flex_views_.size(); v++) {
    if(flex_views_[v].body == body) {
      if(preview_view_ >= 0) {
        flex_views_[preview_view_].changed = true;
      }
      preview_view_ = static_cast<int>(v);
      preview_kind_ = kind;
      preview_coordinate_ = coordinate;
      preview_amplitude_ = amplitude;
      flex_views_[v].changed = true;
    }
  }
}

void shift::gui::Simulation::ClearShapePreview() {
  if(preview_view_ >= 0) {
    flex_views_[preview_view_].changed = true;
  }
  preview_view_ = -1;
}

void shift::gui::Simulation::FlexCoefficients(const FlexView& view,
                                              FlexUpdate& update) const {
  const shift::core::body::Flex& flex = *view.flex;
  const size_t n = flex.get_elastic_count();
  const size_t computed = flex.get_frequencies().size();
  update.mesh = view.mesh;
  update.coefficients.assign(n + computed, 0.f);
  update.translation = ksl_vec3f(0.f, 0.f, 0.f);
  update.rotation = ksl_vec3f(0.f, 0.f, 0.f);
  const bool preview =
    preview_view_ >= 0 && &flex_views_[preview_view_] == &view;
  /* the stress: of the elastic coordinates, or of the shape previewed (its
  coefficients set below), not magnified */
  update.stress_coefficients.assign(n + computed, 0.f);
  const bool stress =
    size_t(view.body) < stress_bodies_.size() &&
    stress_bodies_[size_t(view.body)] &&
    !scene_.meshes[size_t(view.mesh)].flex_stress_basis.empty();
  update.stress = !stress              ? FlexUpdate::Stress::kNone
                  : stress_automatic_ ? FlexUpdate::Stress::kAutomatic
                                      : FlexUpdate::Stress::kRange;
  update.stress_range[0] = float(stress_range_[0]);
  update.stress_range[1] = float(stress_range_[1]);
  update.quantity = stress_quantity_;
  update.yield_strength = float(flex.get_yield_strength());
  if(stress_quantity_ == FlexUpdate::Quantity::kSafetyFactor) {
    /* none without a yield strength; always over its own range */
    if(!(flex.get_yield_strength() > 0.0)) {
      update.stress = FlexUpdate::Stress::kNone;
    } else if(update.stress != FlexUpdate::Stress::kNone) {
      update.stress = FlexUpdate::Stress::kRange;
      update.stress_range[0] = float(safety_range_[0]);
      update.stress_range[1] = float(safety_range_[1]);
    }
  }
  /* the peak of the motion recorded, not of a shape previewed */
  update.peak = update.stress != FlexUpdate::Stress::kNone && stress_peak_ &&
                !preview;
  if(preview) {
    /* the shape previewed, its largest displacement the amplitude */
    const size_t column =
      preview_kind_ == 1 ? n + size_t(preview_coordinate_)
                         : size_t(preview_coordinate_);
    if(column >= n + computed) {
      return;
    }
    double largest = 0.0;
    for(unsigned int p = 0; p < flex.get_mesh_node_count(); p++) {
      ksl_vec3_t u;
      if(column >= n) {
        const double* s = flex.get_computed_shapes().data() +
                          3 * (size_t(p) * computed + (column - n));
        u = ksl_vec3(s[0], s[1], s[2]);
      } else {
        u = flex.get_shapes()[size_t(p) * n + column].lin;
      }
      largest = std::max(largest, ksl_vec3_l2norm(&u));
    }
    update.coefficients[column] =
      float(largest > 0.0 ? preview_amplitude_ / largest : 0.0);
    update.stress_coefficients[column] = update.coefficients[column];
    return;
  }
  /* u + (scale - 1) (u - rigid part of u), u = sum eta_j s_j */
  ksl_vec3_t translation = ksl_vec3(0.0, 0.0, 0.0);
  ksl_vec3_t rotation = ksl_vec3(0.0, 0.0, 0.0);
  const double rigid = 1.0 - deformation_scale_;
  for(size_t j = 0; j < n; j++) {
    const double eta = view.eta[j];
    update.coefficients[j] = float(deformation_scale_ * eta);
    update.stress_coefficients[j] = float(eta);
    ksl_vec3_t t, r;
    ksl_product_av(rigid * eta, &view.rigid_translation[j], &t);
    ksl_product_av(rigid * eta, &view.rigid_rotation[j], &r);
    ksl_axpy_vv(1.0, &t, &translation);
    ksl_axpy_vv(1.0, &r, &rotation);
  }
  update.translation = ksl_vec3f(float(translation.x), float(translation.y),
                                 float(translation.z));
  update.rotation =
    ksl_vec3f(float(rotation.x), float(rotation.y), float(rotation.z));
}

void shift::gui::Simulation::TakeFlexUpdates(std::vector<FlexUpdate>& updates,
                                             bool all) {
  updates.clear();
  for(auto& view : flex_views_) {
    /* the yield strength is a key that may change as the model runs */
    if(view.flex->get_yield_strength() != view.yield_strength) {
      view.yield_strength = view.flex->get_yield_strength();
      view.changed = true;
    }
    if(!view.changed && !all) {
      continue;
    }
    view.changed = false;
    FlexUpdate update;
    FlexCoefficients(view, update);
    if(all) {
      view.peak_sent = 0;
    }
    FlexPeakSamples(view, update);
    updates.push_back(std::move(update));
  }
}

void shift::gui::Simulation::FlexPeakSamples(FlexView& view,
                                             FlexUpdate& update) {
  const int q = view.flex->get_elastic_q_index();
  if(!update.peak || q < 0) {
    update.peak = false;
    view.peak_sent = 0;
    return;
  }
  /* from the first state, if the time shown went back */
  const size_t n = view.flex->get_elastic_count();
  const size_t first = view.peak_count < view.peak_sent ? 0 : view.peak_sent;
  update.peak_first = first;
  update.peak_columns = n;
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t count = std::min(view.peak_count, times_.size());
  for(size_t k = first; k < count; k++) {
    const double* y = states_.data() + k * state_size_ + size_t(q);
    for(size_t j = 0; j < n; j++) {
      update.peak_samples.push_back(float(y[j]));
    }
  }
  view.peak_sent = std::max(first, count);
}

bool shift::gui::Simulation::TakePoses(Poses& poses) {
  if(!poses_changed_) {
    return false;
  }
  poses = poses_;
  poses_changed_ = false;
  return true;
}

// ------------------------------------------------- deformable terrains

void shift::gui::Simulation::Snapshot(const shift::core::terrain::Sph& sph,
                                      std::vector<qfloat16>& positions) {
  const shift::core::sph::State& state = sph.get_state();
  positions.clear();
  positions.reserve(3 * sph.get_particle_count());
  for(size_t k = 0; k < state.size(); k++) {
    if(state.kind[k] & shift::core::sph::State::kMarker) {
      continue;
    }
    for(int c = 0; c < 3; c++) {
      positions.push_back(qfloat16(state.position[k][c]));
    }
  }
}

void shift::gui::Simulation::ShowParticles(long knot) {
  // GUI thread, mutex_ held
  for(SphView& view : sph_views_) {
    const long shown = knot < long(view.knots.size()) ? knot : -1;
    if(shown != view.shown) {
      view.shown = shown;
      view.changed = true;
    }
  }
}

void shift::gui::Simulation::TakeParticleUpdates(
  std::vector<ParticleUpdate>& updates, bool all) {
  updates.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  for(SphView& view : sph_views_) {
    if(!view.changed && !all) {
      continue;
    }
    view.changed = false;
    const std::vector<qfloat16>& positions =
      view.shown >= 0 ? view.knots[size_t(view.shown)] : view.initial;
    const ksl_SE3f_t frame = ToPose(view.sph->get_particle_frame());
    ParticleUpdate update;
    update.particles = view.particles;
    update.centers.resize(positions.size() / 3 * 4);
    for(size_t k = 0; 3 * k < positions.size(); k++) {
      const ksl_vec3f_t local =
        ksl_vec3f(float(positions[3 * k]), float(positions[3 * k + 1]),
                  float(positions[3 * k + 2]));
      ksl_vec3f_t world;
      ksl_product_dvf(&frame, &local, &world);
      for(int c = 0; c < 3; c++) {
        update.centers[4 * k + c] = world.at[c];
      }
      update.centers[4 * k + 3] = 0.f;
    }
    updates.push_back(std::move(update));
  }
}

void shift::gui::Simulation::AddScmView(shift::core::terrain::Scm& scm,
                                        int index) {
  ScmView view;
  view.scm = &scm;
  view.terrain = index;
  view.nx = scm.get_node_count_x();
  view.ny = scm.get_node_count_y();
  view.undisturbed.resize(size_t(view.nx * view.ny));
  const auto& z = scene_.terrains[index].z;
  for(long j = 0; j < view.ny; j++) {
    for(long i = 0; i < view.nx; i++) {
      view.undisturbed[size_t(j * view.nx + i)] =
        z[size_t((j + 1) * (view.nx + 2) + i + 1)];
    }
  }
  // the levels the terrain starts from, zero after Load
  std::vector<float> levels;
  scm.GetLevels(levels);
  view.shown.assign(levels.size(), 0.0f);
  for(size_t k = 0; k < levels.size(); k++) {
    ShowLevel(view, int32_t(k), levels[k]);
  }
  view.base = view.worker = view.shown;
  scm_views_.push_back(std::move(view));
}

void shift::gui::Simulation::ShowLevel(ScmView& view, int32_t index,
                                       float level) {
  float& shown = view.shown[size_t(index)];
  if(shown == level) {
    return;
  }
  shown = level;
  const long i = index % view.nx, j = index / view.nx;
  scene_.terrains[view.terrain].z[size_t((j + 1) * (view.nx + 2) + i + 1)] =
    view.undisturbed[size_t(index)] + level;
  // control point (i + 1, j + 1)
  if(view.x_min > view.x_max) {
    view.x_min = view.x_max = i + 1;
    view.y_min = view.y_max = j + 1;
  } else {
    view.x_min = std::min(view.x_min, i + 1);
    view.x_max = std::max(view.x_max, i + 1);
    view.y_min = std::min(view.y_min, j + 1);
    view.y_max = std::max(view.y_max, j + 1);
  }
}

void shift::gui::Simulation::ShowLevels(size_t base, size_t partial_begin,
                                        size_t partial_end, double p) {
  // the deltas shown in part before go back to their base levels
  for(size_t k = partial_begin_; k < partial_end_; k++) {
    const LevelDelta& d = deltas_[k];
    ScmView& view = scm_views_[size_t(d.view)];
    ShowLevel(view, d.index, view.base[size_t(d.index)]);
  }
  // the base levels, forward or back to position base
  for(; base_delta_ < base; base_delta_++) {
    const LevelDelta& d = deltas_[base_delta_];
    ScmView& view = scm_views_[size_t(d.view)];
    view.base[size_t(d.index)] = d.after;
    ShowLevel(view, d.index, d.after);
  }
  for(; base_delta_ > base; base_delta_--) {
    const LevelDelta& d = deltas_[base_delta_ - 1];
    ScmView& view = scm_views_[size_t(d.view)];
    view.base[size_t(d.index)] = d.before;
    ShowLevel(view, d.index, d.before);
  }
  for(size_t k = partial_begin; k < partial_end; k++) {
    const LevelDelta& d = deltas_[k];
    ShowLevel(scm_views_[size_t(d.view)], d.index,
              float(d.before + p * (d.after - d.before)));
  }
  partial_begin_ = partial_begin;
  partial_end_ = partial_end;
}

void shift::gui::Simulation::RestartTerrains(bool reset) {
  // the worker is not running: Run, Reset and AdoptShownState joined it
  for(auto& view : scm_views_) {
    if(reset) {
      for(size_t k = 0; k < view.shown.size(); k++) {
        ShowLevel(view, int32_t(k), 0.0f);
      }
    }
    view.scm->SetLevels(view.shown);
  }
}

void shift::gui::Simulation::TakeTerrainUpdates(
  std::vector<TerrainUpdate>& updates) {
  updates.clear();
  for(auto& view : scm_views_) {
    if(view.x_min > view.x_max) {
      continue;
    }
    const SceneTerrain& terrain = scene_.terrains[view.terrain];
    TerrainUpdate update;
    update.terrain = view.terrain;
    update.x = int(view.x_min);
    update.y = int(view.y_min);
    update.width = int(view.x_max - view.x_min + 1);
    update.height = int(view.y_max - view.y_min + 1);
    update.z.resize(size_t(update.width) * size_t(update.height));
    for(int j = 0; j < update.height; j++) {
      const float* row =
        terrain.z.data() + size_t(update.y + j) * size_t(terrain.width) +
        size_t(update.x);
      std::copy(row, row + update.width,
                update.z.begin() + size_t(j) * size_t(update.width));
    }
    updates.push_back(std::move(update));
    view.x_min = 0;
    view.x_max = -1;
  }
}

bool shift::gui::Simulation::HasShownUpdates() {
  for(const auto& view : scm_views_) {
    if(view.x_min <= view.x_max) {
      return true;
    }
  }
  for(const auto& view : flex_views_) {
    // as TakeFlexUpdates, which also sends a new yield strength
    if(view.changed ||
       view.flex->get_yield_strength() != view.yield_strength) {
      return true;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for(const auto& view : sph_views_) {
    if(view.changed) {
      return true;
    }
  }
  return false;
}

// ------------------------------------------------------------ grains

/*!
The shapes of the grains become one mesh each, their spheres together in
the grain's principal axes, or their mesh (Template::mesh_vertices, a
hull's own triangles or a file's), flat shaded; the grains are instances of them, grouped by
shape (Scene::instances are contiguous by mesh), and their poses follow the
bodies' and the grains of the views before
*/
void shift::gui::Simulation::AddDemView(shift::core::force_module::Dem& dem) {
  DemView view;
  view.dem = &dem;
  view.first = bodies_.size() + size_t(scene_.grain_count);
  view.record = grain_records_;
  const auto& templates = dem.get_templates();
  const shift::core::dem::State& grains = dem.get_initial_grains();
  /* finer spheres for fewer: 24 by 12 to 20000 spheres, 12 by 6 beyond */
  size_t spheres = 0;
  for(size_t g = 0; g < grains.size(); g++) {
    spheres += templates[grains.template_index[g]].spheres.size();
  }
  const int segments = spheres <= 20000 ? 24 : 12;
  std::vector<int> meshes(templates.size());
  for(size_t t = 0; t < templates.size(); t++) {
    SceneMesh mesh;
    EmptyBox(mesh.bounds_min, mesh.bounds_max);
    if(!templates[t].mesh_triangles.empty()) {
      AddTriangles(templates[t].mesh_vertices, templates[t].mesh_triangles,
                   mesh);
    } else {
      for(const auto& sphere : templates[t].spheres) {
        AddSphere(sphere.center, sphere.radius, segments, segments / 2, mesh);
      }
    }
    const float* color = kGrainColors[t % std::size(kGrainColors)];
    mesh.albedo = ksl_vec3f(color[0], color[1], color[2]);
    mesh.roughness = 0.8f;
    meshes[t] = static_cast<int>(scene_.meshes.size());
    scene_.meshes.push_back(std::move(mesh));
  }
  for(size_t t = 0; t < templates.size(); t++) {
    for(size_t g = 0; g < grains.size(); g++) {
      if(grains.template_index[g] == t) {
        scene_.instances.push_back({meshes[t], static_cast<int>(view.first + g)});
      }
    }
  }
  scene_.grain_count += static_cast<int>(grains.size());
  grain_records_ += grains.size();
  dem_views_.push_back(view);
}

void shift::gui::Simulation::ShowGrains(
  const DemView& view, const shift::core::dem::State& state) {
  poses_.bodies.resize(bodies_.size() + size_t(scene_.grain_count));
  for(size_t g = 0; g < state.size(); g++) {
    poses_.bodies[view.first + g] =
      GrainPose(state.position[g].data(), state.orientation[g].data());
  }
  poses_changed_ = true;
}

/* the state of the grain at fraction p from a to b: the positions and
velocities linearly, the orientation normalized, along the shorter way */
static void Interpolate(const double a_position[3], const float a_q[4],
                        const double b_position[3], const float b_q[4],
                        double p, double position[3], float q[4]) {
  for(int k = 0; k < 3; k++) {
    position[k] = a_position[k] + p * (b_position[k] - a_position[k]);
  }
  const float dot = a_q[0] * b_q[0] + a_q[1] * b_q[1] + a_q[2] * b_q[2] +
                    a_q[3] * b_q[3];
  const float sign = dot < 0.f ? -1.f : 1.f;
  float length = 0.f;
  for(int k = 0; k < 4; k++) {
    q[k] = a_q[k] + float(p) * (sign * b_q[k] - a_q[k]);
    length += q[k] * q[k];
  }
  length = std::sqrt(length);
  for(int k = 0; k < 4; k++) {
    q[k] = length > 0.f ? q[k] / length : a_q[k];
  }
}

void shift::gui::Simulation::InterpolateGrains(size_t i0, size_t i1,
                                               double p) {
  if(grain_records_ == 0 || grains_.size() < (i1 + 1) * grain_records_) {
    return; // none recorded (a log): as the model placed them
  }
  const GrainRecord* a = grains_.data() + i0 * grain_records_;
  const GrainRecord* b = grains_.data() + i1 * grain_records_;
  for(const auto& view : dem_views_) {
    const size_t count = view.dem->get_initial_grains().size();
    for(size_t g = 0; g < count; g++) {
      const GrainRecord& r0 = a[view.record + g];
      const GrainRecord& r1 = b[view.record + g];
      double position[3];
      float q[4];
      Interpolate(r0.position, r0.orientation, r1.position, r1.orientation, p,
                  position, q);
      poses_.bodies[view.first + g] = GrainPose(position, q);
    }
  }
  shown_i0_ = i0;
  shown_i1_ = i1;
  shown_p_ = p;
}

bool shift::gui::Simulation::ShownGrains(const DemView& view,
                                         shift::core::dem::State& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(shown_i0_ > shown_i1_ ||
     grains_.size() < (shown_i1_ + 1) * grain_records_) {
    return false;
  }
  state = view.dem->get_initial_grains();
  const GrainRecord* a = grains_.data() + shown_i0_ * grain_records_;
  const GrainRecord* b = grains_.data() + shown_i1_ * grain_records_;
  const float p = float(shown_p_);
  for(size_t g = 0; g < state.size(); g++) {
    const GrainRecord& r0 = a[view.record + g];
    const GrainRecord& r1 = b[view.record + g];
    float q[4];
    Interpolate(r0.position, r0.orientation, r1.position, r1.orientation,
                shown_p_, state.position[g].data(), q);
    for(int k = 0; k < 4; k++) {
      state.orientation[g][k] = q[k];
    }
    for(int k = 0; k < 3; k++) {
      state.velocity[g][k] =
        r0.velocity[k] + p * (r1.velocity[k] - r0.velocity[k]);
      state.angular_velocity[g][k] =
        r0.angular_velocity[k] +
        p * (r1.angular_velocity[k] - r0.angular_velocity[k]);
    }
  }
  return true;
}

/*!
Without grains recorded at the time shown (no run made yet, the context's
own state after jogging, or a log), the grains the DEM holds start over:
those it was set to, or where its last run left them
*/
void shift::gui::Simulation::RestartGrains(bool reset) {
  // the worker is not running: Run, Reset and AdoptShownState joined it
  shift::core::dem::State state;
  for(const auto& view : dem_views_) {
    if(reset) {
      state = view.dem->get_initial_grains();
    } else if(!ShownGrains(view, state)) {
      view.dem->GetGrains(state);
    }
    view.dem->SetGrains(state);
    ShowGrains(view, state);
  }
}

// ------------------------------------------------------------ editing q

bool shift::gui::Simulation::AdoptShownState() {
  if(!context_ || is_computing()) {
    return false;
  }
  JoinWorker();
  if(has_results()) {
    // continue from the configuration shown; the recorded results no longer
    // correspond to the context once q is edited
    std::memcpy(context_->get_q().data(), interpolated_.data(),
                state_size_ * sizeof(double));
    RestartTerrains(false);
    RestartGrains(false);
    ClearResults();
    context_->UpdateDHVA();
  }
  return true;
}

double shift::gui::Simulation::get_shown_q(int q_index) {
  if(!context_ || q_index < 0 || q_index >= static_cast<int>(state_size_)) {
    return 0.0;
  }
  if(has_results()) {
    return interpolated_[q_index];
  }
  return context_->get_q()[q_index];
}

// ------------------------------------------------------------ HDF5 logs

void shift::gui::Simulation::RemoveLogDirectory() {
  if(!log_directory_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(log_directory_, error);
    log_directory_.clear();
  }
}

/* a string attribute of an object of an HDF5 file */
static std::string ReadStringAttribute(const H5::H5Object& object,
                                       const std::string& name) {
  H5::Attribute attribute = object.openAttribute(name);
  H5::StrType type = attribute.getStrType();
  std::string text;
  attribute.read(type, text);
  return text;
}

/* the length of a one-dimensional dataset, or the rows of a 2-D one */
static hsize_t Rows(const H5::DataSet& dataset) {
  H5::DataSpace space = dataset.getSpace();
  if(space.getSimpleExtentNdims() < 1) {
    return 0;
  }
  hsize_t dims[2] = {0, 0};
  space.getSimpleExtentDims(dims);
  return dims[0];
}

void shift::gui::Simulation::LoadLog(const std::string& filename) {
  ReportLoad("Unpacking the log");
  std::string error_prefix = "Log " + filename + ": ";
  H5::Exception::dontPrint();
  try {
    H5::H5File file(filename, H5F_ACC_RDONLY);
    if(!file.nameExists("/model/json")) {
      throw shift::core::Exception(
        error_prefix + "it holds no model (/model/json); it was written by an "
                       "older shift");
    }
    /* the model's files, under a temporary directory deep enough that the
    paths that climb out of the model's directory (..) stay inside it */
    std::random_device random;
    const std::filesystem::path root =
      std::filesystem::temp_directory_path() /
      ("shift-log-" + std::to_string(random()));
    log_directory_ = root.string();
    struct File {
      std::string path, original, names;
      H5::DataSet dataset;
    };
    std::vector<File> files;
    size_t depth = 0;
    if(file.nameExists("/model/files")) {
      H5::Group group = file.openGroup("/model/files");
      for(hsize_t k = 0; k < group.getNumObjs(); k++) {
        File f;
        f.dataset = group.openDataSet(group.getObjnameByIdx(k));
        f.path = ReadStringAttribute(f.dataset, "path");
        f.original = ReadStringAttribute(f.dataset, "original");
        if(f.dataset.attrExists("names")) {
          f.names = ReadStringAttribute(f.dataset, "names");
        }
        size_t up = 0;
        for(const auto& part : std::filesystem::path(f.path)) {
          if(part != "..") {
            break;
          }
          up++;
        }
        depth = std::max(depth, up);
        files.push_back(std::move(f));
      }
    }
    std::filesystem::path model_directory = root / "model";
    for(size_t k = 0; k < depth; k++) {
      model_directory /= "d";
    }
    std::filesystem::create_directories(model_directory);
    std::map<std::string, std::string> file_map;
    for(auto& f : files) {
      const std::filesystem::path destination =
        (model_directory / f.path).lexically_normal();
      std::filesystem::create_directories(destination.parent_path());
      std::vector<char> bytes(Rows(f.dataset));
      if(!bytes.empty()) {
        f.dataset.read(bytes.data(), H5::PredType::NATIVE_UINT8);
      }
      std::ofstream out(destination, std::ios::binary);
      out.write(bytes.data(), std::streamsize(bytes.size()));
      if(!out) {
        throw shift::core::Exception(error_prefix + "could not write " +
                                     destination.string());
      }
      // the model finds the file here by the names it gave it, whatever
      // its include directories (they may not exist on this computer); the
      // files those name, such as an .mtl, are beside them as they were
      std::istringstream names(f.names);
      for(std::string name; std::getline(names, name);) {
        if(!name.empty()) {
          file_map[name] = destination.string();
        }
      }
      file_map[f.original] = destination.string();
    }
    /* the model, without its loggers */
    H5::DataSet json_dataset = file.openDataSet("/model/json");
    std::string text;
    json_dataset.read(text, json_dataset.getStrType());
    Json::Value model;
    {
      Json::CharReaderBuilder builder;
      std::string errors;
      std::istringstream in(text);
      if(!Json::parseFromStream(builder, in, &model, &errors)) {
        throw shift::core::Exception(error_prefix + "its model: " + errors);
      }
    }
    if(model.isMember("model_parameters")) {
      model["model_parameters"].removeMember("loggers");
    }
    std::filesystem::path model_file =
      model_directory /
      std::filesystem::path(ReadStringAttribute(json_dataset, "filename"))
        .filename();
    model_file.replace_extension(".json");
    {
      std::ofstream out(model_file);
      Json::StreamWriterBuilder builder;
      out << Json::writeString(builder, model);
    }
    LoadModel(model_file.string(), std::move(file_map));

    /* the rows: q, dq and the states of free bodies */
    ReportLoad("Reading the results");
    std::vector<double> times(Rows(file.openDataSet("/time")));
    file.openDataSet("/time").read(times.data(), H5::PredType::NATIVE_DOUBLE);
    const size_t rows = times.size();
    const size_t q_count = context_->get_q_count();
    const size_t dq_count = context_->get_dq_count();
    const size_t free_count = state_size_ - q_count - dq_count;
    std::vector<double> states(rows * state_size_, 0.0);
    const std::pair<const char*, size_t> blocks[] = {
      {"q", q_count}, {"dq", dq_count}, {"free", free_count}};
    size_t offset = 0;
    for(const auto& [name, count] : blocks) {
      if(count == 0) {
        continue;
      }
      const std::string path = std::string("/states/") + name;
      if(!file.nameExists("/states") || !file.nameExists(path)) {
        throw shift::core::Exception(
          error_prefix + "it has no " + path +
          " (log_all_channels was false, or the log is older than the "
          "logging of free bodies)");
      }
      H5::DataSet dataset = file.openDataSet(path);
      hsize_t dims[2] = {0, 0};
      dataset.getSpace().getSimpleExtentDims(dims);
      if(dims[0] < rows || dims[1] != count) {
        throw shift::core::Exception(error_prefix + path + " does not match "
                                                         "the model");
      }
      std::vector<double> block(rows * count);
      H5::DataSpace file_space = dataset.getSpace();
      const hsize_t start[2] = {0, 0}, size[2] = {rows, count};
      file_space.selectHyperslab(H5S_SELECT_SET, size, start);
      H5::DataSpace memory_space(2, size);
      dataset.read(block.data(), H5::PredType::NATIVE_DOUBLE, memory_space,
                   file_space);
      for(size_t r = 0; r < rows; r++) {
        std::copy(block.begin() + r * count, block.begin() + (r + 1) * count,
                  states.begin() + r * state_size_ + offset);
      }
      offset += count;
    }

    /* the changes of the deformable terrains, row by row */
    struct TerrainRows {
      std::vector<int64_t> starts;
      std::vector<int32_t> indices;
      std::vector<float> levels;
    };
    std::vector<TerrainRows> terrain_rows(scm_views_.size());
    for(size_t v = 0; v < scm_views_.size(); v++) {
      const ScmView& view = scm_views_[v];
      const std::string path = "/terrain/" + view.scm->get_name();
      if(!file.nameExists("/terrain") || !file.nameExists(path)) {
        throw shift::core::Exception(error_prefix + "it has no " + path);
      }
      H5::Group group = file.openGroup(path);
      int64_t counts[2] = {0, 0};
      group.openAttribute("node_count").read(H5::PredType::NATIVE_INT64,
                                             counts);
      if(counts[0] != view.nx || counts[1] != view.ny) {
        throw shift::core::Exception(error_prefix + path +
                                     " does not match the grid of the model");
      }
      TerrainRows& t = terrain_rows[v];
      H5::DataSet starts = group.openDataSet("row_start");
      H5::DataSet indices = group.openDataSet("index");
      H5::DataSet levels = group.openDataSet("level");
      t.starts.resize(Rows(starts));
      t.indices.resize(Rows(indices));
      t.levels.resize(Rows(levels));
      if(t.starts.size() < rows || t.levels.size() != t.indices.size()) {
        throw shift::core::Exception(error_prefix + path + " is incomplete");
      }
      if(!t.starts.empty()) {
        starts.read(t.starts.data(), H5::PredType::NATIVE_INT64);
      }
      if(!t.indices.empty()) {
        indices.read(t.indices.data(), H5::PredType::NATIVE_INT32);
        levels.read(t.levels.data(), H5::PredType::NATIVE_FLOAT);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(size_t r = 0; r < rows; r++) {
      if(!times_.empty() && !(times[r] > times_.back())) {
        continue; // a repeated time: Observe keeps the first
      }
      times_.push_back(times[r]);
      states_.insert(states_.end(), states.begin() + r * state_size_,
                     states.begin() + (r + 1) * state_size_);
      knot_deltas_.push_back(deltas_.size());
      // a log holds no lights: they stay as the model starts them
      for(size_t k = 0; k < light_sources_.size(); k++) {
        light_values_.push_back(scene_.lights[k].intensity);
      }
      for(size_t v = 0; v < scm_views_.size(); v++) {
        ScmView& view = scm_views_[v];
        const TerrainRows& t = terrain_rows[v];
        const int64_t end =
          r + 1 < t.starts.size() ? t.starts[r + 1] : int64_t(t.indices.size());
        for(int64_t k = t.starts[r]; k < end; k++) {
          const int32_t index = t.indices[size_t(k)];
          if(index < 0 || index >= view.nx * view.ny) {
            throw shift::core::Exception(error_prefix + "a node of /terrain/" +
                                         view.scm->get_name() +
                                         " is beyond the grid");
          }
          float& level = view.worker[size_t(index)];
          const float after = t.levels[size_t(k)];
          if(level != after) {
            deltas_.push_back({int32_t(v), index, level, after});
            level = after;
          }
        }
      }
    }
  } catch(H5::Exception& e) {
    throw shift::core::Exception(error_prefix + e.getDetailMsg());
  }
  if(!dem_views_.empty()) {
    SHIFT_WARNING("Log " + filename + ": a log holds no grains of the DEM; "
                  "they are shown as the model placed them");
  }
  if(has_results()) {
    std::lock_guard<std::mutex> lock(mutex_);
    shown_time_ = times_.front();
  }
  ShowTime(shown_time_);
}
