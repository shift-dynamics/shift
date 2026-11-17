#include "shift/core/fea/tet_mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include "shift/core/exception.h"

namespace nglib {
#include <nglib.h>
}

double shift::core::fea::TetMesh::Volume() const {
  double volume = 0.0;
  for(const std::array<int, 10>& t : tets) {
    const ksl_vec3_t& p0 = nodes[t[0]];
    ksl_vec3_t a, b, c, axb;
    ksl_subtract_vv(&nodes[t[1]], &p0, &a);
    ksl_subtract_vv(&nodes[t[2]], &p0, &b);
    ksl_subtract_vv(&nodes[t[3]], &p0, &c);
    ksl_cross_vv(&a, &b, &axb);
    volume += ksl_dot_vv(&axb, &c) / 6.0;
  }
  return volume;
}

/* the distance from p to the triangle a, b, c (Ericson, Real-Time Collision
Detection, 2005, 5.1.5) */
static double TriangleDistance(const ksl_vec3_t& p, const ksl_vec3_t& a,
                               const ksl_vec3_t& b, const ksl_vec3_t& c) {
  ksl_vec3_t ab, ac, ap;
  ksl_subtract_vv(&b, &a, &ab);
  ksl_subtract_vv(&c, &a, &ac);
  ksl_subtract_vv(&p, &a, &ap);
  const double d1 = ksl_dot_vv(&ab, &ap), d2 = ksl_dot_vv(&ac, &ap);
  auto at = [&](double v, double w) {
    ksl_vec3_t q = a;
    ksl_axpy_vv(v, &ab, &q);
    ksl_axpy_vv(w, &ac, &q);
    ksl_vec3_t d;
    ksl_subtract_vv(&p, &q, &d);
    return ksl_vec3_l2norm(&d);
  };
  if(d1 <= 0.0 && d2 <= 0.0) {
    return at(0.0, 0.0);
  }
  ksl_vec3_t bp;
  ksl_subtract_vv(&p, &b, &bp);
  const double d3 = ksl_dot_vv(&ab, &bp), d4 = ksl_dot_vv(&ac, &bp);
  if(d3 >= 0.0 && d4 <= d3) {
    return at(1.0, 0.0);
  }
  const double vc = d1 * d4 - d3 * d2;
  if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return at(d1 / (d1 - d3), 0.0);
  }
  ksl_vec3_t cp;
  ksl_subtract_vv(&p, &c, &cp);
  const double d5 = ksl_dot_vv(&ab, &cp), d6 = ksl_dot_vv(&ac, &cp);
  if(d6 >= 0.0 && d5 <= d6) {
    return at(0.0, 1.0);
  }
  const double vb = d5 * d2 - d1 * d6;
  if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return at(0.0, d2 / (d2 - d6));
  }
  const double va = d3 * d6 - d5 * d4;
  if(va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return at(1.0 - w, w);
  }
  const double denominator = 1.0 / (va + vb + vc);
  return at(vb * denominator, vc * denominator);
}

bool shift::core::fea::InsideSurface(const ClosedSurface& surface,
                                     const ksl_vec3_t& point,
                                     double* distance) {
  /* the solid angle of each triangle seen from the point (Van Oosterom and
  Strackee, IEEE Transactions on Biomedical Engineering 30(2), 1983) */
  double angle = 0.0;
  double nearest = 1e300;
  for(const std::array<int, 3>& t : surface.triangles) {
    ksl_vec3_t r[3];
    double l[3];
    for(int k = 0; k < 3; k++) {
      ksl_subtract_vv(&surface.points[t[k]], &point, &r[k]);
      l[k] = ksl_vec3_l2norm(&r[k]);
    }
    ksl_vec3_t r12;
    ksl_cross_vv(&r[1], &r[2], &r12);
    const double numerator = ksl_dot_vv(&r[0], &r12);
    const double denominator = l[0] * l[1] * l[2] +
                               ksl_dot_vv(&r[0], &r[1]) * l[2] +
                               ksl_dot_vv(&r[0], &r[2]) * l[1] +
                               ksl_dot_vv(&r[1], &r[2]) * l[0];
    angle += 2.0 * std::atan2(numerator, denominator);
    if(distance) {
      nearest = std::min(
        nearest, TriangleDistance(point, surface.points[t[0]],
                                  surface.points[t[1]], surface.points[t[2]]));
    }
  }
  if(distance) {
    *distance = nearest;
  }
  return angle / (4.0 * M_PI) > 0.5;
}

/* Netgen keeps its meshing parameters in a global object: one meshing at a
time */
static std::mutex& NetgenMutex() {
  static std::once_flag initialized;
  static std::mutex netgen_mutex;
  std::call_once(initialized, [] { nglib::Ng_Init(); });
  return netgen_mutex;
}

shift::core::fea::ClosedSurface shift::core::fea::WeldSurface(
  const std::vector<ksl_vec3_t>& corners,
  const std::vector<std::array<int, 3>>& triangles, double tolerance) {
  /* the corners in cells of the tolerance; each matched against the points
  of its cell and the cells around */
  ClosedSurface surface;
  const double cell = tolerance > 0.0 ? tolerance : 1e-300;
  std::map<std::array<int64_t, 3>, std::vector<int>> cells;
  std::vector<int> point_of(corners.size(), -1);
  for(size_t i = 0; i < corners.size(); i++) {
    const ksl_vec3_t& c = corners[i];
    const std::array<int64_t, 3> key = {int64_t(std::floor(c.x / cell)),
                                        int64_t(std::floor(c.y / cell)),
                                        int64_t(std::floor(c.z / cell))};
    for(int64_t a = -1; a <= 1 && point_of[i] < 0; a++) {
      for(int64_t b = -1; b <= 1 && point_of[i] < 0; b++) {
        for(int64_t d = -1; d <= 1 && point_of[i] < 0; d++) {
          auto it = cells.find({key[0] + a, key[1] + b, key[2] + d});
          if(it == cells.end()) {
            continue;
          }
          for(int p : it->second) {
            ksl_vec3_t r;
            ksl_subtract_vv(&surface.points[size_t(p)], &c, &r);
            if(ksl_vec3_l2norm(&r) <= tolerance) {
              point_of[i] = p;
              break;
            }
          }
        }
      }
    }
    if(point_of[i] < 0) {
      point_of[i] = static_cast<int>(surface.points.size());
      surface.points.push_back(c);
      cells[key].push_back(point_of[i]);
    }
  }
  for(const std::array<int, 3>& t : triangles) {
    for(int k : t) {
      if(k < 0 || size_t(k) >= corners.size()) {
        throw shift::core::Exception(
          "WeldSurface: a triangle has a bad corner index");
      }
    }
    const std::array<int, 3> w = {point_of[size_t(t[0])],
                                  point_of[size_t(t[1])],
                                  point_of[size_t(t[2])]};
    if(w[0] != w[1] && w[1] != w[2] && w[2] != w[0]) {
      surface.triangles.push_back(w);
      surface.face.push_back(-1);
    }
  }
  return surface;
}

/* the unit normal of a triangle of a surface */
static ksl_vec3_t TriangleNormal(const shift::core::fea::ClosedSurface& s,
                                 const std::array<int, 3>& t) {
  ksl_vec3_t a, b, n;
  ksl_subtract_vv(&s.points[size_t(t[1])], &s.points[size_t(t[0])], &a);
  ksl_subtract_vv(&s.points[size_t(t[2])], &s.points[size_t(t[0])], &b);
  ksl_cross_vv(&a, &b, &n);
  const double length = ksl_vec3_l2norm(&n);
  if(length > 0.0) {
    ksl_product_av(1.0 / length, &n, &n);
  }
  return n;
}

shift::core::fea::ClosedSurface shift::core::fea::RemeshSurface(
  const ClosedSurface& surface, double element_size,
  double minimum_element_size, double feature_angle) {
  if(surface.triangles.empty() || element_size <= 0.0) {
    throw shift::core::Exception(
      "RemeshSurface: an empty surface or no element size");
  }
  ClosedSurface remeshed;
  {
    std::lock_guard<std::mutex> lock(NetgenMutex());
    nglib::Ng_STL_Geometry* geometry = nglib::Ng_STL_NewGeometry();
    for(const std::array<int, 3>& t : surface.triangles) {
      double p[3][3];
      for(int k = 0; k < 3; k++) {
        const ksl_vec3_t& x = surface.points[size_t(t[k])];
        p[k][0] = x.x;
        p[k][1] = x.y;
        p[k][2] = x.z;
      }
      ksl_vec3_t n = TriangleNormal(surface, t);
      double normal[3] = {n.x, n.y, n.z};
      nglib::Ng_STL_AddTriangle(geometry, p[0], p[1], p[2], normal);
    }
    nglib::Ng_Meshing_Parameters parameters;
    parameters.maxh = element_size;
    parameters.minh = minimum_element_size;
    parameters.second_order = 0;
    nglib::Ng_Mesh* mesh = nglib::Ng_NewMesh();
    nglib::Ng_Result result = nglib::Ng_STL_InitSTLGeometry(geometry);
    if(result == nglib::NG_OK) {
      result = nglib::Ng_STL_MakeEdges(geometry, mesh, &parameters);
    }
    /* finer where the surface curves: across each edge of the given
    triangles that is not a feature, the radius of curvature is about the
    distance between the triangles' centroids (across the edge) over the
    angle between their normals; a side of 0.4 of it gives about 16 sides
    round a cylinder */
    if(result == nglib::NG_OK) {
      std::map<std::pair<int, int>, std::vector<size_t>> edges;
      for(size_t k = 0; k < surface.triangles.size(); k++) {
        const std::array<int, 3>& t = surface.triangles[k];
        for(int e = 0; e < 3; e++) {
          edges[std::minmax(t[e], t[(e + 1) % 3])].push_back(k);
        }
      }
      for(const auto& [edge, sides] : edges) {
        if(sides.size() != 2) {
          continue;
        }
        const ksl_vec3_t n0 = TriangleNormal(surface, surface.triangles[sides[0]]);
        const ksl_vec3_t n1 = TriangleNormal(surface, surface.triangles[sides[1]]);
        const double angle =
          std::acos(std::clamp(ksl_dot_vv(&n0, &n1), -1.0, 1.0));
        if(!(angle > 1e-6) || angle >= feature_angle) {
          continue;
        }
        const ksl_vec3_t& a = surface.points[size_t(edge.first)];
        const ksl_vec3_t& b = surface.points[size_t(edge.second)];
        ksl_vec3_t along, c[2], w;
        ksl_subtract_vv(&b, &a, &along);
        ksl_vec3_normalize(&along);
        for(int s = 0; s < 2; s++) {
          c[s] = ksl_vec3(0.0, 0.0, 0.0);
          for(int k : surface.triangles[sides[size_t(s)]]) {
            ksl_axpy_vv(1.0 / 3.0, &surface.points[size_t(k)], &c[s]);
          }
        }
        ksl_subtract_vv(&c[1], &c[0], &w);
        ksl_vec3_t axial;
        ksl_product_av(ksl_dot_vv(&w, &along), &along, &axial);
        ksl_subtract_vv(&w, &axial, &w);
        const double size = std::max(
          0.4 * ksl_vec3_l2norm(&w) / angle, minimum_element_size);
        if(size >= element_size) {
          continue;
        }
        double pa[3] = {a.x, a.y, a.z}, pb[3] = {b.x, b.y, b.z};
        double pm[3] = {0.5 * (a.x + b.x), 0.5 * (a.y + b.y),
                        0.5 * (a.z + b.z)};
        for(double* p : {pa, pb, pm}) {
          nglib::Ng_RestrictMeshSizePoint(mesh, p, size);
        }
      }
      result = nglib::Ng_STL_GenerateSurfaceMesh(geometry, mesh, &parameters);
    }
    const int point_count = nglib::Ng_GetNP(mesh);
    const int triangle_count = nglib::Ng_GetNSE(mesh);
    if(result == nglib::NG_OK && triangle_count > 0) {
      remeshed.points.resize(size_t(point_count));
      for(int i = 0; i < point_count; i++) {
        double x[3];
        nglib::Ng_GetPoint(mesh, i + 1, x);
        remeshed.points[size_t(i)] = {x[0], x[1], x[2]};
      }
      for(int e = 0; e < triangle_count; e++) {
        int pi[8];
        if(nglib::Ng_GetSurfaceElement(mesh, e + 1, pi) != nglib::NG_TRIG) {
          result = nglib::NG_ERROR;
          break;
        }
        remeshed.triangles.push_back({pi[0] - 1, pi[1] - 1, pi[2] - 1});
      }
    }
    nglib::Ng_DeleteMesh(mesh);
    nglib::Ng_STL_DeleteGeometry(geometry);
    if(result != nglib::NG_OK || remeshed.triangles.empty()) {
      throw shift::core::Exception(
        "RemeshSurface: Netgen could not mesh the surface (is it closed, "
        "and free of self-intersections?)");
    }
  }

  /* the points Netgen kept but no triangle uses (the surface's own) go */
  std::vector<int> used(remeshed.points.size(), -1);
  ClosedSurface compact;
  for(std::array<int, 3>& t : remeshed.triangles) {
    for(int& k : t) {
      if(used[size_t(k)] < 0) {
        used[size_t(k)] = static_cast<int>(compact.points.size());
        compact.points.push_back(remeshed.points[size_t(k)]);
      }
      k = used[size_t(k)];
    }
  }
  compact.triangles = remeshed.triangles;

  /* normals pointing out: a positive enclosed volume */
  double enclosed = 0.0;
  for(const std::array<int, 3>& t : compact.triangles) {
    ksl_vec3_t axb;
    ksl_cross_vv(&compact.points[size_t(t[0])], &compact.points[size_t(t[1])],
                 &axb);
    enclosed += ksl_dot_vv(&axb, &compact.points[size_t(t[2])]);
  }
  if(enclosed < 0.0) {
    for(std::array<int, 3>& t : compact.triangles) {
      std::swap(t[1], t[2]);
    }
  }

  /* the faces: triangles joined across edges with normals within the
  feature angle */
  const size_t count = compact.triangles.size();
  std::map<std::pair<int, int>, std::vector<size_t>> edge_triangles;
  std::vector<ksl_vec3_t> normals(count);
  for(size_t k = 0; k < count; k++) {
    const std::array<int, 3>& t = compact.triangles[k];
    normals[k] = TriangleNormal(compact, t);
    for(int e = 0; e < 3; e++) {
      edge_triangles[std::minmax(t[e], t[(e + 1) % 3])].push_back(k);
    }
  }
  const double cos_feature = std::cos(feature_angle);
  compact.face.assign(count, -1);
  int faces = 0;
  for(size_t seed = 0; seed < count; seed++) {
    if(compact.face[seed] >= 0) {
      continue;
    }
    std::vector<size_t> stack = {seed};
    compact.face[seed] = faces;
    while(!stack.empty()) {
      const size_t k = stack.back();
      stack.pop_back();
      const std::array<int, 3>& t = compact.triangles[k];
      for(int e = 0; e < 3; e++) {
        for(size_t j : edge_triangles[std::minmax(t[e], t[(e + 1) % 3])]) {
          if(compact.face[j] < 0 &&
             ksl_dot_vv(&normals[k], &normals[j]) > cos_feature) {
            compact.face[j] = faces;
            stack.push_back(j);
          }
        }
      }
    }
    faces++;
  }
  return compact;
}

shift::core::fea::TetMesh shift::core::fea::MeshVolume(
  const ClosedSurface& surface, double element_size,
  double minimum_element_size, const std::vector<ksl_vec3_t>& interior) {
  if(surface.triangles.empty() ||
     surface.face.size() != surface.triangles.size() || element_size <= 0.0) {
    throw shift::core::Exception(
      "MeshVolume: an empty surface, a face for each triangle, and a "
      "positive element size are needed");
  }

  /* closed and consistently oriented: every edge used once in each
  direction; and the volume it encloses, by the divergence theorem */
  std::map<std::pair<int, int>, int> directed;
  double enclosed = 0.0;
  for(const std::array<int, 3>& t : surface.triangles) {
    for(int k = 0; k < 3; k++) {
      const int i = t[k], j = t[(k + 1) % 3];
      if(i < 0 || j < 0 || i >= static_cast<int>(surface.points.size()) ||
         j >= static_cast<int>(surface.points.size()) || i == j) {
        throw shift::core::Exception(
          "MeshVolume: a triangle of the surface has a bad point index");
      }
      directed[{i, j}]++;
    }
    ksl_vec3_t axb;
    ksl_cross_vv(&surface.points[t[0]], &surface.points[t[1]], &axb);
    enclosed += ksl_dot_vv(&axb, &surface.points[t[2]]) / 6.0;
  }
  for(const auto& [edge, count] : directed) {
    auto opposite = directed.find({edge.second, edge.first});
    if(count != 1 || opposite == directed.end() || opposite->second != 1) {
      throw shift::core::Exception(
        "MeshVolume: the surface is not closed: the edge between points " +
        std::to_string(edge.first) + " and " + std::to_string(edge.second) +
        " is not used once in each direction");
    }
  }
  if(!(enclosed > 0.0)) {
    throw shift::core::Exception(
      "MeshVolume: the surface encloses no volume (are its normals pointing "
      "in?)");
  }

  std::lock_guard<std::mutex> lock(NetgenMutex());

  nglib::Ng_Mesh* mesh = nglib::Ng_NewMesh();
  for(const ksl_vec3_t& p : surface.points) {
    double x[3] = {p.x, p.y, p.z};
    nglib::Ng_AddPoint(mesh, x);
  }
  /* Netgen takes the triangles with their normals pointing out, as given */
  for(const std::array<int, 3>& t : surface.triangles) {
    int pi[3] = {t[0] + 1, t[1] + 1, t[2] + 1};
    nglib::Ng_AddSurfaceElement(mesh, nglib::NG_TRIG, pi);
  }
  /* the interior points inside and clear of the surface, locked; two at
  one place are one */
  std::vector<bool> locked(interior.size(), false);
  for(size_t i = 0; i < interior.size(); i++) {
    double clearance = 0.0;
    if(!InsideSurface(surface, interior[i], &clearance) ||
       clearance < 0.2 * element_size) {
      continue;
    }
    bool repeated = false;
    for(size_t j = 0; j < i; j++) {
      ksl_vec3_t d;
      ksl_subtract_vv(&interior[i], &interior[j], &d);
      repeated = repeated || (locked[j] && ksl_vec3_l2norm(&d) == 0.0);
    }
    locked[i] = true;
    if(!repeated) {
      double x[3] = {interior[i].x, interior[i].y, interior[i].z};
      nglib::Ng_AddLockedPoint(mesh, x);
    }
  }

  nglib::Ng_Meshing_Parameters parameters;
  parameters.maxh = element_size;
  parameters.minh = minimum_element_size;
  parameters.second_order = 0;
  const nglib::Ng_Result result =
    nglib::Ng_GenerateVolumeMesh(mesh, &parameters);
  const int point_count = nglib::Ng_GetNP(mesh);
  const int tet_count = nglib::Ng_GetNE(mesh);
  if(result != nglib::NG_OK || tet_count == 0 ||
     point_count < static_cast<int>(surface.points.size())) {
    nglib::Ng_DeleteMesh(mesh);
    throw shift::core::Exception(
      "MeshVolume: Netgen could not fill the surface with tetrahedra (is it "
      "closed, and free of self-intersections?)");
  }

  TetMesh tet_mesh;
  tet_mesh.nodes.resize(point_count);
  for(int i = 0; i < point_count; i++) {
    double x[3];
    nglib::Ng_GetPoint(mesh, i + 1, x);
    tet_mesh.nodes[i] = {x[0], x[1], x[2]};
  }
  tet_mesh.corner_count = static_cast<unsigned int>(point_count);
  tet_mesh.tets.resize(tet_count);
  for(int e = 0; e < tet_count; e++) {
    int pi[10];
    nglib::Ng_GetVolumeElement(mesh, e + 1, pi);
    std::array<int, 10>& t = tet_mesh.tets[e];
    t.fill(-1);
    for(int k = 0; k < 4; k++) {
      t[k] = pi[k] - 1;
    }
  }
  nglib::Ng_DeleteMesh(mesh);

  /* the interior points' nodes, where Netgen left them */
  tet_mesh.interior_nodes.assign(interior.size(), -1);
  for(size_t i = 0; i < interior.size(); i++) {
    if(!locked[i]) {
      continue;
    }
    for(int n = 0; n < point_count; n++) {
      ksl_vec3_t d;
      ksl_subtract_vv(&tet_mesh.nodes[n], &interior[i], &d);
      if(ksl_vec3_l2norm(&d) <= 1e-12 * element_size) {
        tet_mesh.interior_nodes[i] = n;
        break;
      }
    }
    if(tet_mesh.interior_nodes[i] < 0) {
      throw shift::core::Exception(
        "MeshVolume: Netgen did not keep an interior point as a node");
    }
  }

  /* corner 3 on the positive side of 0, 1, 2 */
  for(std::array<int, 10>& t : tet_mesh.tets) {
    ksl_vec3_t a, b, c, axb;
    ksl_subtract_vv(&tet_mesh.nodes[t[1]], &tet_mesh.nodes[t[0]], &a);
    ksl_subtract_vv(&tet_mesh.nodes[t[2]], &tet_mesh.nodes[t[0]], &b);
    ksl_subtract_vv(&tet_mesh.nodes[t[3]], &tet_mesh.nodes[t[0]], &c);
    ksl_cross_vv(&a, &b, &axb);
    if(ksl_dot_vv(&axb, &c) < 0.0) {
      std::swap(t[1], t[2]);
    }
  }

  /* a mid-side node at the middle of each edge, shared by the elements
  around it */
  std::map<std::pair<int, int>, int> mid;
  auto mid_node = [&](int i, int j) {
    const std::pair<int, int> key = std::minmax(i, j);
    auto found = mid.find(key);
    if(found != mid.end()) {
      return found->second;
    }
    const ksl_vec3_t& a = tet_mesh.nodes[i];
    const ksl_vec3_t& b = tet_mesh.nodes[j];
    const int index = static_cast<int>(tet_mesh.nodes.size());
    tet_mesh.nodes.push_back(
      {0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z)});
    mid.emplace(key, index);
    return index;
  };
  static const int kEdges[6][2] = {{0, 1}, {1, 2}, {2, 0},
                                   {0, 3}, {1, 3}, {2, 3}};
  for(std::array<int, 10>& t : tet_mesh.tets) {
    for(int k = 0; k < 6; k++) {
      t[4 + k] = mid_node(t[kEdges[k][0]], t[kEdges[k][1]]);
    }
  }

  /* the boundary: the given triangles, whose points Netgen keeps */
  tet_mesh.boundary.resize(surface.triangles.size());
  tet_mesh.boundary_face = surface.face;
  for(size_t f = 0; f < surface.triangles.size(); f++) {
    const std::array<int, 3>& t = surface.triangles[f];
    tet_mesh.boundary[f] = {t[0],
                            t[1],
                            t[2],
                            mid_node(t[0], t[1]),
                            mid_node(t[1], t[2]),
                            mid_node(t[2], t[0])};
  }
  const double volume = tet_mesh.Volume();
  if(std::fabs(volume - enclosed) > 1e-9 * enclosed) {
    throw shift::core::Exception(
      "MeshVolume: Netgen's tetrahedra fill " + std::to_string(volume) +
      " of the volume " + std::to_string(enclosed) +
      " the surface encloses (does it intersect itself?)");
  }
  if(tet_mesh.nodes.size() != tet_mesh.corner_count + mid.size()) {
    throw shift::core::Exception(
      "MeshVolume: a boundary triangle is not a face of the tetrahedra");
  }
  return tet_mesh;
}
