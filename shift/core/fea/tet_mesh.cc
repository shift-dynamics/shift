#include "shift/core/fea/tet_mesh.h"

#include <algorithm>
#include <cmath>
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

shift::core::fea::TetMesh shift::core::fea::MeshVolume(
  const ClosedSurface& surface, double element_size,
  double minimum_element_size) {
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

  /* Netgen keeps its meshing parameters in a global object */
  static std::once_flag initialized;
  static std::mutex netgen_mutex;
  std::call_once(initialized, [] { nglib::Ng_Init(); });
  std::lock_guard<std::mutex> lock(netgen_mutex);

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
