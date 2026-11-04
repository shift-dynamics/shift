#include "shift/core/geometry/mass_properties.h"

#include <array>
#include <cmath>
#include <map>
#include <utility>

bool shift::core::geometry::IsClosedSurface(
  const std::vector<ksl_vec3_t>& vertices,
  const std::vector<unsigned int>& faces) {
  if(faces.empty() || faces.size() % 3 != 0) {
    return false;
  }
  /* weld vertices with the same position, e.g. copies at texture seams */
  std::map<std::array<double, 3>, unsigned int> welded;
  std::vector<unsigned int> index(vertices.size());
  for(size_t i = 0; i < vertices.size(); i++) {
    const std::array<double, 3> key = {vertices[i].x, vertices[i].y,
                                       vertices[i].z};
    index[i] = welded.emplace(key, static_cast<unsigned int>(welded.size()))
                 .first->second;
  }
  /* each directed edge once, and its reverse once */
  std::map<std::pair<unsigned int, unsigned int>, int> edges;
  for(size_t f = 0; f < faces.size(); f += 3) {
    for(int k = 0; k < 3; k++) {
      const unsigned int a = index[faces[f + k]];
      const unsigned int b = index[faces[f + (k + 1) % 3]];
      if(a == b) {
        return false; // degenerate triangle
      }
      if(++edges[{a, b}] > 1) {
        return false;
      }
    }
  }
  for(const auto& edge : edges) {
    auto reverse = edges.find({edge.first.second, edge.first.first});
    if(reverse == edges.end() || reverse->second != 1) {
      return false;
    }
  }
  return true;
}

int shift::core::geometry::CloseHoles(std::vector<ksl_vec3_t>& vertices,
                                      std::vector<unsigned int>& faces) {
  if(faces.size() % 3 != 0) {
    return -1;
  }
  std::map<std::array<double, 3>, unsigned int> welded;
  std::vector<unsigned int> index(vertices.size());
  std::vector<unsigned int> representative; // a vertex for each welded one
  for(size_t i = 0; i < vertices.size(); i++) {
    const std::array<double, 3> key = {vertices[i].x, vertices[i].y,
                                       vertices[i].z};
    auto inserted =
      welded.emplace(key, static_cast<unsigned int>(welded.size()));
    if(inserted.second) {
      representative.push_back(static_cast<unsigned int>(i));
    }
    index[i] = inserted.first->second;
  }
  std::map<std::pair<unsigned int, unsigned int>, int> edges;
  for(size_t f = 0; f < faces.size(); f += 3) {
    for(int k = 0; k < 3; k++) {
      const unsigned int a = index[faces[f + k]];
      const unsigned int b = index[faces[f + (k + 1) % 3]];
      if(a != b && ++edges[{a, b}] > 1) {
        return -1; // not consistently oriented, or nonmanifold
      }
    }
  }
  /* a boundary edge a -> b has no b -> a; the cap traverses it as b -> a, so
   * follow the loop backwards: next[b] = a */
  std::map<unsigned int, unsigned int> next;
  for(const auto& edge : edges) {
    const unsigned int a = edge.first.first, b = edge.first.second;
    if(edges.find({b, a}) == edges.end()) {
      if(!next.emplace(b, a).second) {
        return -1; // a vertex where boundaries touch
      }
    }
  }
  std::vector<ksl_vec3_t> new_vertices = vertices;
  std::vector<unsigned int> new_faces = faces;
  int holes = 0;
  while(!next.empty()) {
    std::vector<unsigned int> loop;
    unsigned int v = next.begin()->first;
    while(true) {
      auto found = next.find(v);
      if(found == next.end()) {
        break;
      }
      loop.push_back(v);
      v = found->second;
      next.erase(found);
    }
    if(v != loop.front() || loop.size() < 3) {
      return -1; // the boundary is not a closed loop
    }
    ksl_vec3_t center = {{0.0, 0.0, 0.0}};
    for(auto w : loop) {
      const ksl_vec3_t& p = vertices[representative[w]];
      center.x += p.x;
      center.y += p.y;
      center.z += p.z;
    }
    const double n = static_cast<double>(loop.size());
    center = {{center.x / n, center.y / n, center.z / n}};
    const unsigned int c = static_cast<unsigned int>(new_vertices.size());
    new_vertices.push_back(center);
    for(size_t k = 0; k < loop.size(); k++) {
      new_faces.push_back(representative[loop[k]]);
      new_faces.push_back(representative[loop[(k + 1) % loop.size()]]);
      new_faces.push_back(c);
    }
    holes++;
  }
  vertices.swap(new_vertices);
  faces.swap(new_faces);
  return holes;
}

static void Subexpressions(double w0, double w1, double w2, double& f1,
                           double& f2, double& f3, double& g0, double& g1,
                           double& g2) {
  const double temp0 = w0 + w1;
  f1 = temp0 + w2;
  const double temp1 = w0 * w0;
  const double temp2 = temp1 + w1 * temp0;
  f2 = temp2 + w2 * f1;
  f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
  g0 = f2 + w0 * (f1 + w0);
  g1 = f2 + w1 * (f1 + w1);
  g2 = f2 + w2 * (f1 + w2);
}

bool shift::core::geometry::ComputeMassProperties(
  const std::vector<ksl_vec3_t>& vertices,
  const std::vector<unsigned int>& faces, MassProperties& properties) {
  /* integrals of 1, x, y, z, x^2, y^2, z^2, xy, yz, zx over the solid */
  double integral[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for(size_t f = 0; f + 2 < faces.size(); f += 3) {
    const ksl_vec3_t& p0 = vertices[faces[f]];
    const ksl_vec3_t& p1 = vertices[faces[f + 1]];
    const ksl_vec3_t& p2 = vertices[faces[f + 2]];
    /* the normal (p1 - p0) x (p2 - p0), twice the area */
    const double a1 = p1.x - p0.x, b1 = p1.y - p0.y, c1 = p1.z - p0.z;
    const double a2 = p2.x - p0.x, b2 = p2.y - p0.y, c2 = p2.z - p0.z;
    const double d0 = b1 * c2 - b2 * c1;
    const double d1 = a2 * c1 - a1 * c2;
    const double d2 = a1 * b2 - a2 * b1;
    double f1x, f2x, f3x, g0x, g1x, g2x;
    double f1y, f2y, f3y, g0y, g1y, g2y;
    double f1z, f2z, f3z, g0z, g1z, g2z;
    Subexpressions(p0.x, p1.x, p2.x, f1x, f2x, f3x, g0x, g1x, g2x);
    Subexpressions(p0.y, p1.y, p2.y, f1y, f2y, f3y, g0y, g1y, g2y);
    Subexpressions(p0.z, p1.z, p2.z, f1z, f2z, f3z, g0z, g1z, g2z);
    integral[0] += d0 * f1x;
    integral[1] += d0 * f2x;
    integral[2] += d1 * f2y;
    integral[3] += d2 * f2z;
    integral[4] += d0 * f3x;
    integral[5] += d1 * f3y;
    integral[6] += d2 * f3z;
    integral[7] += d0 * (p0.y * g0x + p1.y * g1x + p2.y * g2x);
    integral[8] += d1 * (p0.z * g0y + p1.z * g1y + p2.z * g2y);
    integral[9] += d2 * (p0.x * g0z + p1.x * g1z + p2.x * g2z);
  }
  integral[0] /= 6.0;
  for(int i = 1; i < 4; i++) {
    integral[i] /= 24.0;
  }
  for(int i = 4; i < 7; i++) {
    integral[i] /= 60.0;
  }
  for(int i = 7; i < 10; i++) {
    integral[i] /= 120.0;
  }
  if(integral[0] < 0.0) { // clockwise triangles
    for(double& value : integral) {
      value = -value;
    }
  }
  const double volume = integral[0];
  if(!(volume > 0.0)) {
    return false;
  }
  properties.volume = volume;
  const double cx = integral[1] / volume;
  const double cy = integral[2] / volume;
  const double cz = integral[3] / volume;
  properties.centroid = {{cx, cy, cz}};
  properties.Ixx = integral[5] + integral[6] - volume * (cy * cy + cz * cz);
  properties.Iyy = integral[4] + integral[6] - volume * (cz * cz + cx * cx);
  properties.Izz = integral[4] + integral[5] - volume * (cx * cx + cy * cy);
  properties.Ixy = -(integral[7] - volume * cx * cy);
  properties.Iyz = -(integral[8] - volume * cy * cz);
  properties.Izx = -(integral[9] - volume * cz * cx);
  return true;
}
