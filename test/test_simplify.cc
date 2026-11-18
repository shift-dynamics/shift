/* Simplification of triangle meshes for levels of detail
(shift/core/geometry/simplify.h): the quadric simplifier keeps closed
surfaces closed and facing out, open borders and texture seams in place;
the grid simplifier reaches any count; pruning keeps a share of the parts
of a mesh and their total area */

#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/geometry/simplify.h"

/* a sphere of radius 1 about the origin: an octahedron with each face
divided into level^2 triangles, its vertices pushed onto the sphere; closed,
the faces wound outward, the vertices shared */
static void Sphere(int level, std::vector<float>& positions,
                   std::vector<uint32_t>& faces) {
  positions.clear();
  faces.clear();
  std::map<std::vector<long>, uint32_t> index;
  auto vertex = [&](double x, double y, double z) {
    const double length = std::sqrt(x * x + y * y + z * z);
    // the key: the point on the octahedron, exact in units of 1/level
    const std::vector<long> key = {std::lround(x * level * 1e3),
                                   std::lround(y * level * 1e3),
                                   std::lround(z * level * 1e3)};
    auto [it, inserted] = index.try_emplace(key, uint32_t(index.size()));
    if(inserted) {
      positions.push_back(float(x / length));
      positions.push_back(float(y / length));
      positions.push_back(float(z / length));
    }
    return it->second;
  };
  const double corners[8][3][3] = {
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},   {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},
    {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}, {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
    {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},  {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},
    {{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}}, {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
  for(const auto& c : corners) {
    // barycentric grid points (i, j) with i + j <= level
    auto point = [&](int i, int j) {
      const double a = double(level - i - j) / level;
      const double b = double(i) / level;
      const double d = double(j) / level;
      return vertex(a * c[0][0] + b * c[1][0] + d * c[2][0],
                    a * c[0][1] + b * c[1][1] + d * c[2][1],
                    a * c[0][2] + b * c[1][2] + d * c[2][2]);
    };
    for(int i = 0; i < level; i++) {
      for(int j = 0; j < level - i; j++) {
        faces.insert(faces.end(), {point(i, j), point(i + 1, j),
                                   point(i, j + 1)});
        if(i + j + 1 < level) {
          faces.insert(faces.end(), {point(i + 1, j), point(i + 1, j + 1),
                                     point(i, j + 1)});
        }
      }
    }
  }
}

/* the normal of a face (twice its area) */
static std::vector<double> FaceNormal(const std::vector<float>& positions,
                                      const uint32_t* face) {
  const float* a = &positions[3 * face[0]];
  const float* b = &positions[3 * face[1]];
  const float* c = &positions[3 * face[2]];
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  return {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
          u[0] * v[1] - u[1] * v[0]};
}

static double Area(const std::vector<float>& positions,
                   const std::vector<uint32_t>& faces) {
  double area = 0.0;
  for(size_t f = 0; f < faces.size(); f += 3) {
    const std::vector<double> n = FaceNormal(positions, &faces[f]);
    area += 0.5 * std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  }
  return area;
}

/* the volume a closed surface encloses (divergence theorem) */
static double Volume(const std::vector<float>& positions,
                     const std::vector<uint32_t>& faces) {
  double volume = 0.0;
  for(size_t f = 0; f < faces.size(); f += 3) {
    const float* a = &positions[3 * faces[f]];
    const std::vector<double> n = FaceNormal(positions, &faces[f]);
    volume += (a[0] * n[0] + a[1] * n[1] + a[2] * n[2]) / 6.0;
  }
  return volume;
}

/* each undirected edge of a closed surface is in exactly two faces */
static bool IsClosed(const std::vector<uint32_t>& faces) {
  std::map<std::pair<uint32_t, uint32_t>, int> edges;
  for(size_t f = 0; f < faces.size(); f += 3) {
    for(int e = 0; e < 3; e++) {
      uint32_t a = faces[f + e];
      uint32_t b = faces[f + (e + 1) % 3];
      edges[{std::min(a, b), std::max(a, b)}]++;
    }
  }
  for(const auto& [edge, count] : edges) {
    if(count != 2) {
      return false;
    }
  }
  return true;
}

/* the faces of a simplified unit sphere face out (within 60 degrees of the
radial direction at their centroids) and stay near the sphere: their
centroids within three times the error (relative to the box of side 2; the
error is a weighted mean of squared distances to planes, not a maximum) */
static void ExpectFacingOut(const std::vector<float>& positions,
                            const std::vector<uint32_t>& faces,
                            double error) {
  for(size_t f = 0; f < faces.size(); f += 3) {
    const std::vector<double> n = FaceNormal(positions, &faces[f]);
    double c[3] = {0.0, 0.0, 0.0};
    for(int k = 0; k < 3; k++) {
      for(int i = 0; i < 3; i++) {
        c[i] += positions[3 * faces[f + k] + i] / 3.0;
      }
    }
    const double n_length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    const double c_length = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    EXPECT_GT((n[0] * c[0] + n[1] * c[1] + n[2] * c[2]) / n_length / c_length,
              0.5);
    EXPECT_LE(1.0 - c_length, 3.0 * 2.0 * error);
  }
}

TEST(Simplify, SphereStaysClosedAndFacingOut) {
  std::vector<float> positions;
  std::vector<uint32_t> faces;
  Sphere(24, positions, faces);
  ASSERT_EQ(faces.size() / 3, 8u * 24u * 24u);
  ASSERT_TRUE(IsClosed(faces));
  const double volume = Volume(positions, faces);

  shift::core::geometry::SimplifyOptions options;
  options.target_face_count = faces.size() / 3 / 10;
  options.max_error = 0.1;
  double error = 0.0;
  const std::vector<uint32_t> simplified = shift::core::geometry::
    SimplifyQuadric(positions, {}, 0, faces, options, &error);

  EXPECT_LE(simplified.size() / 3, options.target_face_count);
  EXPECT_GT(simplified.size() / 3, options.target_face_count / 2);
  EXPECT_TRUE(IsClosed(simplified));
  EXPECT_GT(error, 0.0);
  EXPECT_LE(error, options.max_error);
  // every face still faces out, and the volume is nearly the sphere's
  ExpectFacingOut(positions, simplified, error);
  EXPECT_GT(Volume(positions, simplified), 0.9 * volume);
}

TEST(Simplify, ErrorLimitStopsEarly) {
  std::vector<float> positions;
  std::vector<uint32_t> faces;
  Sphere(24, positions, faces);
  shift::core::geometry::SimplifyOptions options;
  options.target_face_count = 8;
  options.max_error = 0.002;
  double error = 0.0;
  const std::vector<uint32_t> simplified = shift::core::geometry::
    SimplifyQuadric(positions, {}, 0, faces, options, &error);
  // a sphere has no flat parts: it stops far short of the target, with the
  // faces near the sphere
  EXPECT_GT(simplified.size() / 3, faces.size() / 3 / 4);
  EXPECT_LT(simplified.size() / 3, faces.size() / 3);
  EXPECT_LE(error, options.max_error);
  ExpectFacingOut(positions, simplified, options.max_error);
}

/* a square sheet of n x n cells in z = 0, two triangles a cell, facing +z;
with a seam, the column x = n / 2 is split into two wedges, and the
attribute of each vertex is the side (0 left, 1 right) it is on */
static void Sheet(int n, bool seam, std::vector<float>& positions,
                  std::vector<float>& side, std::vector<uint32_t>& faces) {
  positions.clear();
  side.clear();
  faces.clear();
  std::map<std::pair<int, int>, uint32_t> left, right;
  auto vertex = [&](int i, int j, int s) {
    auto& map = (seam && s == 1 && i == n / 2) ? right : left;
    auto [it, inserted] =
      map.try_emplace({i, j}, uint32_t(positions.size() / 3));
    if(inserted) {
      positions.insert(positions.end(), {float(i), float(j), 0.f});
      side.push_back(seam ? float(i < n / 2 || (i == n / 2 && s == 0) ? 0
                                                                      : 1)
                          : 0.f);
    }
    return it->second;
  };
  for(int i = 0; i < n; i++) {
    for(int j = 0; j < n; j++) {
      const int s = i < n / 2 ? 0 : 1;
      faces.insert(faces.end(), {vertex(i, j, s), vertex(i + 1, j, s),
                                 vertex(i + 1, j + 1, s)});
      faces.insert(faces.end(), {vertex(i, j, s), vertex(i + 1, j + 1, s),
                                 vertex(i, j + 1, s)});
    }
  }
}

TEST(Simplify, OpenBorderKeepsItsOutline) {
  std::vector<float> positions, side;
  std::vector<uint32_t> faces;
  Sheet(16, false, positions, side, faces);
  shift::core::geometry::SimplifyOptions options;
  options.target_face_count = 2;
  options.max_error = 1e-3;
  const std::vector<uint32_t> simplified = shift::core::geometry::
    SimplifyQuadric(positions, {}, 0, faces, options);
  // flat: it goes down to a few triangles, covering the whole square
  EXPECT_LE(simplified.size() / 3, 8u);
  EXPECT_NEAR(Area(positions, simplified), 256.0, 1e-6);
  for(size_t f = 0; f < simplified.size(); f += 3) {
    EXPECT_GT(FaceNormal(positions, &simplified[f])[2], 0.0);
  }
}

TEST(Simplify, LockedBorderStays) {
  std::vector<float> positions, side;
  std::vector<uint32_t> faces;
  Sheet(16, false, positions, side, faces);
  shift::core::geometry::SimplifyOptions options;
  options.target_face_count = 2;
  options.max_error = 1e-3;
  options.lock_border = true;
  const std::vector<uint32_t> simplified = shift::core::geometry::
    SimplifyQuadric(positions, {}, 0, faces, options);
  std::set<std::pair<int, int>> border;
  for(uint32_t v : simplified) {
    const int x = int(positions[3 * v]);
    const int y = int(positions[3 * v + 1]);
    if(x == 0 || y == 0 || x == 16 || y == 16) {
      border.insert({x, y});
    }
  }
  EXPECT_EQ(border.size(), 64u); // all 4 x 16 border vertices
  EXPECT_NEAR(Area(positions, simplified), 256.0, 1e-6);
}

TEST(Simplify, SeamStaysAndSidesDoNotMix) {
  std::vector<float> positions, side;
  std::vector<uint32_t> faces;
  Sheet(16, true, positions, side, faces);
  shift::core::geometry::SimplifyOptions options;
  options.target_face_count = 2;
  options.max_error = 1e-3;
  options.attribute_weights = {1.f};
  const std::vector<uint32_t> simplified = shift::core::geometry::
    SimplifyQuadric(positions, side, 1, faces, options);
  EXPECT_LT(simplified.size() / 3, faces.size() / 3 / 10);
  EXPECT_NEAR(Area(positions, simplified), 256.0, 1e-6);
  // no face takes vertices of both sides, and the seam stays at x = 8
  for(size_t f = 0; f < simplified.size(); f += 3) {
    const float s = side[simplified[f]];
    double left = 0.0, right = 0.0;
    for(int k = 0; k < 3; k++) {
      EXPECT_EQ(side[simplified[f + k]], s);
      const float x = positions[3 * simplified[f + k]];
      left = std::max(left, double(8.f - x));
      right = std::max(right, double(x - 8.f));
    }
    EXPECT_TRUE(s == 0.f ? right <= 0.0 : left <= 0.0);
  }
}

TEST(Simplify, GridReachesAnyCount) {
  std::vector<float> positions;
  std::vector<uint32_t> faces;
  Sphere(24, positions, faces);
  for(size_t target : {2000u, 200u, 20u}) {
    double error = 0.0;
    const std::vector<uint32_t> simplified =
      shift::core::geometry::SimplifyGrid(positions, faces, target, &error);
    EXPECT_LE(simplified.size() / 3, target);
    EXPECT_GT(simplified.size() / 3, 0u);
    EXPECT_GT(error, 0.0);
  }
}

/* parts squares of side 1, apart along x, two triangles each */
static void Cards(int parts, std::vector<float>& positions,
                  std::vector<uint32_t>& faces) {
  positions.clear();
  faces.clear();
  for(int i = 0; i < parts; i++) {
    const uint32_t v = uint32_t(positions.size() / 3);
    const float x = 3.f * float(i);
    positions.insert(positions.end(), {x, 0.f, 0.f, x + 1.f, 0.f, 0.f,
                                       x + 1.f, 1.f, 0.f, x, 1.f, 0.f});
    faces.insert(faces.end(), {v, v + 1, v + 2, v, v + 2, v + 3});
  }
}

TEST(Simplify, PruningKeepsTheArea) {
  std::vector<float> positions;
  std::vector<uint32_t> faces;
  Cards(100, positions, faces);
  std::vector<float> scaled;
  double error = 0.0;
  const std::vector<uint32_t> kept = shift::core::geometry::PruneComponents(
    positions, faces, 0.25, 7u, &scaled, &error);
  EXPECT_EQ(kept.size() / 3, 50u);
  // enlarged twice: they reach one radius (half a diagonal) farther, in
  // units of the length of the row of cards
  EXPECT_NEAR(error, std::sqrt(0.5) / 298.0, 1e-9);
  EXPECT_NEAR(Area(scaled, kept), 100.0, 1e-4);
  // each kept card is twice as large, about its own center
  for(size_t f = 0; f < kept.size(); f += 6) {
    const uint32_t v = kept[f];
    EXPECT_NEAR(scaled[3 * (v + 1)] - scaled[3 * v], 2.f, 1e-5f);
    EXPECT_NEAR(0.5f * (scaled[3 * (v + 1)] + scaled[3 * v]),
                0.5f * (positions[3 * (v + 1)] + positions[3 * v]), 1e-5f);
  }
  // the same seed always keeps the same cards; another seed others
  std::vector<float> again;
  EXPECT_EQ(shift::core::geometry::PruneComponents(positions, faces, 0.25, 7u,
                                                   &again),
            kept);
  EXPECT_NE(shift::core::geometry::PruneComponents(positions, faces, 0.25, 8u,
                                                   &again),
            kept);
}

TEST(Simplify, CompactVerticesKeepsOnlyTheUsed) {
  std::vector<uint32_t> faces = {5, 2, 7, 7, 2, 9};
  const std::vector<uint32_t> old_index =
    shift::core::geometry::CompactVertices(faces, 10);
  EXPECT_EQ(old_index, (std::vector<uint32_t>{5, 2, 7, 9}));
  EXPECT_EQ(faces, (std::vector<uint32_t>{0, 1, 2, 2, 1, 3}));
}
