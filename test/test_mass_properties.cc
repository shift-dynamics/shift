#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "shift/core/geometry/mass_properties.h"

using shift::core::geometry::CloseHoles;
using shift::core::geometry::ComputeMassProperties;
using shift::core::geometry::IsClosedSurface;
using shift::core::geometry::MassProperties;

/* a unit cube with its minimum corner at (x, y, z), triangles counterclockwise
seen from outside; corner i is at (i & 1, (i >> 1) & 1, (i >> 2) & 1) */
static void AddCube(double x, double y, double z,
                    std::vector<ksl_vec3_t>& vertices,
                    std::vector<unsigned int>& faces, bool with_top = true) {
  const unsigned int offset = static_cast<unsigned int>(vertices.size());
  for(int i = 0; i < 8; i++) {
    vertices.push_back({{x + (i & 1), y + ((i >> 1) & 1), z + ((i >> 2) & 1)}});
  }
  std::vector<unsigned int> cube = {0, 2, 3, 0, 3, 1,  // -z
                                    0, 1, 5, 0, 5, 4,  // -y
                                    2, 6, 7, 2, 7, 3,  // +y
                                    0, 4, 6, 0, 6, 2,  // -x
                                    1, 3, 7, 1, 7, 5}; // +x
  if(with_top) {
    cube.insert(cube.end(), {4, 5, 7, 4, 7, 6}); // +z
  }
  for(auto i : cube) {
    faces.push_back(offset + i);
  }
}

TEST(MassProperties, TwoCubes) {
  std::vector<ksl_vec3_t> vertices;
  std::vector<unsigned int> faces;
  AddCube(0.0, 0.0, 0.0, vertices, faces);
  AddCube(2.0, 2.0, 0.0, vertices, faces);
  ASSERT_TRUE(IsClosedSurface(vertices, faces));
  MassProperties p;
  ASSERT_TRUE(ComputeMassProperties(vertices, faces, p));
  EXPECT_NEAR(p.volume, 2.0, 1e-12);
  EXPECT_NEAR(p.centroid.x, 1.5, 1e-12);
  EXPECT_NEAR(p.centroid.y, 1.5, 1e-12);
  EXPECT_NEAR(p.centroid.z, 0.5, 1e-12);
  /* each cube: 1/6 about its centroid, plus the parallel axis terms of its
   * centroid at (-+1, -+1, 0) from the centroid of both */
  EXPECT_NEAR(p.Ixx, 2.0 * (1.0 / 6.0 + 1.0), 1e-12);
  EXPECT_NEAR(p.Iyy, 2.0 * (1.0 / 6.0 + 1.0), 1e-12);
  EXPECT_NEAR(p.Izz, 2.0 * (1.0 / 6.0 + 2.0), 1e-12);
  EXPECT_NEAR(p.Ixy, -2.0, 1e-12); // the entry, -integral of x y
  EXPECT_NEAR(p.Iyz, 0.0, 1e-12);
  EXPECT_NEAR(p.Izx, 0.0, 1e-12);
}

TEST(MassProperties, ClockwiseIsReversed) {
  std::vector<ksl_vec3_t> vertices;
  std::vector<unsigned int> faces;
  AddCube(2.0, -1.0, 3.0, vertices, faces);
  for(size_t f = 0; f < faces.size(); f += 3) {
    std::swap(faces[f + 1], faces[f + 2]);
  }
  MassProperties p;
  ASSERT_TRUE(ComputeMassProperties(vertices, faces, p));
  EXPECT_NEAR(p.volume, 1.0, 1e-12);
  EXPECT_NEAR(p.centroid.x, 2.5, 1e-12);
  EXPECT_NEAR(p.centroid.y, -0.5, 1e-12);
  EXPECT_NEAR(p.centroid.z, 3.5, 1e-12);
  EXPECT_NEAR(p.Ixx, 1.0 / 6.0, 1e-12);
  EXPECT_NEAR(p.Ixy, 0.0, 1e-12);
}

TEST(MassProperties, CloseHoles) {
  std::vector<ksl_vec3_t> vertices;
  std::vector<unsigned int> faces;
  AddCube(0.0, 0.0, 0.0, vertices, faces, false);
  AddCube(3.0, 0.0, 0.0, vertices, faces, false);
  EXPECT_FALSE(IsClosedSurface(vertices, faces));
  ASSERT_EQ(CloseHoles(vertices, faces), 2);
  ASSERT_TRUE(IsClosedSurface(vertices, faces));
  MassProperties p;
  ASSERT_TRUE(ComputeMassProperties(vertices, faces, p));
  EXPECT_NEAR(p.volume, 2.0, 1e-12);
  EXPECT_NEAR(p.centroid.x, 2.0, 1e-12);
  EXPECT_NEAR(p.centroid.z, 0.5, 1e-12);
  EXPECT_EQ(CloseHoles(vertices, faces), 0);
}
